from typing import Dict, List, Tuple

import torch
import numpy as np

from controller.controller_pid import ControllerPID
from robot.sensor.camera import CfgCamera, CfgCameraThird
from map.map_grid_map import GridMap
from path_planning.path_planning_astar import AStar
from robot.robot import Robot
from robot.robot_trajectory import Trajectory
from robot.cfg.cfg_target import CfgTarget
from robot.body.body_target import BodyTarget
from utils import to_torch, quat_to_yaw

from isaacsim.core.api.scenes import Scene
from isaacsim.core.utils.types import ArticulationActions

from gsi2isaacsim.gsi_msgs_helper import (
    Plan,
    RobotFeedback,
    SkillInfo,
    Parameter,
    VelTwistPose,
)

class Target(Robot):
    def __init__(
        self,
        cfg_robot = {},
        # cfg_camera: CfgCamera = None,
        # cfg_camera_third_person: CfgCameraThird = None,
        scene: Scene = None,
        map_grid: GridMap = None,
        scene_manager=None,
    ) -> None:

        #

        self.path = None
        self.path_index = None
        self.cfg_robot = CfgTarget(**cfg_robot)
        super().__init__(
            # cfg_camera,
            # cfg_camera_third_person,
            scene=scene,
            map_grid=map_grid,
            scene_manager=None,
        )
        self.body = BodyTarget(cfg_robot=self.cfg_robot, scene=scene)
        self.control_mode = "joint_velocities"
        # # self.scene.add(self.robot)  # 需要再考虑下, scene加入robot要放在哪一个class中, 可能放在scene好一些
        self.pid_distance = ControllerPID(1, 0.1, 0.01, target=0)
        self.pid_angle = ControllerPID(10, 0, 0.1, target=0)

        self.counter = 0
        self.pub_period = 50
        self.previous_pos = None
        self.movement_threshold = (
            0.1  # 移动时，如果两次检测之间的移动距离小于这个阈值，那么就会判定其为异常
        )

        self.is_moving = False

    def start_moving(self):

        self.is_moving = True
        path1 = self.navigate_to(self.cfg_robot.base_pos, self.cfg_robot.mid_pos)
        path2 = self.navigate_to(self.cfg_robot.mid_pos, self.cfg_robot.target_pos)
        path3 = self.navigate_to(self.cfg_robot.target_pos, self.cfg_robot.base_pos)
        path = path1[:-1:] + path2[:-1:] + path3[:-1:]
        self.move_along_path(path)

    def stop_moving(self):
        self.is_moving = False
        self.path = []
        self.path_index = 0

        zero_velocity = torch.zeros((1, 3), dtype=torch.float32)
        self.body.robot_articulation.set_linear_velocities(zero_velocity)
        self.body.robot_articulation.set_angular_velocities(zero_velocity)

    def navigate_to(
        self,
        pos_robot: List = None,
        pos_target: List = None,
        load_from_file: bool = True,
        **kwargs
    ) -> List[Tuple]:

        # 小车/人形机器人的导航只能使用2d的
        if pos_target[2] != 0:
            raise ValueError("The z-axis height of the robot must be on the plane")

        self.nav_begin = np.array(pos_robot)
        self.nav_end = np.array(pos_target)
        self.nav_dist = self._calc_dist(self.nav_begin, self.nav_end)

        pos_index_target = self.map_grid.compute_index(self.nav_end)
        pos_index_robot = self.map_grid.compute_index(self.nav_begin)
        pos_index_robot[-1] = 0  # todo : 这也是因为机器人限制导致的

        # 用于把机器人对应位置的设置为空的, 不然会找不到路线
        if load_from_file:
            grid_map = np.load("./value_map.npy")
            pos_map = np.load("./pos_map.npy")
        else:
            grid_map = self.map_grid.value_map
            np.save("./value_map.npy", grid_map)
            pos_map = self.map_grid.pos_map
            np.save("./pos_map.npy", pos_map)

        grid_map[pos_index_robot] = self.map_grid.empty_cell

        planner = AStar(
            grid_map,
            obs_value=1.0,
            free_value=0.0,
            directions="eight",
            penalty_factor=20,
        )
        path = planner.find_path(
            tuple(pos_index_robot), tuple(pos_index_target), render=True
        )

        real_path = np.zeros_like(path, dtype=np.float32)
        for i in range(path.shape[0]):  # 把index变成连续实际世界的坐标
            real_path[i] = self.map_grid.pos_map[tuple(path[i])]
            real_path[i][-1] = 0

        return real_path

    def move_along_path(self, path: list = None, flag_reset: bool = False) -> bool:
        """
        让机器人沿着一个list的路径点运动
        需求: 在while外面 能够记录已经到达的点, 每次到达某个目标点的 10cm附近,就认为到了, 然后准备下一个点

        """
        self.path_index = 0
        self.path = path

        if self.path_index < len(self.path):  # 当index==len的时候, 就已经到达目标了
            flag_reach = self.move_to(self.path[self.path_index])
            if flag_reach:
                self.path_index += 1
                if self.path_index >= len(self.path):
                    self.path_index = 0
            return False
        else:
            self.flag_action_navigation = False
            self.state_skill_complete = True
            return True

    def _move_to(self, target_pos):

        pos, quat = self.body.get_world_poses()  # type np.array
        target_pos = to_torch(target_pos, device=pos.device)

        if self.counter % self.pub_period == 0:
            self._publish_feedback(params=self._params_from_pose(pos, quat),
                                   progress=self._calc_dist(pos, self.nav_end) * 100 / self.nav_dist)

        if torch.linalg.norm(target_pos[0:2] - pos[0:2]) < 0.1:
            zero_velocity = torch.zeros((1, 3), dtype=torch.float32)
            self.body.robot_articulation.set_linear_velocities(zero_velocity)
            self.body.robot_articulation.set_angular_velocities(zero_velocity)
            self._publish_feedback(params=self._params_from_pose(pos, quat), progress=100)
            return True  # 已经到达目标点附近10cm, 停止运动

        delta = target_pos - pos
        delta_x, delta_y = delta[0], delta[1]
        x_control = self.pid_distance.compute(delta_x, dt=1 / 60)
        y_control = self.pid_distance.compute(delta_y, dt=1 / 60)
        self.linear_velocity = torch.tensor([[x_control, y_control, 0]], dtype=torch.float32)

        yaw = quat_to_yaw(quat)
        robot_to_target_angle = np.arctan2(target_pos[1] - pos[1], target_pos[0] - pos[0])
        delta_angle = robot_to_target_angle - yaw
        angular_control = self.pid_angle.compute(delta_angle, dt=1 / 60)
        self.angular_velocity = torch.tensor([[0.0, 0.0, angular_control]], dtype=torch.float32)

        return False

    def step(self, action):

        self.body.robot_articulation.set_linear_velocities(self.linear_velocity)
        self.body.robot_articulation.set_angular_velocities(self.angular_velocity)
        # obs暂时未实现
        obs = None
        return obs

    def on_physics_step(self, step_size):
        super().on_physics_step(step_size)

        self._publish_status_pose()

        if self.flag_world_reset:
            if self.flag_action_navigation:
                self.move_along_path()  # 每一次都计算下速度
                self.step(self.action)
            if self.is_detecting:
                self.detect(self, target_prim = self.target_prim)