from typing import Dict

import torch

from controller.controller_pid import ControllerPID
from robot.sensor.camera import CfgCamera, CfgCameraThird
from map.map_grid_map import GridMap
from path_planning.path_planning_astar import AStar
from robot.robot import Robot
from robot.robot_trajectory import Trajectory
from robot.cfg import CfgG1
from robot.body.body_g1 import BodyG1
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

class RobotG1(Robot):
    def __init__(
        self,
        cfg_robot: Dict = {},
        # cfg_camera: CfgCamera = None,
        # cfg_camera_third_person: CfgCameraThird = None,
        scene: Scene = None,
        map_grid: GridMap = None,
        scene_manager=None,
    ) -> None:
        self.cfg_robot = CfgG1(**cfg_robot)
        super().__init__(
            # cfg_camera,
            # cfg_camera_third_person,
            scene=scene,
            map_grid=map_grid,
            scene_manager=None,
        )
        self.body = BodyG1(cfg_robot=self.cfg_robot, scene=scene)
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

    def _move_to(self, target_pos):

        import numpy as np

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

        action = to_torch(action)
        if self.control_mode == 'joint_positions':
            action = ArticulationActions(joint_positions=action)
        elif self.control_mode == 'joint_velocities':
            action = ArticulationActions(joint_velocities=action)
        elif self.control_mode == 'joint_efforts':
            action = ArticulationActions(joint_efforts=action)
        else:
            raise NotImplementedError

        self.body.robot_articulation.set_linear_velocities(self.linear_velocity)
        self.body.robot_articulation.set_angular_velocities(self.angular_velocity)
        # FIXME:为了让G1能运动，先用平移来代替
        # obs暂时未实现
        obs = None
        return obs

    def on_physics_step(self, step_size):
        super().on_physics_step(step_size)

        self._publish_status_pose()
        self.counter += 1

        if self.flag_world_reset:
            if self.flag_action_navigation:
                self.move_along_path()  # 每一次都计算下速度
                self.step(self.action)
            if self.is_detecting:
                self.detect(self, target_prim = self.target_prim)