from typing import Dict

import torch

from controller.controller_pid import ControllerPID
from robot.sensor.camera import CfgCamera, CfgCameraThird
from map.map_grid_map import GridMap
from recycle_bin.path_planning_astar import AStar
from robot.robot import Robot
from robot.robot_trajectory import Trajectory
from robot.cfg import CfgG1
from robot.body.body_g1 import BodyG1
from utils import to_torch, quat_to_yaw

from physics_engine.isaacsim_utils import Scene, ArticulationActions

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

    def step(self, action):

        action = to_torch(action)
        if self.control_mode == "joint_positions":
            action = ArticulationActions(joint_positions=action)
        elif self.control_mode == "joint_velocities":
            action = ArticulationActions(joint_velocities=action)
        elif self.control_mode == "joint_efforts":
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
                self.step(self.action)
            if self.is_detecting:
                self.detect(self, target_prim=self.target_prim)
