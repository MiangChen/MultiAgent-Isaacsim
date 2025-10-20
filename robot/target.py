from typing import Dict, List, Tuple

import torch
import numpy as np

from controller.controller_pid import ControllerPID
from robot.sensor.camera import CfgCamera, CfgCameraThird
from map.map_grid_map import GridMap

# from path_planning.path_planning_astar import AStar
from robot.robot import Robot
from robot.robot_trajectory import Trajectory
from robot.cfg.cfg_target import CfgTarget
from robot.body.body_target import BodyTarget
from utils import to_torch, quat_to_yaw

from physics_engine.isaacsim_utils import Scene, ArticulationActions

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
        cfg_robot={},
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

    def stop_moving(self):
        self.is_moving = False
        self.path = []
        self.path_index = 0

        zero_velocity = torch.zeros((1, 3), dtype=torch.float32)
        self.body.robot_articulation.set_linear_velocities(zero_velocity)
        self.body.robot_articulation.set_angular_velocities(zero_velocity)

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
                self.detect(self, target_prim=self.target_prim)
