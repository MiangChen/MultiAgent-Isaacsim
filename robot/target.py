# =============================================================================
# Robot Target Module - Target Object Implementation
# =============================================================================
#
# This module provides target object implementation for robot navigation
# and interaction scenarios, including target tracking and manipulation.
#
# =============================================================================

# Standard library imports
from typing import Dict, List, Tuple

# Third-party library imports
import torch
import numpy as np

# Local project imports
from physics_engine.isaacsim_utils import Scene, ArticulationActions
from robot.robot import Robot
from robot.robot_trajectory import Trajectory
from robot.cfg.cfg_target import CfgTarget
from robot.body.body_target import BodyTarget
from robot.sensor.camera import CfgCamera, CfgCameraThird
from utils import to_torch, quat_to_yaw

# from path_planning.path_planning_astar import AStar

# Custom ROS message imports
from gsi_msgs.gsi_msgs_helper import (
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
            scene_manager=None,
        )
        self.body = BodyTarget(cfg_robot=self.cfg_robot, scene=scene)
        self.control_mode = "joint_velocities"

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

        # obs暂时未实现
        obs = None
        return obs

    def on_physics_step(self, step_size):
        super().on_physics_step(step_size)

        if self.flag_world_reset:
            if self.flag_action_navigation:
                self.step(self.action)
            if self.is_detecting:
                self.detect(self, target_prim=self.target_prim)
