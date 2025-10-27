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
        cfg_robot,
        scene: Scene = None,
    ) -> None:

        self.cfg_robot = CfgTarget(**cfg_robot)
        super().__init__(
            scene=scene,
            scene_manager=None,
        )
        self.body = BodyTarget(cfg_robot=self.cfg_robot, scene=scene)
        self.control_mode = "joint_velocities"
        self.is_moving = False
        self.path = [
            self.cfg_robot.base_pos,
            self.cfg_robot.mid_pos,
            self.cfg_robot.target_pos,
        ]
        self.path_index = 0

    def start_moving(self) -> None:
        self.is_moving = True

    def stop_moving(self):
        self.is_moving = False
        self.path = []
        self.path_index = 0
        zero_velocity = torch.zeros((1, 3), dtype=torch.float32)
        self.body.robot_articulation.set_linear_velocities(zero_velocity)
        self.body.robot_articulation.set_angular_velocities(zero_velocity)

    def execute_frame_skill(
        self,
    ) -> None:
        if self.is_moving:
            if self.node_controller_mpc.has_reached_goal:
                self.navigate_to(list(self.path[self.path_index]))
                self.path_index += 1
                self.path_index %= len(self.path)

    def on_physics_step(self, step_size):
        super().on_physics_step(step_size)
