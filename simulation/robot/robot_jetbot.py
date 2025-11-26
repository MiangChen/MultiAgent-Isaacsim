# =============================================================================
# Robot Jetbot Module - Jetbot Robot Implementation
# =============================================================================
#
# This module provides the Jetbot robot implementation with
# camera sensor, and navigation capabilities within the Isaac Sim environment.
#
# =============================================================================

# Standard library imports
from typing import Dict

# Third-party library imports

# Local project imports
from physics_engine.isaacsim_utils import Scene, ArticulationActions
from simulation.robot.robot import Robot
from simulation.robot.cfg import CfgJetbot
from simulation.robot.body.body_jetbot import BodyJetbot
from utils import to_torch

# ROS2 message imports
from ros_msg.gsi_msgs_helper import (
    Plan,
    RobotFeedback,
    SkillInfo,
    Parameter,
    VelTwistPose,
)


class RobotJetbot(Robot):
    def __init__(
        self,
        cfg_robot: Dict = {},
    ) -> None:
        self.cfg_robot = CfgJetbot(**cfg_robot)
        super().__init__()
        self._body = BodyJetbot(cfg_robot=self.cfg_robot)
        self.control_mode = "joint_velocities"

    def initialize(self) -> None:
        super().initialize()
        return

    def on_physics_step(self, step_size):
        self.target_linear_velocity[2] = 0
        self.target_angular_velocity[0] = 0
        self.target_angular_velocity[1] = 0
        super().on_physics_step(step_size)

        return

    def step(self, action):
        action = to_torch(action)
        if self.control_mode == "joint_position":
            action = ArticulationActions(joint_positions=action)
        elif self.control_mode == "joint_velocities":
            action = ArticulationActions(joint_velocities=action)
        elif self.control_mode == "joint_efforts":
            action = ArticulationActions(joint_efforts=action)
        else:
            raise NotImplementedError
        self.body.robot_articulation.apply_action(action)

        # obs暂时未实现
        obs = None
        return obs
