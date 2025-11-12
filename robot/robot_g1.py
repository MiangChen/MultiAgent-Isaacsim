# =============================================================================
# Robot G1 Module - G1 Humanoid Robot Implementation
# =============================================================================
#
# This module provides the G1 humanoid robot implementation with advanced
# locomotion control, sensor integration, and autonomous navigation.
#
# =============================================================================

# Standard library imports
from typing import Dict

# Third-party library imports

# Local project imports
from physics_engine.isaacsim_utils import Scene, ArticulationActions
from robot.robot import Robot
from robot.cfg import CfgG1
from robot.body.body_g1 import BodyG1
from utils import to_torch

# ROS2 message imports
from gsi_msgs.gsi_msgs_helper import (
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
    ) -> None:
        self.cfg_robot = CfgG1(**cfg_robot)
        super().__init__()
        self._body = BodyG1(cfg_robot=self.cfg_robot)
        self.control_mode = "joint_velocities"

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

        self._body.robot_articulation.set_linear_velocities(self.target_linear_velocity)
        self._body.robot_articulation.set_angular_velocities(
            self.target_angular_velocity
        )
        # FIXME:为了让G1能运动，先用平移来代替
        # obs暂时未实现
        obs = None
        return obs

    def on_physics_step(self, step_size):
        self.target_linear_velocity[2] = 0
        self.target_angular_velocity[0] = 0
        self.target_angular_velocity[1] = 0
        super().on_physics_step(step_size)

        # if self.flag_world_reset:
        # if self.flag_action_navigation:
        #     self.step(self.action)
        # if self.is_detecting:
        #     self.detect(self, target_prim=self.target_prim)
