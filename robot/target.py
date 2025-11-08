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

# Local project imports
from physics_engine.isaacsim_utils import Scene, ArticulationActions
from robot.robot import Robot
from robot.cfg.cfg_target import CfgTarget
from robot.body.body_target import BodyTarget
from robot.skill.base.navigation.navigate_to import navigate_to
from scene.scene_manager import SceneManager

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
        scene_manager: SceneManager = None,
    ) -> None:

        self.cfg_robot = CfgTarget(**cfg_robot)
        super().__init__(
            scene_manager=scene_manager,
        )
        self.body = BodyTarget(cfg_robot=self.cfg_robot)
        self.control_mode = "joint_velocities"
        self.path = self.cfg_robot.move_path
