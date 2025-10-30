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
import torch

# Local project imports
from physics_engine.isaacsim_utils import Scene, ArticulationActions
from robot.robot import Robot
from robot.robot_trajectory import Trajectory
from robot.cfg import CfgG1
from robot.body.body_g1 import BodyG1
from robot.sensor.camera import CfgCamera, CfgCameraThird
from utils import to_torch, quat_to_yaw

# Custom ROS message imports

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
        # cfg_camera: CfgCamera = None,
        # cfg_camera_third_person: CfgCameraThird = None,
        scene: Scene = None,
        scene_manager=None,
    ) -> None:
        self.cfg_robot = CfgG1(**cfg_robot)
        super().__init__(
            # cfg_camera,
            # cfg_camera_third_person,
            scene=scene,
            scene_manager=None,
        )
        self.body = BodyG1(cfg_robot=self.cfg_robot, scene=scene)
        self.control_mode = "joint_velocities"

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

        self.counter += 1

        # if self.flag_world_reset:
        if self.flag_action_navigation:
            self.step(self.action)
        if self.is_detecting:
            self.detect(self, target_prim=self.target_prim)
