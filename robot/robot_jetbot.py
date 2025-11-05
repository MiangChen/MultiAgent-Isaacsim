# =============================================================================
# Robot Jetbot Module - Jetbot Robot Implementation
# =============================================================================
#
# This module provides the Jetbot robot implementation with
# camera sensors, and navigation capabilities within the Isaac Sim environment.
#
# =============================================================================

# Standard library imports
from typing import Dict

# Third-party library imports
import torch

# Local project imports
from physics_engine.isaacsim_utils import Scene, ArticulationActions
from robot.robot import Robot
from robot.cfg import CfgJetbot
from robot.body.body_jetbot import BodyJetbot
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


class RobotJetbot(Robot):
    def __init__(
        self,
        cfg_robot: Dict = {},
        # cfg_camera: CfgCamera = None,
        # cfg_camera_third_person: CfgCameraThird = None,
        scene: Scene = None,
        scene_manager=None,
    ) -> None:
        self.cfg_robot = CfgJetbot(**cfg_robot)
        super().__init__(
            # cfg_camera,
            # cfg_camera_third_person,
            scene=scene,
            scene_manager=scene_manager,
        )
        self.body = BodyJetbot(cfg_robot=self.cfg_robot, scene=scene)
        self.control_mode = "joint_velocities"

        self.counter = 0
        self.pub_period = 50
        self.previous_pos = None
        self.movement_threshold = (
            0.1  # 移动时，如果两次检测之间的移动距离小于这个阈值，那么就会判定其为异常
        )
        if self.cfg_robot.disable_gravity:
            self.scene_manager.disable_gravity_for_hierarchy(self.cfg_robot.path_prim_robot)

    def initialize(self) -> None:
        super().initialize()
        return

    def on_physics_step(self, step_size):
        self.vel_linear[2] = 0
        self.vel_angular[0] = 0
        self.vel_angular[1] = 0
        super().on_physics_step(step_size)


        # if self.flag_world_reset:
        # if self.flag_action_navigation:
        #     self.step(self.action)

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
