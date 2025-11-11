# =============================================================================
# Robot H1 Module - H1 Humanoid Robot Implementation
# =============================================================================
#
# This module provides the H1 humanoid robot implementation with policy-based
# control, camera sensors, and advanced locomotion capabilities.
#
# =============================================================================

# Standard library imports
from typing import Dict

# Third-party library imports
import numpy as np

# Local project imports
from robot.controller.controller_policy_h1 import H1FlatTerrainPolicy
from log.log_manager import LogManager
from physics_engine.isaacsim_utils import Scene, ArticulationActions
from robot.body.body_h1 import BodyH1
from robot.cfg import CfgH1
from robot.robot import Robot
from robot.sensor.camera import CfgCamera, CfgCameraThird
from utils import to_torch, quat_to_yaw

# ROS2 message imports

logger = LogManager.get_logger(__name__)


class RobotH1(Robot):
    """
    H1 Robot Class.
    This class uses an asynchronous factory pattern for initialization
    because its policy controller requires asynchronous loading.
    Please use `await RobotH1.create(...)` to instantiate.
    """

    def __init__(
        self,
        cfg_robot,
    ) -> None:
        self.cfg_robot = CfgH1(**cfg_robot)
        super().__init__()
        # self.create_robot_entity()
        self.control_mode = "joint_positions"

        # 将控制器先初始化为 None，它将在异步工厂中被正确创建
        self.controller_policy: H1FlatTerrainPolicy | None = None
        self.base_command = np.zeros(3)

        self._body = BodyH1(cfg_robot=self.cfg_robot)

    def initialize(self):
        """
        Initializes the robot's connection to the simulation world entities.
        This should be called after the world is ready (e.g., after a reset).
        """
        super().initialize()
        if self.controller_policy:
            self.controller_policy.initialize(self.body.robot_articulation)
        else:
            logger.error(
                f"[Warning] RobotH1 '{self.namespace}' has no controller_policy"
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
        self._body.robot_articulation.apply_action(action)

        # obs暂时未实现
        obs = None
        return obs

    def on_physics_step(self, step_size):
        self.target_linear_velocity[2] = 0
        self.target_angular_velocity[0] = 0
        self.target_angular_velocity[1] = 0
        super().on_physics_step(step_size)

        # if self.flag_world_reset == True:
        # if self.flag_action_navigation == True:
        #     self.move_along_path()  # 每一次都计算下速度
        #     self.action = self.controller_policy.forward(
        #         step_size, self.base_command, self.body.robot_articulation
        #     )

        # self.action = self.controller_policy.forward(
        #     step_size, [0, 0, 0], self.body.robot_articulation
        # )
        # self.step(self.action)
        return
