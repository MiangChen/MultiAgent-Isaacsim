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
from robot.robot_trajectory import Trajectory
from robot.sensor.camera import CfgCamera, CfgCameraThird
from utils import to_torch, quat_to_yaw

# Custom ROS message imports

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
        cfg_robot: Dict = {},
        scene: Scene = None,
        scene_manager=None,
    ) -> None:
        self.cfg_robot = CfgH1(**cfg_robot)
        super().__init__(
            scene,
            scene_manager,
        )
        # self.create_robot_entity()
        self.control_mode = "joint_positions"
        self.scene_manager = scene_manager

        # 将控制器先初始化为 None，它将在异步工厂中被正确创建
        self.controller_policy: H1FlatTerrainPolicy | None = None
        self.base_command = np.zeros(3)

        self.counter = 0
        self.pub_period = 50
        self.previous_pos = None
        self.movement_threshold = (
            0.1  # 移动时，如果两次检测之间的移动距离小于这个阈值，那么就会判定其为异常
        )
        self.body = BodyH1(cfg_robot=self.cfg_robot, scene=self.scene)
        if self.cfg_robot.disable_gravity:
            self.scene_manager.disable_gravity_for_hierarchy(self.cfg_robot.path_prim_robot)

    @classmethod
    async def create(
        cls,
        cfg_robot: CfgH1,
        # cfg_camera: CfgCamera = None,
        # cfg_camera_third_person: CfgCameraThird = None,
        scene: Scene = None,
        scene_manager=None,
    ) -> "RobotH1":
        """
        Asynchronously creates and fully initializes a RobotH1 instance,
        including its asynchronous policy controller.
        """
        instance = cls(
            cfg_robot=cfg_robot,
            # cfg_camera=cfg_camera,
            # cfg_camera_third_person=cfg_camera_third_person,
            scene=scene,
            scene_manager=scene_manager,
        )

        # 异步地创建并加载 H1FlatTerrainPolicy 控制器
        # instance.controller_policy = await H1FlatTerrainPolicy.create(
        #     prim_path=instance.cfg_robot.path_prim_swarm
        # )
        return instance

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
        self.body.robot_articulation.apply_action(action)

        # obs暂时未实现
        obs = None
        return obs

    def on_physics_step(self, step_size):
        self.vel_linear[2] = 0
        self.vel_angular[0] = 0
        self.vel_angular[1] = 0
        super().on_physics_step(step_size)

        self.counter += 1

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
