# =============================================================================
# Robot Drone CF2X Module - Crazyflie 2.X Drone Implementation
# =============================================================================
#
# This module provides the Crazyflie 2.X drone robot implementation with
# specialized flight control, camera sensor, and navigation capabilities.
#
# =============================================================================

# Standard library imports
from typing import Dict

# Third-party library imports
import numpy as np
import carb

# Local project imports
from log.log_manager import LogManager
from physics_engine.isaacsim_utils import Scene, Articulation
from physics_engine.omni_utils import omni

from simulation.robot.robot import Robot
from simulation.robot.cfg import CfgDroneCf2X
from simulation.robot.body.body_drone_cf2x import BodyDroneCf2X

logger = LogManager.get_logger(__name__)


class RobotCf2x(Robot):

    def __init__(
            self,
            cfg_robot: Dict = {},
    ) -> None:
        self.cfg_robot = CfgDroneCf2X(**cfg_robot)
        super().__init__()

        self._body = BodyDroneCf2X(cfg_robot=self.cfg_robot)

        # 无人机基本属性
        self.position = np.array(
            getattr(cfg_robot, "_position", [0.0, 0.0, 0.0]), dtype=np.float32
        )

    def initialize(self):
        """初始化无人机"""
        return

    def get_ground_height_with_raycast(self, current_position: np.ndarray) -> float:
        """
        使用光线投射来精确获取机器人正下方的地面高度。

        Args:
            current_position (np.ndarray): 机器人当前的 [x, y, z] 世界坐标。

        Returns:
            float: 检测到的地面Z坐标。如果未检测到地面，则返回None。
        """
        # 1. 获取 Isaac Sim 的物理场景接口

        physx_interface = omni.physx.get_physx_scene_query_interface()

        # 2. 定义光线投射的起点和方向
        #    起点应该在机器人当前位置的正上方，以避免光线从机器人内部开始
        ray_origin = carb.Float3(
            current_position[0], current_position[1], current_position[2] + 1.0
        )
        #    方向是垂直向下
        ray_direction = carb.Float3(0, 0, -1)

        # 3. 设置最大探测距离
        max_distance = 100.0  # 例如，最大向下探测100米

        # 4. 执行光线投射
        #    hit_report_multiple 会返回所有碰到的物体
        hit = physx_interface.raycast_closest(ray_origin, ray_direction, max_distance)

        # 5. 处理结果
        if hit["hit"]:
            # "position" 字段是光线与物体碰撞点的精确世界坐标
            ground_position = hit["position"]
            print(f"DEBUG: Raycast hit ground at Z={ground_position[2]:.3f}")
            return ground_position[2]
        else:
            print("WARNING: Raycast did not hit any surface below the robot.")
            return None  # 或者返回一个默认的安全高度，比如 0.0

    def on_physics_step(self, step_size):
        super().on_physics_step(step_size)
