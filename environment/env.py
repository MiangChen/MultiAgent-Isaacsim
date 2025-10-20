# =============================================================================
# Environment Module - Simulation Environment Management
# =============================================================================
#
# This module provides the main environment class for managing the Isaac Sim
# simulation environment, integrating world, scene, robot swarm, and mapping
# components.
#
# =============================================================================

# Third-party library imports
import gymnasium as gym

# Local project imports
from map.map_grid_map import GridMap
from physics_engine.isaacsim_utils import World
from robot.swarm_manager import SwarmManager
from scene.scene_manager import SceneManager


class Env(gym.Env):
    def __init__(
        self,
        simulation_app,
        world: World,
        scene_manager: SceneManager = None,
        swarm_manager: SwarmManager = None,
        grid_map: GridMap = None,
    ) -> None:
        # --- 存储参数和基本属性赋值 ---
        self._runner = simulation_app

        # --- 创建核心的、同步的对象 ---
        self.world = world
        self._swarm_manager = swarm_manager
        self._scene_manager = scene_manager
        self._grid_map = grid_map

    async def initialize_async(self) -> None:
        """执行所有必要的异步初始化步骤。"""
        print("Starting environment's asynchronous initialization...")
        # ... 任何需要 await 的操作 ...
        print("Environment's asynchronous initialization finished.")

    def reset(self) -> None:
        self.world.reset()
        self.init_robot()  # init robot's camera
        self._grid_map.initialize()
        print("reset env & init grid map success")
        return

    def step(self, action=None):
        self.world.step()
        return

    def init_robot(self):
        for robot_class in self._swarm_manager.robot_active.keys():
            for robot in self._swarm_manager.robot_active[robot_class]:
                robot.initialize()
                robot.flag_world_reset = True
