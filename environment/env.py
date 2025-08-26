import gymnasium as gym

from map.map_grid_map import GridMap
from robot.swarm_manager import SwarmManager
from scene.scene_manager import SceneManager


class Env(gym.Env):
    """
    一个使用异步工厂模式进行初始化的Gym环境。
    请不要直接调用 Env(...)，而是使用 await Env.create(...) 来创建实例。
    """

    def __init__(self,
                 simulation_app,
                 physics_dt: float,
                 scene_manager: SceneManager,
                 swarm_manager: SwarmManager = None,
                 grid_map: GridMap = None,
                 ) -> None:

        print("Executing synchronous __init__...")

        from isaacsim.core.api import World
        # --- 存储参数和基本属性赋值 ---
        self._render = None
        self._robot_name = None
        self._current_task_name = None
        self._runner = simulation_app

        # --- 创建核心的、同步的对象 ---

        self.world = World(physics_dt=physics_dt)

        self._swarm_manager = swarm_manager
        self._scene_manager = scene_manager
        self._grid_map = grid_map
        print("Synchronous __init__ finished.")

    async def _async_init(self) -> None:
        """
        Simplified async initialization focusing only on environment concerns.
        """
        print("Starting asynchronous initialization...")

    @classmethod
    async def create(cls,
                     simulation_app,
                     physics_dt: float,
                     scene_manager: SceneManager,
                     swarm_manager: SwarmManager = None,
                     grid_map: GridMap = None,
                     ) -> "Env":
        """
        异步地创建并完全初始化一个Env实例。
        """
        instance = cls(
            simulation_app=simulation_app,
            physics_dt=physics_dt,
            swarm_manager=swarm_manager,
            scene_manager=scene_manager,
            grid_map=grid_map,
        )
        await instance._async_init()
        return instance

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
