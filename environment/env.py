import os

from files.assets_scripts_linux import PATH_PROJECT, PATH_ISAACSIM_ASSETS
from map.map_grid_map import GridMap
from robot.robot_jetbot import RobotCfgJetbot, RobotJetbot
from robot.robot_h1 import RobotH1, RobotCfgH1
from robot.robot_cf2x import RobotCf2x, RobotCfgCf2x
from robot.robot_swarm_manager import RobotSwarmManager

import gymnasium as gym
from isaacsim.core.api import World

import importlib.util
try:
    current_dir = os.path.dirname(os.path.abspath(__file__))
except NameError:
    current_dir = os.getcwd()
relative_path_to_module = "../mcp_extension/isaacsim.mcp_extension/isaacsim_mcp_extension/extension.py"
absolute_path = os.path.join(current_dir, relative_path_to_module)

module_name = "scene_manager"

# 创建一个模块规范 (Module Spec)
spec = importlib.util.spec_from_file_location(module_name, absolute_path)

# 根据规范创建并执行模块加载
if spec and spec.loader:
    my_extension_module = importlib.util.module_from_spec(spec)

    # 将模块添加到 sys.modules，这样其他地方也可以 import my_mcp_extension
    # sys.modules[module_name] = my_extension_module

    spec.loader.exec_module(my_extension_module)

spec = importlib.util.spec_from_file_location(module_name, absolute_path)
my_extension_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(my_extension_module)

scene_manager = my_extension_module.MCPExtension()


class Env(gym.Env):
    """
    一个使用异步工厂模式进行初始化的Gym环境。
    请不要直接调用 Env(...)，而是使用 await Env.create(...) 来创建实例。
    """

    def __init__(self, simulation_app,
                 physics_dt: float,
                 usd_path: str,
                 cell_size: float,
                 start_point: list = [0, 0, 0],
                 min_bounds: list = [-20, -20, 0],
                 max_bounds: list = [20, 20, 5],
                 occupied_cell: int = 1,
                 empty_cell=0,
                 invisible_cell=2,
                 ) -> None:

        print("Executing synchronous __init__...")

        # --- 存储参数和基本属性赋值 ---
        self._render = None
        self._robot_name = None
        self._current_task_name = None
        self._runner = simulation_app
        self._usd_path = usd_path  # 保存usd_path以供异步方法使用

        # --- 创建核心的、同步的对象 ---
        self.world = World(physics_dt=physics_dt)
        self.cell_size = cell_size
        self.map_grid = GridMap(
            cell_size=self.cell_size,
            start_point=start_point,
            min_bounds=min_bounds,
            max_bounds=max_bounds,
            occupied_cell=occupied_cell,
            empty_cell=empty_cell,
            invisible_cell=invisible_cell,
        )  # gridmap需要在robot swarm之前使用

        self.robot_swarm = RobotSwarmManager(self.world.scene, self.map_grid)

        # --- 配置机器人管理器 (这些都是同步的配置) ---
        self.robot_swarm.register_robot_class(
            robot_class_name="jetbot",
            robot_class=RobotJetbot,
            robot_class_cfg=RobotCfgJetbot,
        )  # 注册jetbot机器人
        self.robot_swarm.register_robot_class(
            robot_class_name="h1", robot_class=RobotH1, robot_class_cfg=RobotCfgH1
        )  # 注册h1机器人
        self.robot_swarm.register_robot_class(
            robot_class_name="cf2x", robot_class=RobotCf2x, robot_class_cfg=RobotCfgCf2x
        )  # 注册cf2x机器人


        print("Synchronous __init__ finished.")

    async def _async_init(self) -> None:
        """
        处理所有需要等待的异步初始化步骤。
        """
        print("Starting asynchronous initialization...")

        # 加载场景：这通常是一个需要与模拟器交互的潜在异步操作
        scene_manager.load_scene(usd_path=self._usd_path)
        scene_manager.create_robot()

        await self.robot_swarm.load_robot_swarm_cfg(
            f"{PATH_PROJECT}/files/robot_swarm_cfg.yaml"
        )
        # 激活机器人： 是一个典型的异步操作。
        self.robot_swarm.activate_robot(
            f"{PATH_PROJECT}/files/robot_swarm_active_flag.yaml"
        )

    @classmethod
    async def create(cls, simulation_app,
                     physics_dt: float,
                     usd_path: str,
                     cell_size: float,
                     start_point: list = [0, 0, 0],
                     min_bounds: list = [-20, -20, 0],
                     max_bounds: list = [20, 20, 5],
                     occupied_cell: int = 1,
                     empty_cell=0,
                     invisible_cell=2,
                     ) -> "Env":
        """
        异步地创建并完全初始化一个Env实例。
        """
        instance = cls(
            simulation_app=simulation_app,
            physics_dt=physics_dt,
            usd_path=usd_path,
            cell_size=cell_size,
            start_point=start_point,
            min_bounds=min_bounds,
            max_bounds=max_bounds,
            occupied_cell=occupied_cell,
            empty_cell=empty_cell,
            invisible_cell=invisible_cell,
        )

        await instance._async_init()

        return instance

    def reset(self):
        self.world.reset()
        self.init_robot()  # 似乎要在这里先初始化机器人, 才能在grid map中找到机器人的障碍
        self.map_grid.initialize()
        print("reset env & init grid map success")
        return

    def step(self, action):
        self.world.step()
        return

    def init_robot(self):
        for robot_class in self.robot_swarm.robot_active.keys():
            for robot in self.robot_swarm.robot_active[robot_class]:
                robot.initialize()
                robot.flag_world_reset = True
