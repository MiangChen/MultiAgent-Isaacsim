from files.assets_scripts_linux import PATH_PROJECT, PATH_ISAACSIM_ASSETS
from map.map_grid_map import GridMap
from robot.robot_jetbot import RobotCfgJetbot, RobotJetbot
from robot.robot_h1 import RobotH1, RobotCfgH1
from robot.robot_cf2x import RobotCf2x, RobotCfgCf2x
from robot.robot_swarm_manager import RobotSwarmManager

import gymnasium as gym
from isaacsim.core.api import World
from isaacsim.core.utils.prims import create_prim


def create_scene(usd_path: str, prim_path_root: str = "/World"):
    """

    Create a scene from config.(But just input usd file yet.)
    Args:
        usd_path (str): path to scene config file(use to be a .usd file)
        prim_path_root (str): path to root prim
    """
    if (
            usd_path.endswith("usd")
            or usd_path.endswith("usda")
            or usd_path.endswith("usdc")
    ):

        create_prim(
            prim_path_root,
            usd_path=usd_path,
            # scale=self.simulation.scene_scale,
            scale=[1, 1, 1],
            # translation=[0, 0, 0.81696],
            # orientation=[0.610, -0.789, -0.05184, 0.040] # wxyz, xyz还是zyx顺序不确定 
        )
    else:
        raise RuntimeError("Env file path needs to end with .usd, .usda or .usdc .")
    return

async def add_entity():
    #  加入复杂的场景

    from scene_generation.scene_generation_python.tools import ToolFunctions, AddCubeInput
    # tool_impl = ToolFunctions()
    # tools = tool_impl.tools
    #
    # input_data = AddCubeInput(prim_path="/World/Car", size=5.0, position=[1.0, 2.0, 3.0])
    # add_cube_tool = tools[0]
    # result = await add_cube_tool.ainvoke(input_data.model_dump())
    # result = await add_cube_tool.ainvoke(input_value={'prim_path': '/World/Car...ition': [1.0, 2.0, 3.0]})
    my_tools = ToolFunctions()

    # 2. 准备输入数据 (使用 Pydantic 模型)
    cube_input = AddCubeInput(
        prim_path="/World/MyAwesomeCube",
        size=50.0,
        position=[0, 15, 25.0]  # x, y, z
    )

    # 3. 直接 await 调用实例上的 async 方法
    #    这是最简单、最pythonic的方式
    result_message = await my_tools.add_cube(cube_input)

    print(f"Tool execution result: {result_message}")


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
        create_scene(usd_path=self._usd_path)
        #  await add_entity()

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
