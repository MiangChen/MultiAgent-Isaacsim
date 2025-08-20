from files.assets_scripts_linux import PATH_PROJECT, PATH_ISAACSIM_ASSETS
from map.map_grid_map import GridMap
from robot.robot_jetbot import RobotCfgJetbot, RobotJetbot
from robot.robot_h1 import RobotH1, RobotCfgH1
# from robot.robot_cf2x import RobotCf2x, RobotCfgCf2x
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


class Env(gym.Env):

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
        self._render = None
        self._robot_name = None
        self._current_task_name = None
        self._runner = simulation_app
        self.world = World(physics_dt=physics_dt)


        create_scene(usd_path=usd_path)
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
        self.robot_swarm.register_robot_class(
            robot_class_name="jetbot",
            robot_class=RobotJetbot,
            robot_class_cfg=RobotCfgJetbot,
        )  # 注册jetbot机器人
        self.robot_swarm.register_robot_class(
            robot_class_name="h1", robot_class=RobotH1, robot_class_cfg=RobotCfgH1
        )  # 注册h1机器人
#        self.robot_swarm.register_robot_class(
#            robot_class_name="cf2x", robot_class=RobotCf2x, robot_class_cfg=RobotCfgCf2x
#        )  # 注册cf2x机器人
        # TODO 更新路径的设置
        self.robot_swarm.load_robot_swarm_cfg(
            f"{PATH_PROJECT}/files/robot_swarm_cfg.yaml"
        )

        self.robot_swarm.activate_robot(
            f"{PATH_PROJECT}/files/robot_swarm_active_flag.yaml"
        )  # 统一在这里加入机器人

        print("init success")

        return

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
