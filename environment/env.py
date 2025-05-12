from map.map_grid_map import GridMap
from robot.robot_jetbot import RobotCfgJetbot, RobotJetbot
from robot.robot_h1 import RobotH1, RobotCfgH1
from robot.robot_cf2x import RobotCf2x, RobotCfgCf2x
from robot.robot_swarm_manager import RobotSwarmManager
from files.assets_scripts_linux import PATH_PROJECT, PATH_ISAACSIM_ASSETS

import gymnasium as gym
import numpy as np
from isaacsim.core.api import World
from isaacsim.core.utils.viewports import create_viewport_for_camera, set_camera_view
import isaacsim.core.utils.prims as prims_utils
from isaacsim.core.utils.prims import create_prim


def create_scene(config_json_path: str, prim_path_root: str = "background"):
    """

    Create a scene from config.(But just input usd file yet.)
    Args:
        config_json_path (str): path to scene config file(use to be a .usd file)
        prim_path_root (str): path to root prim

    Returns:
        config_json_path (str): path to config file
        world_prim_path (str): path to world prim
    """
    world_prim_path = "/" + prim_path_root
    if (
        config_json_path.endswith("usd")
        or config_json_path.endswith("usda")
        or config_json_path.endswith("usdc")
    ):
        # Add usd directly
        return config_json_path, world_prim_path
    raise RuntimeError("Env file path needs to end with .usd, .usda or .usdc .")


class Env(gym.Env):

    def __init__(self, simulation_app=None, usd_path: str = None) -> None:
        self._render = None
        self._robot_name = None
        self._current_task_name = None
        # self._validate()
        # from isaacsim import SimulationApp
        # self.simulation_app = SimulationApp({"headless": False})  # we can also run as headless.

        self._runner = simulation_app
        self.world = World(physics_dt=1 / 200)
        self.world.scene.add_default_ground_plane()  # 添加地面

        source, prim_path = create_scene(
            usd_path,
            # prim_path_root=f"World",  ## 注意前面不要有/
        )

        create_prim(
            prim_path,
            usd_path=source,
            # scale=self.simulation.scene_scale,
            scale=[1, 1, 1],
            # translation=[0, 0, 0.81],
            # orientation=[0.62, -0.774, -0.09, 0.08]
        )

        self.cell_size = 0.2
        self.map_grid = GridMap(
            min_bounds=[-10, -10, 0], max_bounds=[10, 10, 10], cell_size=self.cell_size
        )  #  gridmap需要再robot swarm之前使用

        self.robot_swarm = RobotSwarmManager(self.world.scene, self.map_grid)
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
        # TODO 更新路径的设置，
        self.robot_swarm.load_robot_swarm_cfg(
            f"{PATH_PROJECT}/files/robot_swarm_cfg.yaml"
        )

        # self.robot_swarm.create_robot(robot_class_name='jetbot', id=0, position=(0.0, 0.0, 0.0),
        #                               orientation=(0.0, 0.0, 0.0, 1),
        #                               robot_class_cfg=JetbotCfg)  # 机器人名字和cfg是对应的, 所以直接输入一个cfg就可以了

        self.robot_swarm.activate_robot(
            f"{PATH_PROJECT}/files/robot_swarm_active_flag.yaml"
        )  # 统一在这里加入机器人

        # 添加相机, 为相机添加viewport
        self.camera_prim_path = "/World/camera_test"
        xform_path = self.camera_prim_path + "/xform_camera"
        # prims_utils.delete_prim(xform_path) # 如果已经有了，要先删除
        # prims_utils.delete_prim(camera_prim_path)
        prims_utils.create_prim(
            prim_path=self.camera_prim_path,
            prim_type="Camera",
            position=np.array([5, 0, 50.0]),
        )
        # 创建一个 Xform 原始来控制相机的位置和旋转
        xform_prim = prims_utils.create_prim(prim_path=xform_path, prim_type="Xform")
        create_viewport_for_camera(
            viewport_name="/test_camera",
            camera_prim_path=self.camera_prim_path,
            width=720 / 2,
            height=720 / 2,
            position_x=800,  ## 设置他在IsaacSimAPP中的相对的位置
            position_y=400,
        )

        ## 可以设置相机的观察位置
        set_camera_view(
            eye=np.array([5, 0, 50]),
            target=np.array([5, 0, 0]),
            camera_prim_path=self.camera_prim_path,
        )
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
