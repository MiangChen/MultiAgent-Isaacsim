import os
from typing import Any

import gymnasium as gym

from scene import Simulator

from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.core.utils.nucleus import get_assets_root_path

from isaacsim.core.api.controllers import BaseController

from isaacsim.core.utils.types import ArticulationAction

from isaacsim.robot.wheeled_robots.robots import WheeledRobot

from isaacsim.core.utils.prims import create_prim

from grid_map import GridMap

from jetbot_config import JetbotCfg, Jetbot
import numpy as np

from robot_swarm import RobotSwarmManager

def create_scene(config_json_path: str, prim_path_root: str = 'background'):
    """

    Create a scene from config.(But just input usd file yet.)
    Args:
        config_json_path (str): path to scene config file(use to be a .usd file)
        prim_path_root (str): path to root prim

    Returns:
        config_json_path (str): path to config file
        world_prim_path (str): path to world prim
    """
    world_prim_path = '/' + prim_path_root
    if config_json_path.endswith('usd') or config_json_path.endswith('usda') or config_json_path.endswith('usdc'):
        # Add usd directly
        return config_json_path, world_prim_path
    raise RuntimeError('Env file path needs to end with .usd, .usda or .usdc .')


class Env(gym.Env):
    """
    Gym Env for a single environment with a single learning agent.
    ----------------------------------------------------------------------
    """

    RESET_INFO_TASK_RUNTIME = 'task_runtime'

    def __init__(self, simulation_app: Simulator) -> None:
        self._render = None
        self._robot_name = None
        self._current_task_name = None
        # self._validate()
        # from isaacsim import SimulationApp
        # self.simulation_app = SimulationApp({"headless": False})  # we can also run as headless.
        print("init success")

        self._runner = simulation_app
        self.world = World()
        # self.world.scene.add_default_ground_plane()
        # 加载复杂场景
        # usd_path = './scene/CityDemopack/World_CityDemopack.usd'
        usd_path = './scene/simple_city.usd'
        source, prim_path = create_scene(
            os.path.abspath(usd_path),
            prim_path_root=f'World',  ## 注意前面不要有/
        )

        create_prim(
            prim_path,
            usd_path=source,
            # scale=self.simulation.scene_scale,
            scale=[1, 1, 1],
            # translation=[self.runtime.env.offset[idx] + i for idx, i in enumerate(self.runtime.scene_position)],
        )

        # assets_root_path = get_assets_root_path()
        # if assets_root_path is None:
        #     carb.log_error("Could not find nucleus server with /Isaac folder")
        # jet_robot_asset_path = assets_root_path + "/Isaac/Robots/Jetbot/jetbot.usd"

        self.robot_swarm = RobotSwarmManager(self.world.scene)
        self.robot_swarm.register_robot_class(robot_class_name='jetbot', robot_class=Jetbot, robot_class_cfg=JetbotCfg)  # 注册jetbot机器人
        self.robot_swarm.load_robot_swarm_cfg("./robot_swarm_cfg.yaml")
        # self.robot_swarm.create_robot(robot_class_name='jetbot', id=0, position=(0.0, 0.0, 0.0),
        #                               orientation=(0.0, 0.0, 0.0, 1),
        #                               robot_class_cfg=JetbotCfg)  # 机器人名字和cfg是对应的, 所以直接输入一个cfg就可以了

        self.robot_swarm.activate_robot("./robot_swarm_active_flag.yaml") # 统一在这里加入机器人

        # self.robot = Jetbot(
        #     JetbotCfg(
        #         position=np.array([-1, -1, 0]),
        #         orientation=np.array([0, 0, 0, 1])
        #     ),
        #     scene=self.world.scene
        # )

        # 加载jetbot机器人
        # self.world.scene.add(self.jetbot_robot)
        #
        # self.add_jetbot()

        # 添加相机

        # 为相机添加iewport

        import isaacsim.core.utils.prims as prims_utils
        from pxr import Usd, UsdGeom
        import numpy as np

        # 为相机添加iewport
        from isaacsim.core.utils.viewports import create_viewport_for_camera, set_camera_view
        self.camera_prim_path = "/World/camera_test"
        xform_path = self.camera_prim_path + "/xform_camera"
        # prims_utils.delete_prim(xform_path) # 如果已经有了，要先删除
        # prims_utils.delete_prim(camera_prim_path)
        prims_utils.create_prim(prim_path=self.camera_prim_path, prim_type="Camera", position=np.array([5, 0, 50.0]), )
        # 创建一个 Xform 原始来控制相机的位置和旋转

        xform_prim = prims_utils.create_prim(prim_path=xform_path, prim_type="Xform")
        create_viewport_for_camera(
            viewport_name="/test_camera",
            camera_prim_path=self.camera_prim_path,
            width=1280 / 2,
            height=720 / 2,
            position_x=0,  ## 设置他在IsaacSimAPP中的相对的位置
            position_y=0,
        )

        ## 可以设置相机的观察位置
        set_camera_view(
            eye=np.array([5, 0, 50]),
            target=np.array([5, 0, 0]),
            camera_prim_path=self.camera_prim_path,
        )
        #
        # # 网格世界的地图
        # self.grid_map = GridMap(
        #     start_point=[0, 0, 0],
        #     min_bounds=[-10, -10, 0],
        #     max_bounds=[10, 10, 5],
        #     cell_size=0.2,
        #     occupied_cell=1,
        #     empty_cell=0,
        #     invisible_cell=2,
        # )
        # self.world.scene.stage.add() # 无法使用的
        # 寻找资源路径, 一定要是本地的路径
        # self.assets_root_path = get_assets_root_path()
        # if self.assets_root_path is None:
        #     carb.log_error("Could not find nucleus server with /Isaac folder")

        # 用于存储所有机器人
        # self.robots = {}
        # from grutopia.core.runner import SimulatorRunner  # noqa E402.
        #
        # self._runner = SimulatorRunner(simulator_runtime=simulator_runtime)

        # ========================= import space ============================
        # import grutopia.core.util.space as space  # noqa E402.
        #
        # self._space = space
        # self.action_space = self._get_action_space()
        # self.observation_space = self._get_observation_space()
        # ===================================================================

        # log.info(f'==================== {self._robot_name} ======================')
        return

    def add_robot(self, robot_name):

        from isaacsim.robot.wheeled_robots.robots import WheeledRobot
        name_dict = {'jetbot': '/Jetbot/jetbot.usd'}
        jet_robot_asset_path = self.assets_root_path + "/Isaac/Robots" + name_dict[robot_name]

        self.robots[f'jetbot_{len(self.robots)}'] = WheeledRobot(
            prim_path="/World/Fancy_Robot",
            name=f"fancy_robot_{len(self.robots)}",
            wheel_dof_names=["left_wheel_joint", "right_wheel_joint"],
            create_robot=True,
            usd_path=jet_robot_asset_path,
        )

        self.world.scene.add(list(self.robots.values())[-1])

        # from isaacsim.core.api.physics
        print(self.world.get_physics_dt())
        if robot_name == 'jetbot':
            pass
        else:
            print("机器人名字错误")
        return

    def add_jetbot(self):

        assets_root_path = get_assets_root_path()
        if assets_root_path is None:
            carb.log_error("Could not find nucleus server with /Isaac folder")
        jet_robot_asset_path = assets_root_path + "/Isaac/Robots/Jetbot/jetbot.usd"

        self.jetbot_robot3 = WheeledRobot(
            prim_path="/World/Fancy_Robot3",
            name="fancy_robot3",
            wheel_dof_names=["left_wheel_joint", "right_wheel_joint"],
            create_robot=True,
            usd_path=jet_robot_asset_path,
            position=[-2, 0, 0],
            # orientation =
        )  # 参考：super().__init__(prim_path=prim_path, name_prefix=name_prefix, position=position, orientation=orientation, scale=scale)

        # self.jetbot_robot2 = WheeledRobot(
        #     prim_path="/World/Fancy_Robot2",
        #     name_prefix="fancy_robot2",
        #     wheel_dof_names=["left_wheel_joint", "right_wheel_joint"],
        #     create_robot=True,
        #     usd_path=jet_robot_asset_path,
        #     position=[-2, -1, 0],
        #     # orientation =
        # )  # 参考：super().__init__(prim_path=prim_path, name_prefix=name_prefix, position=position, orientation=orientation, scale=scale)

        self.world.scene.add(self.jetbot_robot3)

    def reset(self):
        self.world.reset()
        cell_size = 0.2
        self.grid_map = GridMap(min_bounds=[-10, -10, 0], max_bounds=[10, 10, 10], cell_size=cell_size)
        print("init grid map success")

        return

    def step(self, action):
        self.world.step()
        return
