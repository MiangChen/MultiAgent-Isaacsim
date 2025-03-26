import os
from typing import Any

import gymnasium as gym

from scene import Simulator




def create_scene(config_json_path: str, prim_path_root: str = 'background'):
    """
    TODO: rename.
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

        from isaacsim.core.api import World
        from isaacsim.core.api.objects import DynamicCuboid
        from isaacsim.core.utils.nucleus import get_assets_root_path

        self._runner = simulation_app
        self.world = World()
        # self.world.scene.add_default_ground_plane()

        # 加载复杂场景
        # usd_path = './scene/CityDemopack/World_CityDemopack.usd'
        usd_path = './scene/simple_city.usd'
        source, prim_path = create_scene(
            os.path.abspath(usd_path),
            prim_path_root=f'World/env_test_first/scene',
        )
        from isaacsim.core.utils.prims import create_prim

        create_prim(
            prim_path,
            usd_path=source,
            # scale=self.simulation.scene_scale,
            scale=[1, 1, 1],
            # translation=[self.runtime.env.offset[idx] + i for idx, i in enumerate(self.runtime.scene_position)],
        )

        # self.world.scene.stage.add() # 无法使用的
        # 寻找资源路径, 一定要是本地的路径
        self.assets_root_path = get_assets_root_path()
        if self.assets_root_path is None:
            carb.log_error("Could not find nucleus server with /Isaac folder")

        # 用于存储所有机器人
        self.robots = {}
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

        for robot in self.robots.values():
            self.world.scene.add(robot)

        if robot_name == 'jetbot':
            pass
        else:
            print("机器人名字错误")
        return

    def reset(self):
        self.world.reset()
        return

    def step(self, action):
        self.world.step()
        return

