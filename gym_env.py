from typing import Any

import gymnasium as gym

from scene import Simulator
class Env(gym.Env):
    """
    Gym Env for a single environment with a single learning agent.
    ----------------------------------------------------------------------
    """

    RESET_INFO_TASK_RUNTIME = 'task_runtime'

    def __init__(self, simulator: Simulator) -> None:
        self._render = None
        self._runtime = simulator
        self._robot_name = None
        self._current_task_name = None
        self._validate()
        # from isaacsim import SimulationApp
        # self.simulation_app = SimulationApp({"headless": False})  # we can also run as headless.
        print("init success")

        from isaacsim.core.api import World
        from isaacsim.core.api.objects import DynamicCuboid
        from isaacsim.core.utils.nucleus import get_assets_root_path

        self.world = World()
        self.world.scene.add_default_ground_plane()

        # 寻找资源路径, 一定要是本地的路径
        self.assets_root_path = get_assets_root_path()
        if self.assets_root_path is None:
            carb.log_error("Could not find nucleus server with /Isaac folder")

        # 用于存储所有机器人
        self.robots = []
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

        if robot_name == 'jetbot':
            self.robots.append(self.world.scene.add(
                WheeledRobot(
                    prim_path="/World/Fancy_Robot",
                    name="fancy_robot_" + len(self.robots),
                    wheel_dof_names=["left_wheel_joint", "right_wheel_joint"],
                    create_robot=True,
                    usd_path=jet_robot_asset_path,
                )
            )
        else:
            print("机器人名字错误")

