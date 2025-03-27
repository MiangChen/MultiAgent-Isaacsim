#  import omni.usd  # 从4.5开始就无法使用了
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False}) # we can also run as headless.

import asyncio
import numpy as np
import carb

from gym_env import Env
# from controller import CoolController

from isaacsim.core.utils.nucleus import get_assets_root_path

# fancy_cube =  world.scene.add(
#     DynamicCuboid(
#         prim_path="/World/random_cube",
#         name="fancy_cube",
#         position=np.array([1, 1, 1.0]),
#         scale=np.array([0.5015, 0.5015, 0.5015]),
#         color=np.array([0, 0, 1.0]),
#     )
# )



simulation_time = 0.0  # 记录模拟时间
duration = 5.0  # 目标时间 (5 秒)
initial_velocities = np.array([3, 4], dtype=np.float64) # 初始速度
zero_velocities = np.array([0, 0], dtype=np.float64) # 零速度

num_env = 1

env = Env(simulation_app)

# usd_path = './scene/CityDemopack/World_CityDemopack.usd'
# stage = omni.usd.get_context().open_stage(usd_path)

# from pxr import Usd, UsdGeom # 不可以使用pxr记载场景
# stage = Usd.Stage.Open(usd_path)

from isaacsim.core.api.controllers import BaseController

from isaacsim.core.utils.types import ArticulationAction

from isaacsim.robot.wheeled_robots.robots import WheeledRobot


class CoolController(BaseController):
    def __init__(self):
        super().__init__(name="my_cool_controller")
        # An open loop controller that uses a unicycle model
        self._wheel_radius = 0.03
        self._wheel_base = 0.1125
        return

    def forward(self, command):
        # command will have two elements, first element is the forward velocity
        # second element is the angular velocity (yaw only).
        joint_velocities = [0.0, 0.0]
        joint_velocities[0] = ((2 * command[0]) - (command[1] * self._wheel_base)) / (2 * self._wheel_radius)
        joint_velocities[1] = ((2 * command[0]) + (command[1] * self._wheel_base)) / (2 * self._wheel_radius)
        # A controller has to return an ArticulationAction
        return ArticulationAction(joint_velocities=joint_velocities)



if __name__ == "__main__":
    from robot import BaseRobot
    env.reset()

    assets_root_path = get_assets_root_path()
    if assets_root_path is None:
        carb.log_error("Could not find nucleus server with /Isaac folder")
    jet_robot_asset_path = assets_root_path + "/Isaac/Robots/Jetbot/jetbot.usd"

    # from isaacsim.robot.wheeled_robots.robots import WheeledRobot
    #
    # jetbot_robot = WheeledRobot(
    #     prim_path="/World/Fancy_Robot",
    #     name="fancy_robot",
    #     wheel_dof_names=["left_wheel_joint", "right_wheel_joint"],
    #     create_robot=True,
    #     usd_path=jet_robot_asset_path,
    #     position=[-2, 0, 0],
    #     # orientation =
    # )  #  参考：super().__init__(prim_path=prim_path, name=name, position=position, orientation=orientation, scale=scale)
    #
    jetbot_robot2 = WheeledRobot(
        prim_path="/World/Fancy_Robot2",
        name="fancy_robot2",
        wheel_dof_names=["left_wheel_joint", "right_wheel_joint"],
        create_robot=True,
        usd_path=jet_robot_asset_path,
        position=[-2, -1, 0],
        # orientation =
    )  # 参考：super().__init__(prim_path=prim_path, name=name, position=position, orientation=orientation, scale=scale)

    env.world.scene.add(jetbot_robot2)


    controller = CoolController()
    # env.add_jetbot()
    # env.add_robot('jetbot')
    for i in range(500000):
        env.jetbot_robot.apply_action(controller.forward(command=[0.20, np.pi/4]))
        env.jetbot_robot3.apply_action(controller.forward(command=[0.20, np.pi/4]))

        # jetbot_robot2.apply_action(controller.forward(command=[0.20, np.pi/4]))

        # env.robots['jetbot_0'].apply_action(controller.forward(command=[0.20, np.pi/4]))
        env.step(action=None) # execute one physics step and one rendering step

    simulation_app.close() # close Isaac Sim


