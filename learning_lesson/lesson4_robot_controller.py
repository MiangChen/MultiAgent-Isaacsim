from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False}) # we can also run as headless.

from isaacsim.core.api import World
from isaacsim.core.api.controllers import BaseController
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.nucleus import get_assets_root_path
from isaacsim.robot.wheeled_robots.robots import WheeledRobot

import asyncio
import numpy as np
import carb

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

world = World()
world.scene.add_default_ground_plane()
fancy_cube =  world.scene.add(
    DynamicCuboid(
        prim_path="/World/random_cube",
        name="fancy_cube",
        position=np.array([1, 1, 1.0]),
        scale=np.array([0.5015, 0.5015, 0.5015]),
        color=np.array([0, 0, 1.0]),
    ))

assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find nucleus server with /Isaac folder")
jet_robot_asset_path = assets_root_path + "/Isaac/Robots/Jetbot/jetbot.usd"

jetbot_robot = world.scene.add(
    WheeledRobot(
        prim_path="/World/Fancy_Robot",
        name="fancy_robot",
        wheel_dof_names=["left_wheel_joint", "right_wheel_joint"],
        create_robot=True,
        usd_path=jet_robot_asset_path,
    )
)

jetbot_robot2 = WheeledRobot(
            prim_path="/World/Fancy_Robot2",
            name="fancy_robot2",
            wheel_dof_names=["left_wheel_joint", "right_wheel_joint"],
            create_robot=True,
            usd_path=jet_robot_asset_path,
            position=[1, 1, 0],
        )
world.scene.add(jetbot_robot2)


simulation_time = 0.0  # 记录模拟时间
duration = 5.0  # 目标时间 (5 秒)
initial_velocities = np.array([3, 4], dtype=np.float64) # 初始速度
zero_velocities = np.array([0, 0], dtype=np.float64) # 零速度



if __name__ == "__main__":
    world.reset()

    controller = CoolController()
    for i in range(50000):
        jetbot_robot.apply_action(controller.forward(command=[0.20, np.pi/4]))
        jetbot_robot2.apply_action(controller.forward(command=[0.3, np.pi]))
        world.step(render=True) # execute one physics step and one rendering step

    simulation_app.close() # close Isaac Sim

