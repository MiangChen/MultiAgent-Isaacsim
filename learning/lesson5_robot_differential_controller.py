from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False}) # we can also run as headless.

from isaacsim.core.api import World
from isaacsim.core.api.controllers import BaseController
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.nucleus import get_assets_root_path
from isaacsim.robot.wheeled_robots.robots import WheeledRobot
# This extension includes several generic controllers that could be used with multiple robots
from isaacsim.robot.wheeled_robots.controllers.wheel_base_pose_controller import WheelBasePoseController
# Robot specific controller
from isaacsim.robot.wheeled_robots.controllers.differential_controller import DifferentialController

import asyncio
import numpy as np
import carb

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

jetbot_robot = WheeledRobot(
        prim_path="/World/Fancy_Robot",
        name="fancy_robot",
        wheel_dof_names=["left_wheel_joint", "right_wheel_joint"],
        create_robot=True,
        usd_path=jet_robot_asset_path,
    )




if __name__ == "__main__":
    world.scene.add(jetbot_robot)
    world.reset()
    controller = WheelBasePoseController(name="cool_controller",
                                         open_loop_wheel_controller=
                                         DifferentialController(name="simple_control",
                                                                wheel_radius=0.03, wheel_base=0.1125),
                                         is_holonomic=False)

    for i in range(5000):
        position, orientation = jetbot_robot.get_world_pose()
        jetbot_robot.apply_action(controller.forward(start_position=position,
                                                     start_orientation=orientation,
                                                     goal_position=np.array([0.8, 0.8])))
        world.step(render=True) # execute one physics step and one rendering step

    simulation_app.close() # close Isaac Sim
