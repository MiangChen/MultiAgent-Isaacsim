from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False}) # we can also run as headless.

from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.nucleus import get_assets_root_path
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.api.robots import Robot

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

add_reference_to_stage(usd_path=jet_robot_asset_path, prim_path="/World/Fancy_Robot")

jetbot_robot = world.scene.add(Robot(prim_path="/World/Fancy_Robot", name="fancy_robot"))
jetbot_articulation_controller = jetbot_robot.get_articulation_controller()

simulation_time = 0.0  # 记录模拟时间
duration = 5.0  # 目标时间 (5 秒)
initial_velocities = np.array([3, 4], dtype=np.float64) # 初始速度
zero_velocities = np.array([0, 0], dtype=np.float64) # 零速度

def send_robot_actions(step_size):
    global simulation_time # 声明使用全局变量
    global initial_velocities
    global zero_velocities

    # 获取物理引擎的 step time
    dt = world.get_physics_dt()
    simulation_time += dt

    if simulation_time < duration:
        # 在 5 秒内应用初始速度
        jetbot_articulation_controller.apply_action(ArticulationAction(joint_positions=None,
                                                                             joint_efforts=None,
                                                                             joint_velocities= initial_velocities))
        print("simulation time: ", simulation_time)
    else:
        # 5 秒后将速度设置为 0
        jetbot_articulation_controller.apply_action(ArticulationAction(joint_positions=None,
                                                                             joint_efforts=None,
                                                                             joint_velocities= zero_velocities))
    return


world.add_physics_callback("sending_actions", callback_fn=send_robot_actions)

world.reset()

for i in range(5000):
    position, orientation = fancy_cube.get_world_pose()
    linear_velocity = fancy_cube.get_linear_velocity()
    # will be shown on terminal
    # print("Cube position is : " + str(position))
    # print("Cube's orientation is : " + str(orientation))
    # print("Cube's linear velocity is : " + str(linear_velocity))
    # we have control over stepping physics and rendering in this workflow
    # things run in sync
    world.step(render=True) # execute one physics step and one rendering step

simulation_app.close() # close Isaac Sim