# Copyright (c) 2021-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import argparse

import carb
import numpy as np

from controller.controller_policy_h1 import H1FlatTerrainPolicy


import omni.appwindow  # Contains handle to keyboard
from isaacsim.core.api import World
from isaacsim.core.utils.prims import define_prim, get_prim_at_path
# from isaacsim.robot.policy.examples.robots import H1FlatTerrainPolicy
from isaacsim.storage.native import get_assets_root_path

parser = argparse.ArgumentParser(description="Define the number of robots.")
parser.add_argument("--num-robots", type=int, default=1, help="Number of robots (default: 1)")
parser.add_argument(
    "--env-url",
    default="/Isaac/Environments/Grid/default_environment.usd",
    required=False,
    help="Path to the environment url",
)
args = parser.parse_args()
print(f"Number of robots: {args.num_robots}")

first_step = True
reset_needed = False
robots = []

# initialize robot on first step, run robot advance


# spawn world
my_world = World(stage_units_in_meters=1.0, physics_dt=1 / 200, rendering_dt=1 /60)
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")

# spawn warehouse scene
prim = define_prim("/World/Ground", "Xform")
asset_path = assets_root_path + args.env_url
prim.GetReferences().AddReference(asset_path)

# spawn robot
for i in range(0, 1):
    h1 = H1FlatTerrainPolicy(
        prim_path="/World/H1_" + str(i),
        name="H1_" + str(i),
        usd_path=assets_root_path + "/Isaac/Robots/Unitree/H1/h1.usd",
        position=np.array([0, i, 1.05]),
    )

    robots.append(h1)



my_world.reset()
my_world.add_physics_callback("physics_step", callback_fn=robots[0].on_physics_step)  # 回调函数

for robot in robots:
    robot.initialize()

# robot command
# base_command = np.zeros(3)

i = 0
while simulation_app.is_running():
    my_world.step(render=True)
    # if my_world.is_stopped():
    #     reset_needed = True
    # if my_world.is_playing():
    #     if i >= 0 and i < 80:
    #         # forward
    #         base_command = np.array([1, 0, 0])
    #     elif i >= 80 and i < 130:
    #         # rotate
    #         base_command = np.array([0.5, 0, 0.5])
    #     elif i >= 130 and i < 200:
    #         # side ways
    #         base_command = np.array([0, 0, 0.5])
    #     elif i == 200:
    #         i = 0
    #     i += 1

simulation_app.close()

