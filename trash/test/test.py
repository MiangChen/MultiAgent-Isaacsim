import os

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})  # we can also run as headless.

from environment.env import Env
import numpy as np

#  import omni.usd  # 从4.5开始就无法使用了
first_step = True
reset_needed = False
robots = []

# initialize robot on first step, run robot advance
def on_physics_step(step_size) -> None:
    global first_step
    global reset_needed
    if first_step:
        for robot in robots:
            robot.initialize()
        first_step = False
    # elif reset_needed:
    #     my_world.reset(True)
    #     reset_needed = False
    #     first_step = True
    else:
        for robot in robots:
            robot.step(step_size, base_command)

if __name__ == "__main__":
    # 加载复杂场景
    # usd_path = './scene/CityDemopack/World_CityDemopack.usd'
    usd_path = '../scene/simple_city.usd'
    usd_abs_path = os.path.abspath(usd_path)
    env = Env(simulation_app, usd_abs_path)
    env.reset()

    from controller.controller_policy_h1 import H1FlatTerrainPolicy

    prim_path = "/World/h1"
    h1 = H1FlatTerrainPolicy(prim_path=prim_path, name="h1", position=np.array([0, 0, 1.05]))
    h1.initialize()

    base_command = [0.5, 0, 0]
    for i in range(500000):
        h1.forward(dt=None, command= base_command)
        env.step(action=None)
        # if i % 10 == 0:
