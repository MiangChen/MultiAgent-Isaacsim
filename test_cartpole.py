import os

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})  # we can also run as headless.

from environment.env import Env
import numpy as np

#  import omni.usd  # 从4.5开始就无法使用了

if __name__ == "__main__":
    # 加载复杂场景
    # usd_path = './scene/CityDemopack/World_CityDemopack.usd'
    usd_path = './scene/simple_city.usd'
    usd_abs_path = os.path.abspath(usd_path)
    env = Env(simulation_app, usd_abs_path)
    env.reset()

    from controller.controller_policy_cartpole2 import CartpolePolicy

    prim_path = "/World/cartpole"
    carpole = CartpolePolicy(prim_path=prim_path, name="cartpole", position=np.array([0, 0, 1.05]))
    # h1.initialize()
    # base_command = [0.01, 0, 0]
    for i in range(500000):
        env.step(action=None)
        if i % 10 == 0:
            # h1.forward(dt=None, command= base_command)
            pass