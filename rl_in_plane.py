from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False}) # we can also run as headless.

import asyncio
import numpy as np
import carb

from gym_env import Env



fancy_cube =  world.scene.add(
    DynamicCuboid(
        prim_path="/World/random_cube",
        name="fancy_cube",
        position=np.array([1, 1, 1.0]),
        scale=np.array([0.5015, 0.5015, 0.5015]),
        color=np.array([0, 0, 1.0]),
    )
)



simulation_time = 0.0  # 记录模拟时间
duration = 5.0  # 目标时间 (5 秒)
initial_velocities = np.array([3, 4], dtype=np.float64) # 初始速度
zero_velocities = np.array([0, 0], dtype=np.float64) # 零速度

num_env = 1

env = Env()

if __name__ == "__main__":
    world.reset()

    controller = CoolController()
    for i in range(5000):
        jetbot_robot.apply_action(controller.forward(command=[0.20, np.pi/4]))
        world.step(render=True) # execute one physics step and one rendering step

    simulation_app.close() # close Isaac Sim


