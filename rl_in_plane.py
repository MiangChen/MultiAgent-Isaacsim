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


if __name__ == "__main__":
    from robot import BaseRobot
    env.reset()

    assets_root_path = get_assets_root_path()
    if assets_root_path is None:
        carb.log_error("Could not find nucleus server with /Isaac folder")
    # jet_robot_asset_path = assets_root_path + "/Isaac/Robots/Jetbot/jetbot.usd"

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
    # jetbot_robot2 = WheeledRobot(
    #     prim_path="/World/Fancy_Robot2",
    #     name="fancy_robot2",
    #     wheel_dof_names=["left_wheel_joint", "right_wheel_joint"],
    #     create_robot=True,
    #     usd_path=jet_robot_asset_path,
    #     position=[-2, -1, 0],
    #     # orientation =
    # )  # 参考：super().__init__(prim_path=prim_path, name=name, position=position, orientation=orientation, scale=scale)

    # env.world.scene.add(jetbot_robot2)


    from isaacsim.core.api.objects import VisualSphere
    sphere = VisualSphere(
        prim_path="/World/red_point",
        position=np.array([3.0, 3.0, 0.0], dtype=np.float32),
        radius=0.1,
        color=np.array([1.0, 0.0, 0.0], dtype=np.float32)
    )
    env.world.scene.add(sphere)
    # env.add_jetbot()
    # env.add_robot('jetbot')
    env.robot.move_along_path([[1,1], [1,2], [2,2], [2,1],[1,1]], reset_flag=True)
    for i in range(500000):
        # env.jetbot_robot.apply_action(controller.forward(command=[0.20, np.pi/4]))
        # env.jetbot_robot3.apply_action(controller.forward(command=[0.20, np.pi/4]))

        # jetbot_robot2.apply_action(controller.forward(command=[0.20, np.pi/4]))

        # env.robots['jetbot_0'].apply_action(controller.forward(command=[0.20, np.pi/4]))
        from isaacsim.core.utils.viewports import create_viewport_for_camera, set_camera_view

        result = np.zeros(3)  # 创建一个包含三个0.0的数组
        xy_coords = env.robot.robot.get_world_pose()[0][:2]
        result[:2] = xy_coords  # 将xy坐标赋值给result的前两个元素
        result[2] = 10
        set_camera_view(
            eye= result, # np.array([5+i*0.001, 0, 50]),
            target= env.robot.robot.get_world_pose()[0], #np.array([5+i*0.001, 0, 0]),
            camera_prim_path=env.camera_prim_path,
        )
        # env.robot.apply_action([10.0, 9.0])
        # env.robot.move_to([3, 3])
        env.robot.move_along_path()
        env.step(action=None) # execute one physics step and one rendering step

    simulation_app.close() # close Isaac Sim

