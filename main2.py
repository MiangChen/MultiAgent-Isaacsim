import os

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})  # we can also run as headless.
from files.assets_scripts_linux import NAME_USR, PATH_PROJECT, PATH_ISAACSIM_ASSETS

import carb

carb.settings.get_settings().set(
    "/presitent/isaac/asset_root/default",
    f"{PATH_ISAACSIM_ASSETS}/Assets/Isaac/4.5",
)

from environment.env import Env
import numpy as np
import matplotlib

#  import omni.usd  # 从4.5开始就无法使用了
from isaacsim.core.utils.viewports import set_camera_view

matplotlib.use('TkAgg')

simulation_time = 0.0  # 记录模拟时间
duration = 5.0  # 目标时间 (5 秒)

num_env = 1

if __name__ == "__main__":
    # 加载复杂场景
    # usd_path = './scene/CityDemopack/World_CityDemopack.usd'
    # usd_path = f'{PATH_PROJECT}/scene/simple_city.usd'
    # usd_path = f'{PATH_PROJECT}/scene/flatroom4.usd'
    usd_path = f'{PATH_PROJECT}/scene/flatroom2.usd'
    usd_abs_path = os.path.abspath(usd_path)
    env = Env(simulation_app, usd_abs_path)
    env.reset()

    # 需要先构建地图, 才能做后续的规划
    env.map_grid.generate_grid_map('2d')

    env.world.add_physics_callback("physics_step_h1_0",
                                   callback_fn=env.robot_swarm.robot_active['h1'][0].on_physics_step)

    from llm.proj import run_once

    input("按下 Enter 开始一次录音（5秒）...")
    x, y, z = run_once(file_path="coffee.wav")
    # x, y, z = -1.2, 3.78, 0
    print(f"导航到{x, y,z}点")
    env.robot_swarm.robot_active['h1'][0].navigate_to([x, y, z], load_from_file=True)
    for i in range(500000):

        # 设置相机的位置
        camera_pose = np.zeros(3)  # 创建一个包含三个0.0的数组

        # camera_pose[:2] = pos_cf2x[:2]  # 将xy坐标赋值给result的前两个元素
        # camera_pose[2] = pos_cf2x[-1] + 1
        # set_camera_view(
        #     eye=camera_pose,  # np.array([5+i*0.001, 0, 50]),
        #     target=pos_cf2x,  # np.array([5+i*0.001, 0, 0]),
        #     camera_prim_path=env.camera_prim_path,
        # )

        # 先不混用
        state_skill_complete_all = True
        for robot_class in env.robot_swarm.robot_class:
            for robot in env.robot_swarm.robot_active[robot_class]:
                state_skill_complete_all = state_skill_complete_all and robot.state_skill_complete

        env.step(action=None)  # execute one physics step and one rendering step
        if i % 60 == 0:  # 1s加一个轨迹
            pass
            print(len(env.robot_swarm.robot_active['h1'][0].path), env.robot_swarm.robot_active['h1'][0].path_index)
            # print(env.robot_swarm.robot_active['cf2x'][0].robot_entity.get_joint_velocities())

    simulation_app.close()  # close Isaac Sim
