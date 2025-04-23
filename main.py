import os

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})  # we can also run as headless.

from environment.env import Env
import numpy as np
import matplotlib

#  import omni.usd  # 从4.5开始就无法使用了
from isaacsim.core.utils.viewports import set_camera_view

matplotlib.use('TkAgg')

# fancy_cube =  world.scene.add(
#     DynamicCuboid(
#         prim_path="/World/random_cube",
#         name_prefix="fancy_cube",
#         position=np.array([1, 1, 1.0]),
#         scale=np.array([0.5015, 0.5015, 0.5015]),
#         color=np.array([0, 0, 1.0]),
#     )
# )


simulation_time = 0.0  # 记录模拟时间
duration = 5.0  # 目标时间 (5 秒)
initial_velocities = np.array([3, 4], dtype=np.float64)  # 初始速度
zero_velocities = np.array([0, 0], dtype=np.float64)  # 零速度
num_env = 1
from robot.robot_h1 import RobotH1, RobotCfgH1

# h1 = RobotH1(RobotCfgH1())
if __name__ == "__main__":
    # 加载复杂场景
    # usd_path = './scene/CityDemopack/World_CityDemopack.usd'
    usd_path = './scene/simple_city.usd'
    usd_abs_path = os.path.abspath(usd_path)
    env = Env(simulation_app, usd_abs_path)
    env.reset()

    # assets_root_path = get_assets_root_path()
    # if assets_root_path is None:
    #     carb.log_error("Could not find nucleus server with /Isaac folder")
    # jet_robot_asset_path = assets_root_path + "/Isaac/Robots/Jetbot/jetbot.usd"

    # from isaacsim.core.api.objects import VisualSphere
    #
    # sphere = VisualSphere(
    #     prim_path=f"{env.robot.robot.prim_path}/target_point",
    #     name_prefix='jetbot_target',
    #     position=np.array([3.0, 3.0, 0.0], dtype=np.float32),
    #     radius=0.1,
    #     color=np.array([1.0, 0.0, 0.0], dtype=np.float32)
    # )
    # env.world.scene.add(sphere)
    # env.add_jetbot()
    # env.add_robot('jetbot')
    # zone_corners = [[1, 1], [1, 10], [10, 10], [10, 1]]
    # path = env.robot.explore_zone(zone_corners)
    # env.robot.move_along_path(path, reset_flag=True)  # [[1,1], [1,2], [2,2], [2,1],[1,1]]g
    #
    # x_coords = [point[0] for point in path]
    # y_coords = [point[1] for point in path]
    #
    # plt.plot(x_coords, y_coords, marker='o', linestyle='-', color='blue')
    #
    # # 绘制区域边界
    # zone_x = [corner[0] for corner in zone_corners] + [zone_corners[0][0]]
    # zone_y = [corner[1] for corner in zone_corners] + [zone_corners[0][1]]
    # # plt.plot(zone_x, zone_y, color='red', linestyle='--')
    #
    # plt.xlabel("X")
    # plt.ylabel("Y")
    # plt.title("探索路径")
    # plt.grid(True)
    # plt.show()

    # 下面的内容,就是一个navigate to 的实现方法, 需要实例化一个全局的gridmap, 然后每一个jetbot都可以调用这个gridmap, 不要重复调用;
    env.map_grid.generate_grid_map('2d')

    # 添加回调函数, 每一个world step都会执行其中的内容
    env.robot_swarm.robot_active['jetbot'][0].navigate_to()
    env.world.add_physics_callback("physics_step_jetbot_0", callback_fn=env.robot_swarm.robot_active['jetbot'][0].on_physics_step)
    env.world.add_physics_callback("physics_step_jetbot_1", callback_fn=env.robot_swarm.robot_active['jetbot'][1].on_physics_step)
    env.world.add_physics_callback("physics_step_jetbot_2", callback_fn=env.robot_swarm.robot_active['jetbot'][2].on_physics_step)
    env.world.add_physics_callback("physics_step_h1_0", callback_fn=env.robot_swarm.robot_active['h1'][0].on_physics_step)
    env.robot_swarm.robot_active['h1'][0].base_command = [0.1, 0, 0.5]
    for i in range(500000):

        # 设置相机的位置
        camera_pose = np.zeros(3)  # 创建一个包含三个0.0的数组
        pos = env.robot_swarm.robot_active['jetbot'][0].get_world_pose()[0]  # x y z 坐标
        pos1 = env.robot_swarm.robot_active['jetbot'][1].get_world_pose()[0]  # x y z 坐标
        camera_pose[:2] = pos[:2]  # 将xy坐标赋值给result的前两个元素
        camera_pose[2] = 10
        set_camera_view(
            eye=camera_pose,  # np.array([5+i*0.001, 0, 50]),
            target=pos,  # np.array([5+i*0.001, 0, 0]),
            camera_prim_path=env.camera_prim_path,
        )

        # 使用pddl进行规划


        env.step(action=None)  # execute one physics step and one rendering step
        if i % 60 == 0:  # 1s加一个轨迹
            env.robot_swarm.robot_active['jetbot'][0].traj.add_trajectory(pos)
            env.robot_swarm.robot_active['jetbot'][1].traj.add_trajectory(pos1)


    simulation_app.close()  # close Isaac Sim
