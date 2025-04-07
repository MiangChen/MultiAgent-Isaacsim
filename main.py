#  import omni.usd  # 从4.5开始就无法使用了
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})  # we can also run as headless.

import asyncio
import numpy as np
import carb

from gym_env import Env
# from controller import CoolController

from isaacsim.core.utils.nucleus import get_assets_root_path
import matplotlib

matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

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
initial_velocities = np.array([3, 4], dtype=np.float64)  # 初始速度
zero_velocities = np.array([0, 0], dtype=np.float64)  # 零速度

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

    # from isaacsim.core.api.objects import VisualSphere
    #
    # sphere = VisualSphere(
    #     prim_path=f"{env.robot.robot.prim_path}/target_point",
    #     name='jetbot_target',
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

    from astar import AStar
    # import time
    # # time.sleep(10)
    # print(type(env.robot.robot))
    # print(dir(env.robot.robot))
    # print("*******************************")
    # print(type(env.robot.robot))
    # print(dir(env.robot.robot))
    # robot_pos = env.robot.robot.get_world_pose()[0]  # x y z 坐标
    def get_robot_pos(robot_prim_path = "/World/robot/jetbot"):
        from pxr import UsdGeom

        import isaacsim.core.utils.stage as stage_utils
        stage = stage_utils.get_current_stage()
        prim_robot = stage.GetPrimAtPath(robot_prim_path)
        # 获取 prim
        # prim_robot = stage.GetPrimAtPath(robot_prim_path)

        # 检查是否是 Xformable（可变换对象）
        if prim_robot.IsA(UsdGeom.Xformable):
            xform = UsdGeom.Xformable(prim_robot)
            local_transform = xform.GetLocalTransformation()  # 返回 Gf.Matrix4d
            local_position = local_transform.ExtractTranslation()  # 提取平移部分
            return local_position
            # print("Local Position:", local_position)
            # index =grid_map.compute_index(local_position)
            # print(index)
            # print(grid_map.pos_map.shape)
            # pos_grid_map = grid_map.pos_map[47, 50, 0, :]
            # print("pos_grid_map:", pos_grid_map)
            # print(pos_grid_map[index])
    env.grid_map.generate_grid_map('2d')
    robot_pos = get_robot_pos()

    index_robot = env.grid_map.compute_index(robot_pos)

    env.grid_map.generate_grid_map('2d')

    print(index_robot)
    target_pos = [9, 9, 0]

    index_target = env.grid_map.compute_index(target_pos)
    print("target index", index_target)

    # 初始化astar规划器
    directions = [
        [1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0],
        [1, 1, 0], [1, -1, 0], [-1, 1, 0], [-1, -1, 0],
    ]
    grid_map = env.grid_map.value_map
    grid_map[index_robot] = env.grid_map.empty_cell
    planner = AStar(env.grid_map.value_map, obs_value=1.0, free_value=0.0, directions=directions)
    path = planner.find_path(tuple(index_robot), tuple(index_target))
    real_path = np.zeros_like(path, dtype=np.float32)
    for i in range(path.shape[0]): # 把index变成连续实际世界的坐标
        real_path[i] = env.grid_map.pos_map[tuple(path[i])]
        real_path[i][-1] = 0
    print("start", robot_pos, "end", target_pos)
    print(real_path)
    env.robot.move_along_path(real_path, reset_flag=True)  # [[1,1], [1,2], [2,2], [2,1],[1,1]]g

    for i in range(500000):
        # env.jetbot_robot.apply_action(controller.forward(command=[0.20, np.pi/4]))
        # env.jetbot_robot3.apply_action(controller.forward(command=[0.20, np.pi/4]))

        # jetbot_robot2.apply_action(controller.forward(command=[0.20, np.pi/4]))

        # env.robots['jetbot_0'].apply_action(controller.forward(command=[0.20, np.pi/4]))
        from isaacsim.core.utils.viewports import create_viewport_for_camera, set_camera_view

        result = np.zeros(3)  # 创建一个包含三个0.0的数组
        xy_coords = env.robot.robot.get_world_pose()[0][:2]
        # xy_coords = get_robot_pos()[:2]
        result[:2] = xy_coords  # 将xy坐标赋值给result的前两个元素
        result[2] = 10
        set_camera_view(
            eye=result,  # np.array([5+i*0.001, 0, 50]),
            target=env.robot.robot.get_world_pose()[0],  # np.array([5+i*0.001, 0, 0]),
            # target=get_robot_pos(),  # np.array([5+i*0.001, 0, 0]),
            camera_prim_path=env.camera_prim_path,
        )
        # env.robot.apply_action([10.0, 9.0])
        # env.robot.move_to([3, 3])
        env.robot.move_along_path()
        env.step(action=None)  # execute one physics step and one rendering step
        if i % 60 == 0:  # 1s加一个轨迹
            env.robot.traj.add_trajectory(env.robot.robot.get_world_pose()[0])
            # env.robot.traj.add_trajectory(get_robot_pos())

    simulation_app.close()  # close Isaac Sim
