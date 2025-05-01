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

num_env = 1

if __name__ == "__main__":
    # 加载复杂场景
    # usd_path = './scene/CityDemopack/World_CityDemopack.usd'
    usd_path = f'{PATH_PROJECT}/scene/simple_city.usd'
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

    # 需要先构建地图, 才能做后续的规划
    env.map_grid.generate_grid_map('2d')

    # 添加回调函数, 每一个world step都会执行其中的内容
    env.world.add_physics_callback("physics_step_jetbot_0",
                                   callback_fn=env.robot_swarm.robot_active['jetbot'][0].on_physics_step)
    env.world.add_physics_callback("physics_step_jetbot_1",
                                   callback_fn=env.robot_swarm.robot_active['jetbot'][1].on_physics_step)
    env.world.add_physics_callback("physics_step_jetbot_2",
                                   callback_fn=env.robot_swarm.robot_active['jetbot'][2].on_physics_step)
    env.world.add_physics_callback("physics_step_jetbot_3",
                                   callback_fn=env.robot_swarm.robot_active['jetbot'][3].on_physics_step)
    env.world.add_physics_callback("physics_step_h1_0",
                                   callback_fn=env.robot_swarm.robot_active['h1'][0].on_physics_step)

    env.robot_swarm.robot_active['cf2x'][0].forward()


    # 进行任务规划
    from pddl.solver_p import plan
    from map.map_semantic_map import MapSemantic

    map_semantic = MapSemantic()
    state_step = 0  # state用来表示状态相关的量
    for robot in plan[f"step_{state_step}"].keys():
        id = int(robot[-1])
        for robot_action in plan[f"step_{state_step}"][robot].keys():
            if robot_action == 'navigate-to':
                map_semantic_start = plan[f"step_{state_step}"][robot][robot_action]['start']
                map_semantic_end = plan[f"step_{state_step}"][robot][robot_action]['goal']

                pos_target = map_semantic.map_semantic[map_semantic_end]
                env.robot_swarm.robot_active['jetbot'][id].navigate_to(pos_target)
            elif robot_action == 'pick-up':
                object_semantic_name = plan[f"step_{state_step}"][robot][robot_action]['it']
                object_semantics_pos = plan[f"step_{state_step}"][robot][robot_action]['loc']

    env.robot_swarm.robot_active['h1'][0].navigate_to([5, 5, 0])
    for i in range(500000):

        # 设置相机的位置
        camera_pose = np.zeros(3)  # 创建一个包含三个0.0的数组
        pos = env.robot_swarm.robot_active['jetbot'][0].get_world_poses()[0]  # x y z 坐标
        pos1 = env.robot_swarm.robot_active['jetbot'][1].get_world_poses()[0]  # x y z 坐标
        pos_cf2x = env.robot_swarm.robot_active['cf2x'][0].get_world_poses()[0]  # x y z 坐标
        camera_pose[:2] = pos_cf2x[:2]  # 将xy坐标赋值给result的前两个元素
        camera_pose[2] = pos_cf2x[-1] + 1
        set_camera_view(
            eye=camera_pose,  # np.array([5+i*0.001, 0, 50]),
            target=pos_cf2x,  # np.array([5+i*0.001, 0, 0]),
            camera_prim_path=env.camera_prim_path,
        )

        # 使用pddl进行规划
        # 根据已经规划好的, 进行一个划分,
        # 要首先确定机器人的action complete状态, 肯定要记录上次的action是什么, 然后action 有一个 flag, 用于记录这个action启动后, 有没有完成; flag_action_complete
        # 如果发现每一个机器人都完成了action, 那么就可以进入plan搜索下一个step,
        # 一个flag用于确定是否启动回调函数
        # 一个flag用于确定机器人时候时候完成了某个action
        # 先不混用
        state_skill_complete_all = True
        for robot_class in env.robot_swarm.robot_class:
            for robot in env.robot_swarm.robot_active[robot_class]:
                state_skill_complete_all = state_skill_complete_all and robot.state_skill_complete

        if state_skill_complete_all == True:
            state_step += 1
            if f"step_{state_step}" in plan.keys():
                for robot in plan[f"step_{state_step}"].keys():
                    id = int(robot[-1])
                    for robot_action in plan[f"step_{state_step}"][robot].keys():
                        if robot_action == 'navigate-to':
                            map_semantic_start = plan[f"step_{state_step}"][robot][robot_action]['start']
                            map_semantic_end = plan[f"step_{state_step}"][robot][robot_action]['goal']
                            pos_target = map_semantic.map_semantic[map_semantic_end]
                            env.robot_swarm.robot_active['jetbot'][id].navigate_to(pos_target)
                        elif robot_action == 'pick-up':
                            object_semantic_name = plan[f"step_{state_step}"][robot][robot_action]['it']
                            object_semantics_pos = plan[f"step_{state_step}"][robot][robot_action]['loc']
                            env.robot_swarm.robot_active['jetbot'][id].pick_up()
                        elif robot_action == 'put-down':
                            object_semantic_name = plan[f"step_{state_step}"][robot][robot_action]['it']
                            object_semantics_pos = plan[f"step_{state_step}"][robot][robot_action]['loc']

        env.step(action=None)  # execute one physics step and one rendering step
        if i % 60 == 0:  # 1s加一个轨迹
            env.robot_swarm.robot_active['jetbot'][0].traj.add_trajectory(pos)
            env.robot_swarm.robot_active['jetbot'][1].traj.add_trajectory(pos1)
            env.robot_swarm.robot_active['jetbot'][2].traj.add_trajectory(pos1)
            env.robot_swarm.robot_active['jetbot'][3].traj.add_trajectory(pos1)
            # print(len(env.robot_swarm.robot_active['h1'][0].path), env.robot_swarm.robot_active['h1'][0].path_index)
            # print(env.robot_swarm.robot_active['cf2x'][0].robot_entity.get_joint_velocities())

    simulation_app.close()  # close Isaac Sim
