import argparse
import asyncio
from itertools import count

import yaml

import matplotlib
matplotlib.use('TkAgg')

from physics_engine.isaacsim_simulation_app import initialize_simulation_app_from_yaml


if __name__ == "__main__":

    # -- init simulation app --
    parser = argparse.ArgumentParser(description="Initialize Isaac Sim from a YAML config.")
    parser.add_argument("--config", type=str, default="./files/sim_cfg.yaml",
                        help="Path to the configuration physics engine.")
    parser.add_argument("--enable", type=str, action='append', help="Enable a feature. Can be used multiple times.")
    args = parser.parse_args()

    simulation_app = initialize_simulation_app_from_yaml(args.config)

    # -- init simulation env --
    from environment.env import Env
    from map.map_grid_map import GridMap
    from map.map_semantic_map import MapSemantic
    from scene.scene_manager import SceneManager

    from files.variables import WORLD_USD_PATH
    from files.assets_scripts_linux import PATH_PROJECT

    from robot.robot_cf2x import RobotCf2x, RobotCfgCf2x
    from robot.robot_jetbot import RobotCfgJetbot, RobotJetbot
    from robot.robot_h1 import RobotH1, RobotCfgH1
    from robot.swarm_manager import SwarmManager


    async def setup_simulation(simulation_app) -> Env:

        # --- 配置机器人管理器 (这些都是同步的配置) ---
        swarm_manager.register_robot_class(
            robot_class_name="jetbot",
            robot_class=RobotJetbot,
            robot_class_cfg=RobotCfgJetbot,
        )  # 注册jetbot机器人
        swarm_manager.register_robot_class(
            robot_class_name="h1", robot_class=RobotH1, robot_class_cfg=RobotCfgH1
        )  # 注册h1机器人
        swarm_manager.register_robot_class(
            robot_class_name="cf2x", robot_class=RobotCf2x, robot_class_cfg=RobotCfgCf2x
        )  # 注册cf2x机器人

        # Create environment first
        env = await Env.create(
            simulation_app=simulation_app,
            physics_dt=cfg['world']['physics_dt'],
            swarm_manager=swarm_manager,
            scene_manager=scene_manager,
            grid_map=map_grid,
        )

        # Initialize swarm manager after environment is ready
        try:
            await swarm_manager.initialize_async(
                scene=env.world.scene,
                robot_swarm_cfg_path=f"{PATH_PROJECT}/files/robot_swarm_cfg.yaml",
                robot_active_flag_path=f"{PATH_PROJECT}/files/robot_swarm_active_flag.yaml"
            )
        except Exception as e:
            print(f"Error during swarm manager initialization: {e}")
            raise

        return env

    with open(f'./files/env_cfg.yaml', 'r') as f:
        cfg = yaml.safe_load(f)

    # create gridmap manager
    map_grid = GridMap(
        cell_size=cfg['map']['cell_size'],
        start_point=cfg['map']['start_point'],
        min_bounds=cfg['map']['min_bounds'],
        max_bounds=cfg['map']['max_bounds'],
        occupied_cell=cfg['map']['occupied_cell'],
        empty_cell=cfg['map']['empty_cell'],
        invisible_cell=cfg['map']['invisible_cell'],
    )

    # create semantic
    map_semantic = MapSemantic()
    # create swarm manager
    swarm_manager = SwarmManager(map_grid)
    # create scene manager
    scene_manager = SceneManager()
    # load scene
    scene_manager.load_scene(usd_path=WORLD_USD_PATH)
    # create some cars
    scale = [2, 5, 1.0]
    CUBES_CONFIG = {
        "car0": {
            "shape_type": "cuboid",
            "size": scale,  # 尺寸未指定，使用默认值 1.0
            "position": [11.6, 3.5, 0],
            "color": [255, 255, 255],
            "make_dynamic": False,
        },

        "car1": {
            "shape_type": "cuboid",
            "size": scale,
            "position": [0.3, 3.5, 0],
            "color": [255, 255, 255],
            "make_dynamic": False,
        },
        "car2": {
            "shape_type": "cuboid",
            "size": scale,
            "position": [-13.2, 3.5, 0],
            "color": [255, 255, 255],
            "make_dynamic": False,
        },
        "car3": {
            "shape_type": "cuboid",
            "size": scale,
            "position": [-7.1, 10, 0],
            "color": [255, 255, 255],
            "make_dynamic": False,
        },
        "car4": {
            "shape_type": "cuboid",
            "size": scale,
            "position": [-0.9, 30, 0],
            "orientation": [0.707, 0, 0, 0.707],  # 使用上面计算出的旋转值
            "color": [255, 255, 255],
            "make_dynamic": False,
        },
    }
    created_prim_paths = []
    print( "all semantics in scene:", scene_manager.count_semantics_in_scene().get('result') )
    for cube_name, config in CUBES_CONFIG.items():
        print(f"--- Processing: {cube_name} ---")

        # --- step A: use create_shape  ---
        creation_result = scene_manager.create_shape(**config)

        # --- step B: check the result ---
        if creation_result.get("status") == "success":
            # 从返回值中提取 prim_path
            prim_path = creation_result.get("result")
            print(f"  Successfully created prim at: {prim_path}")
            created_prim_paths.append(prim_path)

            # --- step C: use add_semantic ---
            semantic_result = scene_manager.add_semantic(
                prim_path=prim_path,
                semantic_label='car'  # target semantic label 'car'
            )

            if semantic_result.get("status") == "success":
                print(f"  Successfully applied semantic label 'car' to {prim_path}")
            else:
                print(f"  [ERROR] Failed to apply semantic label: {semantic_result.get('message')}")

        else:
            print(f"  [ERROR] Failed to create shape '{cube_name}': {creation_result.get('message')}")

    # create camera
    # result = scene_manager.create_camera(position=[0, 0, 5], quat=scene_manager.euler_to_quaternion(85,0,0))
    # scene_manager.change_viewport(prim_path=result.get('result'))
    result = scene_manager.change_viewport(prim_path='/World/robot/jetbot/jetbot/jetbot_0/chassis/rgb_camera/jetbot_camera')
    print("create camera ",result)
    print("All prims with 'car' label:", created_prim_paths)

    print(scene_manager.count_semantics_in_scene().get('result'))

    try:
        # 获取事件循环并执行我们的一次性异步设置函数
        loop = asyncio.get_event_loop()
        env = loop.run_until_complete(setup_simulation(simulation_app))

        # scene_manager.change_viewport(prim_path='/World/robot/jetbot/jetbot/jetbot_0/chassis/rgb_camera/jetbot_camera')
        result = scene_manager.add_semantic_camera(prim_path='/World/semantic_camera', position = [ 0, 4, 2 ], quat = scene_manager.euler_to_quaternion(roll=90))
        semantic_camera = result.get('result').get('camera_instance')
        semantic_camera_prim_path = result.get('result').get('prim_path')
        scene_manager.change_viewport(prim_path=semantic_camera_prim_path)

        env.reset()

        semantic_camera.initialize()
        semantic_camera.add_bounding_box_2d_loose_to_frame()        # add semantic detection

        # 先构建地图, 才能做后续的规划
        map_grid.generate_grid_map('2d')

        # 添加回调函数, 每一个world step都会执行其中的内容
        env.world.add_physics_callback("physics_step_jetbot_0",
                                       callback_fn=swarm_manager.robot_active['jetbot'][0].on_physics_step)
        env.world.add_physics_callback("physics_step_jetbot_1",
                                       callback_fn=swarm_manager.robot_active['jetbot'][1].on_physics_step)
        env.world.add_physics_callback("physics_step_jetbot_2",
                                       callback_fn=swarm_manager.robot_active['jetbot'][2].on_physics_step)
        env.world.add_physics_callback("physics_step_jetbot_3",
                                       callback_fn=swarm_manager.robot_active['jetbot'][3].on_physics_step)
        env.world.add_physics_callback("physics_step_h1_0",
                                       callback_fn=swarm_manager.robot_active['h1'][0].on_physics_step)

        # 注册cf2x无人机的物理步进回调
        # env.world.add_physics_callback("physics_step_cf2x_0",
        #                                callback_fn=swarm_manager.robot_active['cf2x'][0].on_physics_step)

        # swarm_manager.robot_active['cf2x'][0].forward()  # 注释掉单次forward调用

        print("--- Initializing experiment plan and semantic map ---")

        # 定义任务规划
        plan = {
            'step_0': {'robot2': {'navigate-to': {'start': 'depot2', 'goal': 'place1'}},
                       'robot3': {'navigate-to': {'start': 'depot3', 'goal': 'place2'}}},
            'step_1': {'robot2': {'pick-up': {'it': 'item1', 'loc': 'place1'}},
                       'robot3': {'pick-up': {'it': 'item2', 'loc': 'place2'}}},
            'step_2': {'robot2': {'navigate-to': {'start': 'place1', 'goal': 'depot2'}},
                       'robot3': {'navigate-to': {'start': 'place2', 'goal': 'depot3'}}},
            'step_3': {'robot2': {'put-down': {'it': 'item1', 'loc': 'depot2'}},
                       'robot3': {'put-down': {'it': 'item2', 'loc': 'depot3'}}}
        }

        plan_step = 0

        swarm_manager.robot_active['h1'][0].navigate_to([-10, 5, 0])
        count = 0
        # --- simulation loop ---
        while simulation_app.is_running():
            # 1. world step
            env.step(action=None)

            if count % 120 == 0:
                result = semantic_camera.get_current_frame()['bounding_box_2d_loose']
                print("get bounding box 2d loose",result)
                if result:
                    car_prim, car_pose = map_semantic.get_prim_and_pose_by_semantic(result, 'car')
                    print("get car prim and pose\n",car_prim, '\n', car_pose)
            count += 1

            # 2. check if all robots have completed their actions
            state_skill_complete_all = True
            for robot_class in swarm_manager.robot_class:
                for robot in swarm_manager.robot_active[robot_class]:
                    # todo: only check the robots in PDDL plan
                    state_skill_complete_all = state_skill_complete_all and robot.state_skill_complete

            if state_skill_complete_all:
                if f"step_{plan_step}" in plan:
                    print(f"--- All robots completed actions. Advancing to PDDL step {plan_step} ---")
                    # 3. advance to next step
                    for robot in plan.get(f"step_{plan_step}").keys():
                        id = int(robot[-1])
                        for robot_action in plan.get(f"step_{plan_step}")[robot].keys():
                            if robot_action == 'navigate-to':
                                map_semantic_end = plan.get(f"step_{plan_step}")[robot][robot_action]['goal']
                                pos_target = map_semantic.map_semantic[map_semantic_end]
                                swarm_manager.robot_active['jetbot'][id].navigate_to(pos_target)
                            elif robot_action == 'pick-up':
                                swarm_manager.robot_active['jetbot'][id].pick_up()
                    plan_step += 1
    finally:
        # 4. use __exit__ to close simulation safely
        print("--- Simulation finished. Manually closing application. ---")
        if simulation_app:
            simulation_app.__exit__(None, None, None)
