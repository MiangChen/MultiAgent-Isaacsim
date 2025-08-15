import os
import yaml

import argparse
from isaacsim import SimulationApp
import omni

def initialize_simulation_app_from_yaml(config_path):
    """
    Initialize SimulationApp with settings from a YAML file.

    Args:
        config_path (str): Path to the YAML configuration file.
    """
    # 1. 读取YAML配置文件
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)

    # 2. 获取SimulationApp的构造函数配置
    # 'headless' 设置现在直接从 YAML 文件中读取
    sim_app_config = config.get("simulation_app_config", {})
    is_headless = sim_app_config.get("headless", False)  # 安全地获取headless值，默认为False

    print(f"Initializing Isaac Sim with headless={is_headless}")

    # 3. 拼接 experience 路径
    experience = config.get("experience", "")
    # 确保 EXP_PATH 环境变量已设置
    exp_path = os.environ.get("EXP_PATH")
    if not exp_path:
        raise ValueError("EXP_PATH environment variable is not set. Please set it to your Isaac Sim experience path.")

    full_experience_path = f'{exp_path}/{experience}'

    # --- 新增逻辑：从YAML读取并启用扩展 ---
    # extensions_to_enable = config.get("enabled_extensions", [])
    # if extensions_to_enable:
    #     ext_manager = omni.kit.app.get_app().get_extension_manager()
    #     for ext_name in extensions_to_enable:
    #         print(f"Enabling extension: {ext_name}")
    #         ext_manager.set_extension_enabled_immediate(ext_name, True)

    # 4. 初始化 SimulationApp
    simulation_app = SimulationApp(sim_app_config, experience=full_experience_path)


    # 5. 循环设置所有其他渲染和物理参数
    if "settings" in config and config["settings"]:
        for key, value in config["settings"].items():
            simulation_app.set_setting(key, value)
            print(f"Set setting: {key} = {value}")

    # 6. 根据从配置中读取的 headless 状态来决定是否禁用视口
    if is_headless:
        import omni.kit.viewport.utility
        try:
            viewport = omni.kit.viewport.utility.get_active_viewport()
            if viewport:
                viewport.updates_enabled = False
                print("Viewport updates disabled for headless mode.")
        except Exception as e:
            print(f"Could not disable viewport: {e}")

    return simulation_app

# simulation_app = SimulationApp({"headless": False})  # we can also run as headless.
parser = argparse.ArgumentParser(description="Initialize Isaac Sim from a YAML config.")
parser.add_argument("--config", type=str, default="./files/sim_cfg.yaml", help="Path to the configuration YAML file.")
parser.add_argument("--enable", type=str, action='append', help="Enable a feature. Can be used multiple times.")

args = parser.parse_args()
# --enable isaacsim.ros2.bridge --enable omni.kit.graph.editor.core --enable omni.graph.bundle.action --enable omni.kit.graph.delegate.modern --enable isaacsim.asset.gen.omap --enable omni.graph.window.action
# 从YAML文件初始化
simulation_app = initialize_simulation_app_from_yaml(args.config)


# 性能优化
# carb.settings.get_settings().set_int("/rtx/debugMaterialType", 0)

from environment.env import Env

import matplotlib

matplotlib.use('TkAgg')


if __name__ == "__main__":
    # 加载场景\世界引擎\grid map的参数
    with open(f'./files/env_cfg.yaml', 'r') as f:
        cfg = yaml.safe_load(f)
    usd_abs_path = os.path.abspath(cfg['scene']['usd_path'])
    env = Env(
        # 场景
        usd_path=usd_abs_path,
        # 世界引擎
        simulation_app=simulation_app,
        physics_dt=cfg['world']['physics_dt'],
        # grid map的参数
        cell_size=cfg['map']['cell_size'],
        start_point=cfg['map']['start_point'],
        min_bounds=cfg['map']['min_bounds'],
        max_bounds=cfg['map']['max_bounds'],
        occupied_cell=cfg['map']['occupied_cell'],
        empty_cell=cfg['map']['empty_cell'],
        invisible_cell=cfg['map']['invisible_cell'],
    )
    env.reset()

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

    # 注册cf2x无人机的物理步进回调
    # env.world.add_physics_callback("physics_step_cf2x_0",
    #                                callback_fn=env.robot_swarm.robot_active['cf2x'][0].on_physics_step)

    # env.robot_swarm.robot_active['cf2x'][0].forward()  # 注释掉单次forward调用

    # 进行任务规划
    # from pddl.solver_p import plan
    plan = {'step_0': {'robot2': {'navigate-to': {'start': 'depot2', 'goal': 'place1'}},
                       'robot3': {'navigate-to': {'start': 'depot3', 'goal': 'place2'}}},
            'step_1': {'robot2': {'pick-up': {'it': 'item1', 'loc': 'place1'}},
                       'robot3': {'pick-up': {'it': 'item2', 'loc': 'place2'}}},
            'step_2': {'robot2': {'navigate-to': {'start': 'place1', 'goal': 'depot2'}},
                       'robot3': {'navigate-to': {'start': 'place2', 'goal': 'depot3'}}},
            'step_3': {'robot2': {'put-down': {'it': 'item1', 'loc': 'depot2'}},
                       'robot3': {'put-down': {'it': 'item2', 'loc': 'depot3'}}}}

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

    # env.robot_swarm.robot_active['h1'][0].navigate_to([-10, 5, 0])
    for i in range(5000000):

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
        # if i % 60 == 0:  # 1s加一个轨迹
        # env.robot_swarm.robot_active['jetbot'][0].traj.add_trajectory(pos)
        # env.robot_swarm.robot_active['jetbot'][1].traj.add_trajectory(pos1)
        # env.robot_swarm.robot_active['jetbot'][2].traj.add_trajectory(pos1)
        # env.robot_swarm.robot_active['jetbot'][3].traj.add_trajectory(pos1)
        # print(len(env.robot_swarm.robot_active['h1'][0].path), env.robot_swarm.robot_active['h1'][0].path_index)
        # print(env.robot_swarm.robot_active['cf2x'][0].robot_entity.get_joint_velocities())

    simulation_app.close()  # close Isaac Sim
