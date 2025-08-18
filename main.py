import argparse
import asyncio
import yaml

import matplotlib
matplotlib.use('TkAgg')

from physics_engine.isaacsim_simulation_app import initialize_simulation_app_from_yaml


async def setup_simulation(simulation_app):
    from environment.env import Env

    # 加载场景\世界引擎\grid map的参数
    with open(f'./files/env_cfg.yaml', 'r') as f:
        cfg = yaml.safe_load(f)
    from files.variables import WORLD_USD_PATH

    env = await Env.create(
        # 场景
        usd_path=WORLD_USD_PATH,
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

    # 先构建地图, 才能做后续的规划
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
    return env


# --- 2. 主程序入口，负责实验逻辑和模拟循环 ---
if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Initialize Isaac Sim from a YAML config.")
    parser.add_argument("--config", type=str, default="./files/sim_cfg.yaml",
                        help="Path to the configuration physics engine.")
    parser.add_argument("--enable", type=str, action='append', help="Enable a feature. Can be used multiple times.")

    args = parser.parse_args()

    # 使用 `with` 语句来创建和管理 simulation_app 的整个生命周期
    simulation_app = initialize_simulation_app_from_yaml(args.config)

    try:
        # --- 设置阶段 ---
        # 获取事件循环并执行我们的一次性异步设置函数
        loop = asyncio.get_event_loop()
        env = loop.run_until_complete(setup_simulation(simulation_app))

        # --- 实验逻辑定义阶段 (从 setup 移到这里) ---
        print("--- Initializing experiment plan and semantic map ---")
        from map.map_semantic_map import MapSemantic

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

        map_semantic = MapSemantic()
        state_step = 0

        # 启动 plan 的第一步
        for robot in plan[f"step_{state_step}"].keys():
            id = int(robot[-1])
            for robot_action in plan[f"step_{state_step}"][robot].keys():
                if robot_action == 'navigate-to':
                    map_semantic_end = plan[f"step_{state_step}"][robot][robot_action]['goal']
                    pos_target = map_semantic.map_semantic[map_semantic_end]
                    env.robot_swarm.robot_active['jetbot'][id].navigate_to(pos_target)

        env.robot_swarm.robot_active['h1'][0].navigate_to([-10, 5, 0])
        print("--- Initial robot goals set. Starting simulation loop. ---")

        # --- 模拟循环阶段 ---
        # while simulation_app.is_running():
        while True:
            # 1. 驱动模拟器前进
            env.step(action=None)

            # 2. 检查并执行 PDDL 状态机逻辑
            state_skill_complete_all = True
            for robot_class in env.robot_swarm.robot_class:
                for robot in env.robot_swarm.robot_active[robot_class]:
                    state_skill_complete_all = state_skill_complete_all and robot.state_skill_complete

            if state_skill_complete_all:
                state_step += 1
                if f"step_{state_step}" in plan:
                    print(f"--- All robots completed actions. Advancing to PDDL step {state_step} ---")
                    for robot in plan[f"step_{state_step}"].keys():
                        id = int(robot[-1])
                        for robot_action in plan[f"step_{state_step}"][robot].keys():
                            if robot_action == 'navigate-to':
                                map_semantic_end = plan[f"step_{state_step}"][robot][robot_action]['goal']
                                pos_target = map_semantic.map_semantic[map_semantic_end]
                                env.robot_swarm.robot_active['jetbot'][id].navigate_to(pos_target)
                            elif robot_action == 'pick-up':
                                env.robot_swarm.robot_active['jetbot'][id].pick_up()
                            # ... (其他 PDDL 动作逻辑)

    finally:
        # 4. 手动调用 __exit__，这与离开 with 块的作用相同
        #    它会安全地关闭和清理应用程序
        print("--- Simulation finished. Manually closing application. ---")
        if simulation_app:
            simulation_app.__exit__(None, None, None)