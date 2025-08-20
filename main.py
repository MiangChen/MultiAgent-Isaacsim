#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os, threading, queue, argparse, yaml
from collections import defaultdict, deque
import numpy as np

# ===== Isaac =====
from isaacsim import SimulationApp  # 保留原有导入方式

def initialize_simulation_app_from_yaml(config_path: str) -> SimulationApp:
    """
    用 YAML 配置初始化 SimulationApp，并应用 settings、在 headless 下禁用视口。
    需要环境变量 EXP_PATH 指向 Isaac Sim 的 experience 根目录。
    """
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f) or {}

    # SimulationApp 构造参数
    sim_app_config = config.get("simulation_app_config", {})
    is_headless = sim_app_config.get("headless", False)
    print(f"Initializing Isaac Sim with headless={is_headless}")

    # 拼接 experience 路径
    experience = config.get("experience", "")
    exp_path = os.environ.get("EXP_PATH")
    if not exp_path:
        raise ValueError("EXP_PATH environment variable is not set. Please set it to your Isaac Sim experience path.")
    full_experience_path = os.path.join(exp_path, experience) if experience else exp_path

    # 创建 SimulationApp
    simulation_app = SimulationApp(sim_app_config, experience=full_experience_path)

    # 批量设置 settings
    if "settings" in config and config["settings"]:
        for key, value in config["settings"].items():
            simulation_app.set_setting(key, value)
            print(f"Set setting: {key} = {value}")

    # headless 下尝试禁用视口更新
    if is_headless:
        try:
            import omni.kit.viewport.utility
            viewport = omni.kit.viewport.utility.get_active_viewport()
            if viewport:
                viewport.updates_enabled = False
                print("Viewport updates disabled for headless mode.")
        except Exception as e:
            print(f"Could not disable viewport: {e}")

    return simulation_app

parser = argparse.ArgumentParser(description="Initialize Isaac Sim from a YAML config.")
parser.add_argument("--config", type=str, default="/home/ubuntu/multiagent-isaacsimROS/src/multiagent_isaacsim/multiagent_isaacsim/files/sim_cfg.yaml", help="Path to the configuration YAML file.")
parser.add_argument("--enable", type=str, action='append', help="Enable a feature. Can be used multiple times.")

args = parser.parse_args()
# --enable isaacsim.ros2.bridge --enable omni.kit.graph.editor.core --enable omni.graph.bundle.action --enable omni.kit.graph.delegate.modern --enable isaacsim.asset.gen.omap --enable omni.graph.window.action
# 从YAML文件初始化
simulation_app = initialize_simulation_app_from_yaml(args.config)


import carb
from files.assets_scripts_linux import PATH_ISAACSIM_ASSETS

from environment.env import Env

# ===== ROS 2 =====
import rclpy

rclpy.init(args=None)
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from ros.ros_swarm import BaseNode, SceneMonitorNode
from plan_msgs.msg import (  # TODO: 换成你的真实包名
    Parameter, SkillInfo, RobotSkill, Plan as PlanMsg, TimestepSkills
)

# ---------- 语义地图 ----------
from map.map_semantic_map import MapSemantic
_sem_map = MapSemantic()

# ---------- 每机器人串行队列 ----------
# key: (robot_class:str, robot_idx:int) -> deque[SkillInfo]
_skill_queues = defaultdict(deque)
_skill_lock = threading.Lock()

def _param_dict(params):
    d = {}
    if params:
        for p in params:
            d[p.key] = p.value
    return d


def _parse_robot_id(robot_id: str):
    import re
    s = robot_id or ""
    # 允许 name、name_12、name-12、name12 三种；name 里允许数字和下划线
    m = re.match(r"^([A-Za-z]\w*?)[_-]?(\d+)$", s)
    if not m:
        return "jetbot", 0
    name = m.group(1)
    idx = int(m.group(2))
    return name, idx

# —— 具体技能：在主线程执行（会在各自实现里把 state_skill_complete=False）——
def _skill_navigate_to(env, rc, rid, params):
    pos = _sem_map.map_semantic[params["goal"]]
    env.robot_swarm.robot_active[rc][rid].navigate_to(pos)

def _skill_pick_up(env, rc, rid, params):
    env.robot_swarm.robot_active[rc][rid].pick_up()

def _skill_put_down(env, rc, rid, params):
    env.robot_swarm.robot_active[rc][rid].put_down()

_SKILL_TABLE = {
    "navigate-to": _skill_navigate_to,
    "pick-up": _skill_pick_up,
    "put-down": _skill_put_down,
}

# ---------- ROS：Plan 接收（累计入队），SceneMonitor 只运行 ----------
def build_ros_nodes():
    qos = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=50,
    )

    plan_receiver = BaseNode('plan_receiver')

    def _plan_cb(msg: PlanMsg):
        """按 time step 顺序，把每个机器人在该步的技能依次入队；不覆盖、不丢任务。"""
        try:
            # 1) 按 timestep 排序（防止发送端乱序）
            steps = sorted(msg.steps, key=lambda s: s.timestep)

            with _skill_lock:
                for ts in steps:                # ts: TimestepSkills
                    for rs in ts.robots:        # rs: RobotSkill
                        rc, rid = _parse_robot_id(rs.robot_id)

                        # 确保队列存在
                        q = _skill_queues[(rc, rid)]

                        # 2) 依序入队每个技能；必要时可插入时间步分隔符
                        for sk in rs.skill_list:  # sk: SkillInfo
                            q.append(sk)

                        # 可选：如果执行侧需要“时间步边界”，插入一个哨兵
                        # q.append(EndOfTimestep(ts.timestep))

        except Exception as e:
            carb.log_error(f"[PlanCB] {e}")

    # 这里应传 QoSProfile 对象，而不是 depth 数字
    plan_receiver.create_subscription(PlanMsg, '/Plan', _plan_cb, qos)

    scene_monitor = SceneMonitorNode()
    return plan_receiver, scene_monitor

def spin_ros_in_background(nodes, stop_evt: threading.Event):
    exec_ = MultiThreadedExecutor(num_threads=4)
    for n in nodes: exec_.add_node(n)
    try:
        while not stop_evt.is_set():
            exec_.spin_once(timeout_sec=0.05)
    finally:
        for n in nodes:
            try: exec_.remove_node(n)
            except: pass
            try: n.destroy_node()
            except: pass
        rclpy.shutdown()


# ---------- 主程序 ----------
def main():

    # ====== 资产根设置（保持你原始逻辑，放在 app 创建之后） ======
    carb.settings.get_settings().set(
        "/presitent/isaac/asset_root/default",
        f"{PATH_ISAACSIM_ASSETS}/Assets/Isaac/4.5"
    )

    # ====== ROS 初始化移入 main，避免导入即初始化 ======
    # 创建 Env（保持你原始逻辑）
    with open('/home/ubuntu/multiagent-isaacsimROS/src/multiagent_isaacsim/multiagent_isaacsim/files/env_cfg.yaml', 'r') as f:
        cfg = yaml.safe_load(f)
    usd_abs_path = os.path.abspath(cfg['scene']['usd_path'])

    env = Env(
        usd_path=usd_abs_path,
        simulation_app=simulation_app,
        physics_dt=cfg['world']['physics_dt'],
        cell_size=cfg['map']['cell_size'],
        start_point=cfg['map']['start_point'],
        min_bounds=cfg['map']['min_bounds'],
        max_bounds=cfg['map']['max_bounds'],
        occupied_cell=cfg['map']['occupied_cell'],
        empty_cell=cfg['map']['empty_cell'],
        invisible_cell=cfg['map']['invisible_cell'],
    )
    env.reset()
    env.map_grid.generate_grid_map('2d')

    # 物理步回调（按你的项目）
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

    # ROS：Plan 接收 + SceneMonitor 只 run
    plan_receiver, scene_monitor = build_ros_nodes()
    stop_evt = threading.Event()
    t_ros = threading.Thread(target=spin_ros_in_background,
                             args=([plan_receiver, scene_monitor], stop_evt),
                             daemon=True)
    t_ros.start()

    # 预热渲染
    env.world.reset()
    for _ in range(10):
        simulation_app.update()

    try:
        while simulation_app.is_running():
            # ====== 全局屏障：必须所有机器人都完成，才会下发“下一批” ======
            state_skill_complete_all = True
            for robot_class in env.robot_swarm.robot_class:
                for robot in env.robot_swarm.robot_active[robot_class]:
                    # 没有此属性时按已完成处理，避免异常
                    done = getattr(robot, "state_skill_complete", True)
                    state_skill_complete_all = state_skill_complete_all and bool(done)

            if state_skill_complete_all:
                # 所有人都完成 → 同步为每个机器人各取一条、同时启动
                with _skill_lock:
                    keys = list(_skill_queues.keys())
                for rc, rid in keys:
                    with _skill_lock:
                        dq = _skill_queues.get((rc, rid))
                        next_skill = dq.popleft() if (dq and len(dq) > 0) else None
                        if dq is not None and len(dq) == 0:
                            _skill_queues.pop((rc, rid), None)
                    if next_skill is not None:
                        name = next_skill.skill.strip().lower()
                        params = _param_dict(next_skill.params)
                        fn = _SKILL_TABLE.get(name)
                        if fn is None:
                            carb.log_warn(f"[Scheduler] unsupported skill: {name}")
                        else:
                            try:
                                # 在主线程启动；各技能实现内部会把 state_skill_complete = False
                                fn(env, rc, rid, params)
                            except Exception as e:
                                carb.log_error(f"[Scheduler] start skill error: {e}")
            # 如果没全体完成，就什么也不发，等待完成

            # 推进仿真（物理+渲染）
            env.step(action=None)

    finally:
        stop_evt.set()
        t_ros.join(timeout=1.0)
        simulation_app.close()

if __name__ == "__main__":
    main()
