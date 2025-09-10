from collections import defaultdict, deque
import threading
from typing import Dict, Any

from gsi_msgs.gsi_msgs_helper import Plan

# Global variables for ROS integration
_skill_queues = defaultdict(deque)
_skill_lock = threading.Lock()


def _parse_robot_id(robot_id: str) -> tuple[str, int]:
    """Parse robot ID string to extract robot class and index

    Args:
        robot_id: Robot identifier string (e.g., "jetbot_1", "h1-2")

    Returns:
        tuple[str, int]: Tuple of (robot_class_name, robot_index)
    """
    import re

    # 使用 := 将 re.match 的结果赋值给 m 并同时进行判断
    if m := re.match(r"^([A-Za-z]\w*?)[_-]?(\d+)$", robot_id or ""):
        return m.group(1), int(m.group(2))
    else:
        return "jetbot", 0


def _plan_cb(msg: Plan):
    """Plan callback to queue skills for robots"""
    try:
        steps = sorted(msg.steps, key=lambda s: s.timestep)
        with _skill_lock:
            for ts in steps:
                for rs in ts.robots:
                    rc, rid = _parse_robot_id(rs.robot_id)
                    q = _skill_queues[(rc, rid)]
                    q.extend(rs.skill_list)
    except Exception as e:
        logger.error(f"[PlanCB] Error: {e}")


def process_ros_skills(swarm_manager) -> None:
    """Process ROS skill queue and execute skills with injected SwarmManager

    Args:
        swarm_manager: Injected swarm manager instance
    """
    # 1. Check if all robots have completed their current skills

    state_skill_complete_all = all(
        getattr(robot, "state_skill_complete", True)
        for robot_class in swarm_manager.robot_class
        for robot in swarm_manager.robot_active[robot_class]
    )
    if not state_skill_complete_all:
        return

    # 2. 采用 "先收集任务，再执行" 的模式优化锁的使用
    skills_to_execute = []
    with _skill_lock:
        # 使用 list() 创建一个副本进行迭代，以安全地在循环中修改字典
        for (rc, rid), dq in list(_skill_queues.items()):
            if dq:  # 检查队列是否非空
                next_skill = dq.popleft()
                skills_to_execute.append((rc, rid, next_skill))
                if not dq:  # 如果弹出后队列为空，则从字典中移除
                    del _skill_queues[(rc, rid)]

    # 3. 在锁之外执行可能耗时的技能调用
    for rc, rid, skill_msg in skills_to_execute:
        name = skill_msg.skill.strip().lower()
        fn = _SKILL_TABLE.get(name)

        if fn is None:
            logger.warning(f"[Scheduler] unsupported skill: {name}")
            continue

        try:
            params = {p.key: p.value for p in skill_msg.params}
            fn(swarm_manager, rc, rid, params)
        except Exception as e:
            logger.error(f"[Scheduler] start skill '{name}' error: {e}")


# Skill execution functions with dependency injection
def _skill_navigate_to(
        swarm_manager, rc: str, rid: int, params: Dict[str, Any], semantic_map
) -> None:
    """Execute navigate-to skill with injected semantic map

    Args:
        swarm_manager
        rc: Robot class name
        rid: Robot index
        params: Skill parameters containing 'goal' key
        semantic_map: Injected semantic map instance
    """
    pos = semantic_map.map_semantic[params["goal"]]
    swarm_manager.robot_active[rc][rid].navigate_to(pos)


def _skill_pick_up(swarm_manager, rc: str, rid: int, params: Dict[str, Any]) -> None:
    """Execute pick-up skill

    Args:
        swarm_manager
        rc: Robot class name
        rid: Robot index
        params: Skill parameters (unused for pick-up)
    """
    object_prim_path = params.get("object_prim_path")
    robot_prim_path = params.get("robot_prim_path")
    return swarm_manager.robot_active[rc][rid].pickup_object_if_close_unified(robot_hand_prim_path=robot_prim_path,
                                                                              object_prim_path=object_prim_path)


def _skill_put_down(swarm_manager, rc: str, rid: int, params: Dict[str, Any]) -> None:
    """Execute put-down skill

    Args:
        swarm_manager
        rc: Robot class name
        rid: Robot index
        params: Skill parameters (unused for put-down)
    """
    object_prim_path = params.get("object_prim_path")
    robot_prim_path = params.get("robot_prim_path")
    swarm_manager.robot_active[rc][rid].put_down(robot_hand_prim_path=robot_prim_path,
                                                 object_prim_path=object_prim_path)

def _skill_take_photo(swarm_manager, rc: str, rid: int, params: Dict[str, Any]) -> None:

    swarm_manager.robot_active[rc][rid].take_photo(file_path=params.get("file_path"))

_SKILL_TABLE = {
    "navigate-to": _skill_navigate_to,
    "pick-up": _skill_pick_up,
    "put-down": _skill_put_down,
}
