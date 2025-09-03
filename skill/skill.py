from collections import defaultdict, deque
import threading
from typing import Dict, Any

from gsi_msgs.gsi_msgs_helper import Plan

# Global variables for ROS integration
_skill_queues = defaultdict(deque)
_skill_lock = threading.Lock()


def _param_dict(params) -> Dict[str, Any]:
    """Convert ROS parameter list to dictionary

    Args:
        params: List of ROS Parameter objects

    Returns:
        Dict[str, Any]: Dictionary mapping parameter keys to values
    """
    d = {}
    if params:
        for p in params:
            d[p.key] = p.value
    return d


def _parse_robot_id(robot_id: str) -> tuple[str, int]:
    """Parse robot ID string to extract robot class and index

    Args:
        robot_id: Robot identifier string (e.g., "jetbot_1", "h1-2")

    Returns:
        tuple[str, int]: Tuple of (robot_class_name, robot_index)
    """
    import re

    s = robot_id or ""
    m = re.match(r"^([A-Za-z]\w*?)[_-]?(\d+)$", s)
    if not m:
        return "jetbot", 0
    name = m.group(1)
    idx = int(m.group(2))
    return name, idx


def _plan_cb(msg: Plan):
    """Plan callback to queue skills for robots"""
    try:
        steps = sorted(msg.steps, key=lambda s: s.timestep)
        with _skill_lock:
            for ts in steps:
                for rs in ts.robots:
                    rc, rid = _parse_robot_id(rs.robot_id)
                    q = _skill_queues[(rc, rid)]
                    for sk in rs.skill_list:
                        q.append(sk)
    except Exception as e:
        logger.error(f"[PlanCB] Error: {e}")


def process_ros_skills(swarm_manager) -> None:
    """Process ROS skill queue and execute skills with injected SwarmManager

    Args:
        swarm_manager: Injected swarm manager instance
    """
    # Check if all robots have completed their current skills

    state_skill_complete_all = True
    for robot_class in swarm_manager.robot_class:
        for robot in swarm_manager.robot_active[robot_class]:
            done = getattr(robot, "state_skill_complete", True)
            state_skill_complete_all = state_skill_complete_all and bool(done)

    if state_skill_complete_all:
        # All robots completed -> get next skill for each robot
        from skill.skill import _skill_queues, _skill_lock, _param_dict, _SKILL_TABLE

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
                    logger.warning(f"[Scheduler] unsupported skill: {name}")
                else:
                    try:
                        fn(swarm_manager, rc, rid, params)
                    except Exception as e:
                        logger.error(f"[Scheduler] start skill error: {e}")


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
    swarm_manager.robot_swarm.robot_active[rc][rid].pick_up()


def _skill_put_down(swarm_manager, rc: str, rid: int, params: Dict[str, Any]) -> None:
    """Execute put-down skill

    Args:
        swarm_manager
        rc: Robot class name
        rid: Robot index
        params: Skill parameters (unused for put-down)
    """
    swarm_manager.robot_active[rc][rid].put_down()


_SKILL_TABLE = {
    "navigate-to": _skill_navigate_to,
    "pick-up": _skill_pick_up,
    "put-down": _skill_put_down,
}
