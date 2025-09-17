from collections import defaultdict, deque
import threading
from typing import Dict, Any

from log.log_manager import LogManager
from gsi2isaacsim.gsi_msgs_helper import Plan
from map.map_semantic_map import MapSemantic

# Global variables for ROS integration
_skill_queues = defaultdict(deque)
_skill_lock = threading.Lock()

logger = LogManager.get_logger(__name__)


class SkillManager:

    def __init__(self, semantic_map: MapSemantic, swarm_manager):

        self.previous_skill = []
        self.semantic_map = semantic_map
        self.swarm_manager = swarm_manager

        self._SKILL_TABLE = {
            "navigate": self._skill_navigate_to,
            "pick-up": self._skill_pick_up,
            "load_object": self._skill_pick_up,
            "put-down": self._skill_put_down,
            "unload_object": self._skill_put_down,
            "take_photo": self._skill_take_photo,
        }

        # 初始化时打印支持的技能表
        logger.info("SkillManager initialized with skills: %s", list(self._SKILL_TABLE.keys()))

    def _parse_robot_id(self, robot_id: str) -> tuple[str, int]:
        """Parse robot ID string to extract robot class and index

        Args:
            robot_id: Robot identifier string (e.g., "jetbot_1", "h1-2")

        Returns:
            tuple[str, int]: Tuple of (robot_class_name, robot_index)
        """
        import re

        # 使用 := 将 re.match 的结果赋值给 m 并同时进行判断
        if m := re.match(r"^([A-Za-z]\w*?)[_-]?(\d+)$", robot_id or ""):
            rc, rid = m.group(1), int(m.group(2))
            return rc, rid
        else:
            logger.warning(f"Failed to parse robot_id='{robot_id}', using default (jetbot,0)")
            return "jetbot", 0

    def _plan_cb(self, msg: Plan):
        """Plan callback to queue skills for robots"""
        try:
            steps = sorted(msg.steps, key=lambda s: s.timestep)
            if self.previous_skill == steps:
                # 如果 plan 没有变化，则跳过
                logger.info("[PlanCB] Received duplicate plan, skipping")
                return

            self.previous_skill = steps
            logger.info("[PlanCB] Received new plan with %d steps", len(steps))

            with _skill_lock:
                # 遍历时间步的任务
                for ts in steps:
                    for rs in ts.robots:
                        rc, rid = self._parse_robot_id(rs.robot_id)
                        q = _skill_queues[(rc, rid)]
                        q.extend(rs.skill_list)
                        logger.info(f"[PlanCB] Queued {len(rs.skill_list)} skills for {rc}-{rid}")
        except Exception as e:
            logger.error(f"[PlanCB] Error: {e}")

    def process_ros_skills(self) -> None:
        """Process ROS skill queue and execute skills with injected SwarmManager
        """
        # 1. Check if all robots have completed their current skills
        state_skill_complete_all = all(
            getattr(robot, "state_skill_complete", True)
            for robot_class in self.swarm_manager.robot_class
            for robot in self.swarm_manager.robot_active[robot_class]
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
            fn = self._SKILL_TABLE.get(name)

            if fn is None:
                logger.warning(f"[Scheduler] Unsupported skill: {name}")
                continue

            try:
                params = {p.key: p.value for p in skill_msg.params}
                fn(self.swarm_manager, rc, rid, params)
            except Exception as e:
                logger.error(f"[Scheduler] Start skill '{name}' error: {e}")

    # Skill execution functions with dependency injection
    def _skill_navigate_to(self,
                           rc: str, rid: int, params: Dict[str, Any]) -> None:
        """Execute navigate-to skill with injected semantic map

        Args:
            swarm_manager
            rc: Robot class name
            rid: Robot index
            params: Skill parameters containing 'area' key
        """
        #        pos = json.loads(params["area"])
        pos = [-4.5, 11.4, 0]  # FIXME: 临时覆盖
        logger.info(f"[Skill] {rc}-{rid} navigating to {pos}")
        return self.swarm_manager.robot_active[rc][rid].navigate_to(pos)

    def _skill_pick_up(self, rc: str, rid: int, params: Dict[str, Any]) -> None:
        """Execute pick-up skill

        Args:
            swarm_manager
            rc: Robot class name
            rid: Robot index
            params: Skill parameters (unused for pick-up)
        """
        logger.info(f"[Skill] {rc}-{rid} executing pick-up")
        object_prim_path = params.get("object_prim_path")
        robot_prim_path = params.get("robot_prim_path")
        return self.swarm_manager.robot_active[rc][rid].pickup_object_if_close_unified(
            robot_hand_prim_path=robot_prim_path,
            object_prim_path=object_prim_path)

    def _skill_put_down(self, rc: str, rid: int, params: Dict[str, Any]) -> None:
        """Execute put-down skill

        Args:
            swarm_manager
            rc: Robot class name
            rid: Robot index
            params: Skill parameters (unused for put-down)
        """
        logger.info(f"[Skill] {rc}-{rid} executing put-down")
        object_prim_path = params.get("object_prim_path")
        robot_prim_path = params.get("robot_prim_path")
        return self.swarm_manager.robot_active[rc][rid].put_down(robot_hand_prim_path=robot_prim_path,
                                                                 object_prim_path=object_prim_path)

    def _skill_take_photo(self, rc: str, rid: int, params: Dict[str, Any]) -> None:
        """Execute take-photo skill

        Args:
            swarm_manager
            rc: Robot class name
            rid: Robot index
            params: Skill parameters (requires 'file_path')
        """
        logger.info(f"[Skill] {rc}-{rid} executing take-photo")
        file_path = params.get("file_path")
        return self.swarm_manager.robot_active[rc][rid].take_photo(file_path=file_path)
