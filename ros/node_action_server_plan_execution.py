import asyncio
import functools
import threading
import uuid

from rclpy.action import ActionServer
from rclpy.node import Node

from gsi2isaacsim.gsi_msgs_helper import (
    PrimTransform,
    SceneModifications,
    RobotFeedback,
    VelTwistPose,
    RobotSkill,
    PlanExecution,
    SkillExecution,
    SkillFeedback,
)
from ros.node_action_client_skill import NodeActionClientSkill
from log.log_manager import LogManager

logger = LogManager.get_logger(__name__)


class NodeActionServerPlanExecution(Node):
    """
    并行的任务调度动作服务器（调度器/Orchestrator）。
    """

    def __init__(self, loop):
        super().__init__(node_name="node_action_server_plan_execution")
        self.loop = loop
        self.action_server_plan_execution = ActionServer(
            self,
            PlanExecution,
            action_name="/isaac_sim/plan_execution",
            execute_callback=self.execute_callback_wrapper,
        )
        self.action_client_skill = NodeActionClientSkill(
            node_name="action_client_skill", loop=self.loop
        )

        self._feedback_state = {}
        self._feedback_lock = threading.Lock()

        logger.info("✅ Parallel Plan Dispatch Server is ready.")

    def execute_callback_wrapper(self, goal_handle):
        """
        This function is called by the ROS executor in a worker thread.
        Schedules the async implementation on the main event loop and waits for the result.
        """

        future = asyncio.run_coroutine_threadsafe(
            self.async_execute_callback(goal_handle), self.loop
        )

        return future.result()

    async def async_execute_callback(self, goal_handle):
        plan = goal_handle.request.plan
        logger.info(f"Dispatching plan with {len(plan.steps)} timesteps...")

        for step in plan.steps:
            logger.info(f"--- Starting Timestep {step.timestep} ---")
            with self._feedback_lock:
                self._feedback_state.clear()

            tasks = []
            for robot_skill_msg in step.robots:
                if not robot_skill_msg.skill_list:
                    continue

                feedback_handler = functools.partial(
                    self._handle_skill_feedback,
                    goal_handle=goal_handle,
                    current_timestep=step.timestep,
                )

                task = self.action_client_skill.send_skill_goal(
                    robot_skill_msg=robot_skill_msg,
                    feedback_handler=feedback_handler,
                )
                tasks.append(task)

            if not tasks:
                logger.info(
                    f"Timestep {step.timestep} has no tasks. Moving to next step."
                )
                continue

            logger.info(
                f"Dispatching {len(tasks)} concurrent skills for timestep {step.timestep}..."
            )

            results = await asyncio.gather(*tasks)
            if not all(res.get("success", False) for res in results):
                error_msg = f"Execution failed in timestep {step.timestep}."
                logger.error(error_msg)
                for i, res in enumerate(results):
                    if not res.get("success", False):
                        failed_robot = step.robots[i].robot_id
                        logger.error(
                            f"Robot '{failed_robot}' failed with message: {res.get('message')}"
                        )

                goal_handle.abort()
                return PlanExecution.Result(success=False, message=error_msg)

            logger.info(f"--- Timestep {step.timestep} Completed Successfully ---")
        goal_handle.succeed()
        logger.info("✅ Plan dispatched and executed successfully.")
        return PlanExecution.Result(success=True, message="Plan executed successfully.")

    def _handle_skill_feedback(
        self,
        robot_skill_msg: RobotSkill,
        skill_execution_feedback: SkillExecution.Feedback,
        goal_handle,
        current_timestep: int,
    ):
        """
        处理单个技能的反馈，并聚合发布 PlanExecution 的反馈。
        """
        robot_name = robot_skill_msg.robot_id
        skill_info = robot_skill_msg.skill_list[0]

        with self._feedback_lock:
            current_skill_feedback = self._feedback_state.get(robot_name)
            if not current_skill_feedback:
                current_skill_feedback = SkillFeedback()
                current_skill_feedback.robot_id = robot_name
                current_skill_feedback.skill_name = skill_info.skill
                current_skill_feedback.skill_id = str(uuid.uuid4())
                self._feedback_state[robot_name] = current_skill_feedback

            current_skill_feedback.status = skill_execution_feedback.status

            agg_feedback = PlanExecution.Feedback()
            agg_feedback.current_timestep = current_timestep
            agg_feedback.skill_statuses = list(self._feedback_state.values())

            if goal_handle.is_active:
                logger.info(
                    f"Publishing aggregated feedback for timestep {current_timestep} with {len(agg_feedback.skill_statuses)} statuses."
                )
                goal_handle.publish_feedback(agg_feedback)
