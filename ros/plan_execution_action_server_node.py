import asyncio
import threading

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
from ros.skill_action_client_node import SkillActionClientNode
from log.log_manager import LogManager

logger = LogManager.get_logger(__name__)


class PlanExecutionServer(Node):
    """
    并行的任务调度动作服务器（调度器/Orchestrator）。
    """

    def __init__(self, loop, swarm_manager):
        super().__init__(node_name="plan_execution_action_server")
        self.loop = loop  # Store the main event loop
        self.plan_execution_action_server = ActionServer(
            self,
            PlanExecution,
            action_name="/isaac_sim/plan_execution",
            execute_callback=self.execute_callback_wrapper,
        )

        self.skill_client_action_server = SkillActionClientNode(node_name="skill_client_action_server")
        self._client_lock = threading.Lock()
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
            tasks = []
            for robot_skill in step.robots:
                robot_name = robot_skill.robot_id

                # 假设每个 RobotSkill 消息中至少有一个 skill
                if not robot_skill.skill_list:
                    logger.warn(
                        f"Robot '{robot_name}' has an empty skill list in timestep {step.timestep}. Skipping."
                    )
                    continue

                skill_info = robot_skill.skill_list[0]
                skill_name = skill_info.skill
                params = {p.key: p.value for p in skill_info.params}

                task = self.skill_client_action_server.send_skill_goal(
                    robot_name=robot_name, skill_name=skill_name, params=params
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
                            f"  - Robot '{failed_robot}' failed with message: {res.get('message')}"
                        )

                goal_handle.abort()
                return PlanExecution.Result(success=False, message=error_msg)

            logger.info(f"--- Timestep {step.timestep} Completed Successfully ---")
        goal_handle.succeed()
        logger.info("✅ Plan dispatched and executed successfully.")
        return PlanExecution.Result(success=True, message="Plan executed successfully.")

    # def _publish_aggregated_feedback(
    #         self, goal_handle, current_timestep, feedback_state
    # ):
    #     agg_feedback = PlanExecution.Feedback()
    #     agg_feedback.current_timestep = current_timestep
    #     agg_feedback.skill_statuses = list(feedback_state.values())
    #     if goal_handle.is_active:
    #         goal_handle.publish_feedback(agg_feedback)
