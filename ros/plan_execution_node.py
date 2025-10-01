import asyncio
from typing import Dict
import traceback
import threading

from geometry_msgs.msg import Transform as RosTransform
from rclpy.action import ActionServer, ActionClient, GoalResponse
from rclpy.node import Node
from rclpy.task import Future as RclpyFuture

import omni.usd
from pxr import Tf, Gf

from action_msgs.msg import GoalStatus
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
from log.log_manager import LogManager

logger = LogManager.get_logger(__name__)


class PlanExecutionServer(Node):
    """
    并行的任务调度动作服务器（调度器/Orchestrator）。
    """

    def __init__(self, loop, swarm_manager):
        super().__init__(node_name="plan_execution_server")
        self.loop = loop  # Store the main event loop
        self._action_server = ActionServer(
            self,
            PlanExecution,
            action_name="/isaac_sim/plan_execution",
            execute_callback=self.execute_callback_wrapper,
        )

        # self._skill_clients: Dict[str, ActionClient] = {}
        # self._client_lock = threading.Lock()
        logger.info("✅ Parallel Plan Dispatch Server is ready.")

    # This function is called by the ROS executor in a worker thread.
    def execute_callback_wrapper(self, goal_handle):
        """
        Schedules the async implementation on the main event loop and waits for the result.
        """

        # Schedule the coroutine on the main event loop
        future = asyncio.run_coroutine_threadsafe(
            self.async_execute_callback(goal_handle), self.loop
        )

        return future.result()

    async def async_execute_callback(self, goal_handle):
        plan = goal_handle.request.plan
        logger.info(f"Dispatching plan with {len(plan.steps)} timesteps...")

        for step in plan.steps:
            logger.info(f"--- Starting Timestep {step.timestep} ---")

        #     # Initialize feedback state for this timestep
        #     timestep_feedback_state: Dict[str, SkillFeedback] = {}
        #     for robot_skill in step.robots:
        #         skill_id = f"{robot_skill.robot_id}-{robot_skill.skill_list[0].skill}-{id(robot_skill)}"
        #         timestep_feedback_state[skill_id] = SkillFeedback(
        #             skill_id=skill_id, status="Pending"
        #         )
        #
        #     # Concurrently execute all skills for the current timestep
        #     tasks = [
        #         self.dispatch_skill(
        #             goal_handle,
        #             robot_skill,
        #             step.timestep,
        #             f"{robot_skill.robot_id}-{robot_skill.skill_list[0].skill}-{id(robot_skill)}",
        #             timestep_feedback_state,
        #         )
        #         for robot_skill in step.robots
        #     ]
        #     results = await asyncio.gather(*tasks, return_exceptions=True)
        #
        #     # Check if any task failed or raised an exception
        #     if not all(res is True for res in results):
        #         # Log any exceptions that occurred
        #         for i, res in enumerate(results):
        #             if isinstance(res, Exception):
        #                 logger.error(f"Exception during skill dispatch: {res}")
        #                 traceback.print_exc()
        #
        #         error_msg = f"Execution failed in timestep {step.timestep}."
        #         logger.error(error_msg)
        #         goal_handle.abort()
        #         return PlanExecution.Result(success=False, message=error_msg)
        #
        #     logger.info(f"--- Timestep {step.timestep} Completed Successfully ---")

        goal_handle.succeed()
        logger.info("✅ Plan dispatched and executed successfully.")
        return PlanExecution.Result(success=True, message="Plan executed successfully.")

    # async def dispatch_skill(
    #     self,
    #     plan_goal_handle,
    #     robot_skill: RobotSkill,
    #     current_timestep: int,
    #     skill_id: str,
    #     feedback_state: dict,
    # ):
    #     skill_name = robot_skill.skill_list[0].skill
    #     action_name = f"/{skill_name}"
    #     logger.info(f"Dispatching skill '{skill_id}' on action '{action_name}'")
    #
    #     try:
    #         action_client = self._get_or_create_client(action_name)
    #         if not await self._wait_for_server(action_client, timeout_sec=3.0):
    #             raise RuntimeError(f"Action server '{action_name}' not available.")
    #
    #         skill_goal = SkillExecution.Goal(skill_request=robot_skill)
    #
    #         def skill_feedback_callback(feedback_msg):
    #             status = feedback_msg.feedback.status
    #             if skill_id in feedback_state:
    #                 feedback_state[skill_id].status = status
    #                 self._publish_aggregated_feedback(
    #                     plan_goal_handle, current_timestep, feedback_state
    #                 )
    #
    #         send_goal_future = action_client.send_goal_async(
    #             skill_goal, feedback_callback=skill_feedback_callback
    #         )
    #         goal_handle = await self._wrap_ros_future(send_goal_future)
    #
    #         if not goal_handle.accepted:
    #             raise RuntimeError(f"Skill '{skill_id}' was rejected by the server.")
    #
    #         logger.info(f"Skill '{skill_id}' accepted by server.")
    #         feedback_state[skill_id].status = "Executing"
    #         self._publish_aggregated_feedback(
    #             plan_goal_handle, current_timestep, feedback_state
    #         )
    #
    #         result_wrapper = await self._wrap_ros_future(goal_handle.get_result_async())
    #
    #         if result_wrapper.status != GoalStatus.STATUS_SUCCEEDED:
    #             raise RuntimeError(
    #                 f"Skill '{skill_id}' did not succeed. Status: {result_wrapper.status}"
    #             )
    #
    #         if result_wrapper.result.success:
    #             logger.info(f"Skill '{skill_id}' completed successfully.")
    #             feedback_state[skill_id].status = "Success"
    #             self._publish_aggregated_feedback(
    #                 plan_goal_handle, current_timestep, feedback_state
    #             )
    #             return True
    #         else:
    #             raise RuntimeError(
    #                 f"Skill '{skill_id}' failed with message: {result_wrapper.result.message}"
    #             )
    #
    #     except Exception as e:
    #         logger.error(f"Exception for skill '{skill_id}': {type(e).__name__} - {e}")
    #         feedback_state[skill_id].status = f"Error: {e}"
    #         self._publish_aggregated_feedback(
    #             plan_goal_handle, current_timestep, feedback_state
    #         )
    #         # Re-raise the exception so asyncio.gather can catch it
    #         raise

    # def _get_or_create_client(self, action_name: str) -> ActionClient:
    #     with self._client_lock:
    #         if action_name not in self._skill_clients:
    #             logger.info(f"Creating new action client for '{action_name}'...")
    #             self._skill_clients[action_name] = ActionClient(
    #                 self, SkillExecution, action_name
    #             )
    #         return self._skill_clients[action_name]

    def _publish_aggregated_feedback(
        self, goal_handle, current_timestep, feedback_state
    ):
        agg_feedback = PlanExecution.Feedback()
        agg_feedback.current_timestep = current_timestep
        agg_feedback.skill_statuses = list(feedback_state.values())
        if goal_handle.is_active:
            goal_handle.publish_feedback(agg_feedback)

    # async def _wait_for_server(
    #     self, action_client: ActionClient, timeout_sec=2.0
    # ) -> bool:
    #     """Asynchronously wait for an Action Server to be available."""
    #     # This function is now called from the main asyncio thread, so we can run
    #     # the blocking wait_for_server call in the loop's default executor (a thread pool).
    #     return await self.loop.run_in_executor(
    #         None, lambda: action_client.wait_for_server(timeout_sec=timeout_sec)
    #     )
    #
    # async def _wrap_ros_future(self, rclpy_future: RclpyFuture):
    #     """Wrap an rclpy.Future in an asyncio.Future."""
    #     # This function is also now called from the main asyncio thread.
    #     aio_future = self.loop.create_future()
    #
    #     def on_done(ros_future):
    #         try:
    #             result = ros_future.result()
    #             self.loop.call_soon_threadsafe(aio_future.set_result, result)
    #         except Exception as e:
    #             self.loop.call_soon_threadsafe(aio_future.set_exception, e)
    #
    #     rclpy_future.add_done_callback(on_done)
    #     return await aio_future


#
#
# class SkillServerNode(Node):
#     """
#     这个节点模拟了多个技能的执行。
#     它会为每个指定的技能名称启动一个专用的Action Server。
#     """
#
#     def __init__(self):
#         super().__init__("skill_server_node")
#         self.skill_servers = []
#
#         # 定义我们想要模拟的技能列表
#         skill_names = ["navigate", "load_object", "unload_object", "take_picture"]
#
#         for skill_name in skill_names:
#             server = ActionServer(
#                 self,
#                 SkillExecution,
#                 f"/{skill_name}",  # Action topic is named after the skill
#                 self.execute_callback,
#             )
#             self.skill_servers.append(server)
#             logger.info(f"✅ Mock Action Server for '/{skill_name}' is ready.")
#
#     def execute_callback(self, goal_handle):
#         """
#         模拟技能执行的通用回调函数。
#         """
#         try:
#             skill_req = goal_handle.request.skill_request
#             skill_name = skill_req.skill_list[0].skill
#             params_list = skill_req.skill_list[0].params
#             params_dict = {param.key: param.value for param in params_list}
#             robot = skill_req.robot_id
#             robot_name, robot_id = robot.split("-")
#             robot_id = int(robot_id)
#             logger.info(
#                 f"[{skill_name.upper()}] Robot '{robot}' received request. Simulating execution..."
#             )
#
#             feedback_msg = SkillExecution.Feedback()
#
#             # 使用skill manager
#             from containers import get_container
#
#             container = get_container()
#
#             skill_manager = container.skill_manager()
#
#             if robot_name.lower() in ["ugv", "jetbot"]:
#                 skill_manager._SKILL_TABLE.get(skill_name)(
#                     rc="jetbot", rid=robot_id, params=params_dict
#                 )
#
#             elif robot_name.lower() in ["uav", "drone", "cf2x"]:
#                 skill_manager._SKILL_TABLE.get(skill_name)(
#                     rc="cf2x", rid=robot_id, params=params_dict
#                 )
#
#             feedback_msg.status = f"Robot '{robot_id}' executing '{skill_name}"
#             logger.info(
#                 f"Feedback from [{skill_name.upper()}|{robot_id}]: {feedback_msg.status}"
#             )
#             goal_handle.publish_feedback(feedback_msg)
#
#             goal_handle.succeed()
#             logger.info(
#                 f"[{skill_name.upper()}] Robot '{robot_id}' finished successfully ✅"
#             )
#             result = SkillExecution.Result()
#             result.success = True
#             result.message = f"'{skill_name}' completed successfully by '{robot_id}'."
#
#             logger.info(
#                 f"✅ [{skill_name.upper()}] Execution finished for robot '{robot_id}'."
#             )
#             return result
#         except Exception as e:
#             logger.error(f"!!!!!! EXCEPTION IN SKILL EXECUTION CALLBACK !!!!!!")
#             logger.error(f"Skill: {skill_req.skill_list[0].skill}, Robot: {skill_req.robot_id}")
#             logger.error(f"Error Type: {type(e).__name__}")
#             logger.error(f"Error Message: {e}")
