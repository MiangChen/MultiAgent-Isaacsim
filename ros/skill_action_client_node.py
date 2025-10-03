import rclpy
import asyncio
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.task import Future as RclpyFuture
from asyncio import Future as AsyncioFuture
import threading

from action_msgs.msg import GoalStatus
from gsi2isaacsim.gsi_msgs_helper import (
    PrimTransform,
    SceneModifications,
    RobotFeedback,
    VelTwistPose,
    RobotSkill,
    Parameter,
    PlanExecution,
    SkillExecution,
    SkillFeedback,
    SkillInfo,
)
from log.log_manager import LogManager

logger = LogManager.get_logger(__name__)


class SkillActionClientNode(Node):
    """
    一个通用的技能Action客户端，可以与任何一个RobotBase的skill_server通信。
    """

    def __init__(self, node_name="skill_client_node", loop=None):
        super().__init__(node_name)
        self._action_clients = {}
        self._client_lock = threading.Lock()
        self.loop = loop

    def get_action_client(self, robot_name: str) -> ActionClient:
        """
        获取或创建一个到特定机器人的ActionClient。
        Action名称格式为 /skill/{robot_name}
        """
        with self._client_lock:
            action_name = f"/skill/{robot_name}"
            if action_name not in self._action_clients:
                logger.info(f"Creating new action client for '{action_name}'...")
                self._action_clients[action_name] = ActionClient(
                    self, SkillExecution, action_name
                )
            return self._action_clients[action_name]

    def _wrap_rclpy_future(self, rclpy_future: RclpyFuture) -> AsyncioFuture:
        """
        将一个 rclpy.task.Future 转换为一个 asyncio.Future。
        这是连接 rclpy 线程和 asyncio 事件循环的关键。
        """
        # 1. 创建一个 asyncio 的 Future，这是我们将要 await 的对象
        aio_future = self.loop.create_future()

        # 2. 定义一个回调函数，当 rclpy_future 完成时，它会在 rclpy 的线程中被调用
        def on_done(done_rclpy_future: RclpyFuture):
            try:
                result = done_rclpy_future.result()
                # 3. 使用线程安全的方式，在 asyncio 事件循环中设置结果
                self.loop.call_soon_threadsafe(aio_future.set_result, result)
            except Exception as e:
                # 4. 同样，使用线程安全的方式，在 asyncio 事件循环中设置异常
                self.loop.call_soon_threadsafe(aio_future.set_exception, e)

        # 5. 将这个回调函数添加到 rclpy_future
        rclpy_future.add_done_callback(on_done)

        # 6. 返回 asyncio 的 Future
        return aio_future

    async def send_skill_goal(self, robot_skill_msg: RobotSkill, feedback_handler=None):
        """
        异步发送一个技能目标，并等待最终结果。
        feedback_handler: 一个可选的回调函数, 用于处理实时的feedback.可以接受(robot_name, feedback_msg)两个参数
        """
        robot_name = robot_skill_msg.robot_id
        skill_name = (
            robot_skill_msg.skill_list[0].skill if robot_skill_msg.skill_list else "unknown"
        )

        action_client = self.get_action_client(robot_name)
        if not await self.loop.run_in_executor(
                None, lambda: action_client.wait_for_server(timeout_sec=3.0)
        ):
            logger.error(f"Action server for '{robot_name}' not available after waiting.")
            return {"success": False, "message": "Server not available."}

        logger.info(f"Preparing to send skill '{skill_name}' to robot '{robot_name}'")

        # 创建顶层的 Goal 对象
        goal_msg = SkillExecution.Goal()
        goal_msg.skill_request = robot_skill_msg

        logger.info(f"Sending goal to '{robot_name}': {goal_msg}")

        def feedback_callback(feedback_msg):
            feedback = feedback_msg.feedback
            logger.info(f"Received feedback from '{robot_name}': {feedback.status}")

            if feedback_handler:
                try:
                    feedback_handler(robot_skill_msg, feedback)
                except Exception as e:
                    logger.error(f"Error in custom feedback handler for '{robot_name}': {e}")

        send_goal_rclpy_future = action_client.send_goal_async(
            goal=goal_msg, feedback_callback=feedback_callback
        )

        try:
            goal_handle = await self._wrap_rclpy_future(send_goal_rclpy_future)
        except Exception as e:
            logger.error(f"Failed to send goal: {e}")
            return {"success": False, "message": f"Exception while sending goal: {e}"}

        if not goal_handle.accepted:
            logger.warn(f"Goal for '{robot_name}' was rejected.")
            return {"success": False, "message": "Goal rejected by server."}

        logger.info(f"Goal for '{robot_name}' was accepted. Waiting for result...")

        get_result_rclpy_future = goal_handle.get_result_async()
        try:
            result_wrapper = await self._wrap_rclpy_future(get_result_rclpy_future)
            status = result_wrapper.status
            result = result_wrapper.result
            # 根据 SkillExecution.action 定义，result 有 `bool success` 和 `string message`
            logger.info(f"Got result from '{robot_name}' with status: {status}")
            return {
                "success": result.success,
                "message": result.message,
                "status": status,
            }
        except Exception as e:
            logger.error(f"Failed to get result: {e}")
            return {"success": False, "message": f"Exception while getting result: {e}"}
