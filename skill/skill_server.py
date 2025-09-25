import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from log.log_manager import LogManager
from gsi_msgs_helper import SkillExecution

logger = LogManager.get_logger(__name__)

class SkillServer(Node):
    """
    这个节点模拟了多个技能的执行。
    它会为每个指定的技能名称启动一个专用的Action Server。
    """

    def __init__(self):
        super().__init__("skill_server_node")
        self.skill_servers = []

        # 定义我们想要模拟的技能列表
        skill_names = ["navigate", "load_object", "unload_object", "take_picture"]

        for skill_name in skill_names:
            server = ActionServer(
                self,
                SkillExecution,
                f"/{skill_name}",  # Action topic is named after the skill
                self.execute_callback,
            )
            self.skill_servers.append(server)
            self.get_logger().info(
                f"✅ Action Server for '/{skill_name}' is ready."
            )

    def execute_callback(self, goal_handle):
        """
        模拟技能执行的通用回调函数。
        """
        skill_req = goal_handle.request.skill_request
        skill_name = skill_req.skill_list[0].skill
        robot_id = skill_req.robot_id

        self.get_logger().info(
            f"[{skill_name.upper()}] Robot '{robot_id}' received request. Simulating execution..."
        )

        # 模拟耗时工作
        feedback_msg = SkillExecution.Feedback()
        for i in range(1, 11):
            feedback_msg.status = (
                f"Robot '{robot_id}' executing '{skill_name}', progress {i*10}%"
            )
            self.get_logger().info(
                f"Feedback from [{skill_name.upper()}|{robot_id}]: {feedback_msg.status}"
            )
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.5)  # 模拟0.5秒的延迟

        goal_handle.succeed()
        self.get_logger().info(
            f"[{skill_name.upper()}] Robot '{robot_id}' finished successfully ✅"
        )
        result = SkillExecution.Result()
        result.success = True
        result.message = f"'{skill_name}' completed successfully by '{robot_id}'."

        self.get_logger().info(
            f"✅ [{skill_name.upper()}] Execution finished for robot '{robot_id}'."
        )
        return result


def main(args=None):
    rclpy.init(args=args)
    skill_server_node = SkillServer()

    # 多线程执行器是必须的，这样可以同时处理对不同技能的请求
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(skill_server_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        skill_server_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
