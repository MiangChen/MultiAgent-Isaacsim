# =============================================================================
# Node Robot Module - Individual Robot ROS2 Communication
# =============================================================================
#
# This module provides ROS2 node implementation for managing all ROS2
# communication for individual robot entities, including odometry publishing
# and command velocity subscription.
#
# =============================================================================

# Standard library imports
import threading

# Local project imports
from log.log_manager import LogManager

# ROS2 imports
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# Custom ROS message imports
from gsi_msgs.gsi_msgs_helper import SkillExecution

logger = LogManager.get_logger(__name__)


class NodeRobot(Node):
    """
    为单个机器人实体管理所有ROS2通信的节点。
    """

    def __init__(self, namespace: str):
        super().__init__(node_name=f"node_{namespace}", namespace=namespace)
        self.namespace = namespace
        self.robot_instance = None

        self.publisher_odom = self.create_publisher(Odometry, "odom", 10)
        self.subscriber_cmd_vel = self.create_subscription(
            Twist, "cmd_vel", self.callback_cmd_vel, 10
        )
        self.action_server_skill = ActionServer(
            self,
            action_type=SkillExecution,
            action_name=f"skill_execution",
            execute_callback=self.callback_execute_skill,
        )
        logger.info(f"ROS2 Node for {self.namespace} has been created.")

    def callback_cmd_vel(self, msg):
        if self.robot_instance and hasattr(self.robot_instance, "callback_cmd_vel"):
            self.robot_instance.callback_cmd_vel(msg)
        else:
            self.get_logger().warning("No robot instance connected for cmd_vel")

    def callback_execute_skill(self, goal_handle):
        return self.robot_instance.callback_task_execution(goal_handle)

    def set_robot_instance(self, robot):
        self.robot_instance = robot

    # 移除独立的spinning方法，由robot.py的统一executor管理
    # def start_spinning(self):
    #     if self.executor is None:
    #         self.executor = MultiThreadedExecutor()
    #     self.executor.add_node(self)
    #     self.thread.start()
    #     self.get_logger().info("Planner node spinning started in its own thread.")

    # def _spin(self):
    #     try:
    #         self.executor.spin()
    #     except Exception as e:
    #         self.get_logger().error(f"Spin failed in node {self.namespace}: {e}")