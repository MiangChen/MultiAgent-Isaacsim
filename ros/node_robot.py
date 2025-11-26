# Local project imports
from log.log_manager import LogManager

# ROS2 imports
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

logger = LogManager.get_logger(__name__)


class NodeRobot(Node):
    """
    机器人基础 ROS2 通信节点。

    职责：
    - cmd_vel subscriber: 接收外部速度控制指令
    - odom publisher: 发布里程计数据
    """

    def __init__(self, namespace: str, topics: dict = None):
        super().__init__(node_name=f"node_{namespace}", namespace=namespace)
        self.namespace = namespace
        self.robot_instance = None
        self.topics = topics or {}

        # 创建publishers和subscribers
        self._create_publishers()
        self._create_subscribers()

        logger.info(
            f"ROS2 Node for {self.namespace} has been created with topics: {list(self.topics.keys())}"
        )

    def _create_publishers(self):
        """根据配置创建publishers"""
        self.publisher_dict = {}

        if "odom" in self.topics:
            self.publisher_odom = self.create_publisher(
                Odometry, self.topics["odom"], 10
            )
            self.publisher_dict["odom"] = self.publisher_odom

    def _create_subscribers(self):
        """根据配置创建subscribers"""
        self.subscriber_dict = {}

        # cmd_vel subscriber (for external control)
        if "cmd_vel" in self.topics:
            self.subscriber_cmd_vel = self.create_subscription(
                Twist, self.topics["cmd_vel"], self.callback_cmd_vel, 10
            )
            self.subscriber_dict["cmd_vel"] = self.subscriber_cmd_vel

    def callback_cmd_vel(self, msg: Twist):
        """
        cmd_vel callback: Convert ROS Twist to Control object

        This allows external control via ROS topics (e.g., teleop, joystick)
        """
        from simulation.control.command import RobotControl

        control = RobotControl()
        control.linear_velocity = [msg.linear.x, msg.linear.y, msg.linear.z]
        control.angular_velocity = [msg.angular.x, msg.angular.y, msg.angular.z]

        if self.robot_instance:
            self.robot_instance.apply_control(control)

    def set_robot_instance(self, robot):
        self.robot_instance = robot
