from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from log.log_manager import LogManager

logger = LogManager.get_logger(__name__)


class NodeRobot(Node):
    """
    为单个机器人实体管理所有ROS2通信的节点。
    """

    def __init__(self, namespace: str):
        super().__init__(node_name=f"node_{namespace}", namespace=namespace)
        self.namespace = namespace
        self.publisher_odom = self.create_publisher(Odometry, 'odom', 10)
        self.subscriber_cmd_vel = self.create_subscription(Twist, 'cmd_vel', self.callback_cmd_vel, 10)
        logger.info(f"ROS2 Node for {self.namespace} has been created.")

    def callback_cmd_vel(self, msg):
        pass
