import rclpy
from rclpy.node import Node

from log.log_manager import LogManager

logger = LogManager.get_logger(__name__)


class RobotNode(Node):
    """
    为单个机器人实体管理所有ROS2通信的节点。
    """

    def __init__(self, robot_name: str):
        super().__init__(node_name=f"node_{robot_name}")
        self.robot_name = robot_name
        logger.info(f"ROS2 Node for {self.robot_name} has been created.")
