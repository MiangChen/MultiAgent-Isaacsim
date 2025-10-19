from rclpy.node import Node

from log.log_manager import LogManager

logger = LogManager.get_logger(__name__)


class NodeRobot(Node):
    """
    为单个机器人实体管理所有ROS2通信的节点。
    """

    def __init__(self, namespace: str):
        super().__init__(node_name=f"node_{namespace}", namespace=namespace)
        self.namespace = namespace
        logger.info(f"ROS2 Node for {self.namespace} has been created.")
