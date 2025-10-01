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
from log.log_manager import LogManager

logger = LogManager.get_logger(__name__)


class SwarmNode(Node):

    def __init__(self):
        super().__init__("swarm")
        # 结构: { robot_class: { robot_id: { "motion": publisher, "feedback": publisher } } }
        self.publisher_dict: dict[str, dict[int, dict[str, any]]] = {}
        self.subscriber_dict: dict[str, dict[int, dict[str, any]]] = {}

    def register_feedback_publisher(self, robot_class: str, robot_id: int, qos=10):
        """
        注册一个 feedback publisher
        topic 规则: /feedback/<robot_class>_<robot_id>
        """
        # 如果类别不存在，先建字典
        if robot_class not in self.publisher_dict:
            self.publisher_dict[robot_class] = {}
        # 如果 robot_id 不存在，先建子字典
        if robot_id not in self.publisher_dict[robot_class]:
            self.publisher_dict[robot_class][robot_id] = {}

        topic = f"/feedback/{robot_class}_{robot_id}"
        pub = self.create_publisher(RobotFeedback, topic, qos)

        # 存到子字典里
        self.publisher_dict[robot_class][robot_id]["feedback"] = pub

        #        self.get_logger().info(
        #            f"Registered feedback publisher for {robot_class}[{robot_id}] on topic {topic}"
        #        )
        return pub

    def register_motion_publisher(self, robot_class: str, robot_id: int, qos=50):
        """
        注册一个 motion publisher
        topic 规则: /motion/<robot_class>_<robot_id>
        """
        if robot_class not in self.publisher_dict:
            self.publisher_dict[robot_class] = {}
        if robot_id not in self.publisher_dict[robot_class]:
            self.publisher_dict[robot_class][robot_id] = {}

        topic = f"/motion/{robot_class}_{robot_id}"
        pub = self.create_publisher(VelTwistPose, topic, qos)

        self.publisher_dict[robot_class][robot_id]["motion"] = pub

        return pub

    def register_cmd_subscriber(
        self, robot_class: str, robot_id: int, callback=None, qos=50
    ):
        """
        注册一个cmd subscriber
        topic 规则： /cmd/<robot_class>_<robot_id>
        """
        if robot_class not in self.subscriber_dict:
            self.subscriber_dict[robot_class] = {}
        # 如果 robot_id 不存在，先建子字典
        if robot_id not in self.subscriber_dict[robot_class]:
            self.subscriber_dict[robot_class][robot_id] = {}

        topic = f"/cmd/{robot_class}_{robot_id}"
        sub = self.create_subscription(VelTwistPose, topic, callback, qos)

        self.subscriber_dict[robot_class][robot_id]["cmd"] = sub

        #        self.get_logger().info(
        #            f"Registered cmd subscriber for {robot_class}[{robot_id}] on topic {topic}"
        #        )
        return sub

    def publish_feedback(self, robot_class: str, robot_id: int, msg: RobotFeedback):
        self.publisher_dict[robot_class][robot_id]["feedback"].publish(msg)

    def publish_motion(self, robot_class: str, robot_id: int, msg: VelTwistPose):
        self.publisher_dict[robot_class][robot_id]["motion"].publish(msg)

    def update_swarm_data(self):
        self.shared_data["robot_count"] = len(self.robot_positions)
        self.shared_data["formation_type"] = "triangle"
        self.publish_data("robot_count", len(self.robot_positions))
        self.publish_data("formation_type", "triangle")

    def get_robot_positions(self):
        return self.robot_positions

    def request_map_info(self):
        """请求地图信息"""

        def handle_map_response(value):
            logger.info(f"Received map size: {value}")

        self.query_node_data("map", "map_size", handle_map_response)
