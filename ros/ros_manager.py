import threading
import logging

logger = logging.getLogger(__name__)

from gsi_msgs.gsi_msgs_helper import Plan
import rclpy
rclpy.init(None)
from rclpy.executors import MultiThreadedExecutor
from ros.ros_node import PlanNode, SceneMonitorNode, SwarmNode, get_swarm_node
from skill.skill import _plan_cb


class RosManager:
    def __init__(self):
        self.plan_receiver_node = None
        self.scene_monitor_node = None
        self.swarm_node = None
        self.executor = None
        self.stop_event = threading.Event()
        self.thread = None

        self.build_nodes()

    def build_nodes(self):
        """构建所有ROS节点"""
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
        )
        self.plan_receiver_node = PlanNode('plan_receiver')
        self.plan_receiver_node.create_subscription(Plan, '/Plan', _plan_cb, qos)
        self.scene_monitor_node = SceneMonitorNode()
        self.swarm_node = get_swarm_node()
        logger.info("ROS nodes built successfully.")

    def start(self):
        """在后台线程中启动ROS节点"""
        if not self.plan_receiver_node:
            logger.warning("ROS nodes not built. Cannot start.")
            return
        nodes = [self.plan_receiver_node, self.scene_monitor_node, self.swarm_node]
        self.thread = threading.Thread(
            target=self._spin_in_background, args=(nodes, self.stop_event), daemon=True
        )
        self.thread.start()
        logger.info("ROS manager thread started.")

    def _spin_in_background(self):
        """后台运行ROS节点的实际工作函数"""
        self.executor = MultiThreadedExecutor(num_threads=4)
        nodes = [self.plan_receiver_node, self.scene_monitor_node, self.swarm_node]
        for n in nodes:
            if n:
                self.executor.add_node(n)

        try:
            while not self.stop_event.is_set():
                self.executor.spin_once(timeout_sec=0.05)
        finally:
            for n in nodes:
                if n and self.executor:
                    try:
                        self.executor.remove_node(n)
                    except Exception as e:
                        logger.error(f"Error removing node: {e}")
                    try:
                        n.destroy_node()
                    except Exception as e:
                        logger.error(f"Error destroying node: {e}")
            if rclpy.ok():
                rclpy.shutdown()
            logger.info("ROS executor shutdown complete.")

    def stop(self):
        """停止ROS线程"""
        if self.thread and self.thread.is_alive():
            self.stop_event.set()
            self.thread.join(timeout=3.0)
            if self.thread.is_alive():
                logger.warning("ROS thread did not stop gracefully.")
        logger.info("ROS manager stopped.")
