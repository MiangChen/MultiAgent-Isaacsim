import threading

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from gsi2isaacsim.gsi_msgs_helper import Plan
from ros.plan_execution_action_server_node import PlanExecutionServer
from ros.scene_monitor_node import SceneMonitorNode
from ros.swarm_node import SwarmNode
from log.log_manager import LogManager

logger = LogManager.get_logger(__name__)


class RosManager:
    def __init__(self, action_mode=True, swarm_manager=None, loop=None):

        self.action_mode = action_mode
        self.scene_monitor_node = None
        self.swarm_node = None
        self.plan_execution_action_server = None
        self.skill_client_action_server = None

        self.loop = loop
        self.stop_event = threading.Event()
        self.thread = None
        self.swarm_manager = swarm_manager

        self.executor = MultiThreadedExecutor()
        self.build_nodes()

    def build_nodes(self) -> None:
        """构建所有ROS节点"""
        # qos = QoSProfile(
        #     reliability=ReliabilityPolicy.RELIABLE,
        #     history=HistoryPolicy.KEEP_LAST,
        #     depth=50,
        # )
        self.scene_monitor_node = SceneMonitorNode()
        self.swarm_node = SwarmNode()

        self.plan_execution_action_server = PlanExecutionServer(
            loop=self.loop,
            swarm_manager=self.swarm_manager
        )
        self.skill_client_action_server = (
            self.plan_execution_action_server.skill_client_action_server
        )

        logger.info("ROS nodes built successfully.")

    def start(self):
        """在后台线程中启动ROS节点"""
        if not self.plan_execution_action_server:
            logger.warning("ROS nodes not built. Cannot start.")
            return

        self.thread = threading.Thread(target=self._spin_in_background, daemon=True)
        self.thread.start()
        logger.info("ROS manager thread started.")

    def _spin_in_background(self):
        """后台运行ROS节点的实际工作函数"""
        nodes = [
            self.plan_execution_action_server,
            self.skill_client_action_server,
            self.scene_monitor_node,
            self.swarm_node,
        ]

        try:
            for n in nodes:
                if n:
                    self.executor.add_node(n)
            while not self.stop_event.is_set():
                self.executor.spin_once(timeout_sec=0.05)
        finally:
            for n in nodes:
                if n and self.executor:
                    self.executor.remove_node(n)
                    n.destroy_node()
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
