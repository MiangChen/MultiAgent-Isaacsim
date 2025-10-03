import threading

import rclpy
from rclpy.executors import MultiThreadedExecutor

from ros.node_action_server_plan_execution import NodeActionServerPlanExecution
from ros.node_scene_monitor import NodeSceneMonitor
from log.log_manager import LogManager

logger = LogManager.get_logger(__name__)


class RosManager:

    def __init__(self, loop=None):

        self.scene_monitor_node = None
        self.action_server_plan_execution = None
        self.action_server_skill_client = None

        self.loop = loop
        self.stop_event = threading.Event()
        self.thread = None

        self.executor = MultiThreadedExecutor()
        self.build_nodes()

    def build_nodes(self) -> None:
        """构建所有ROS节点"""
        self.scene_monitor_node = NodeSceneMonitor()

        self.action_server_plan_execution = NodeActionServerPlanExecution(loop=self.loop)
        self.action_server_skill_client = (
            self.action_server_plan_execution.action_server_skill_client
        )

        logger.info("ROS nodes built successfully.")

    def start(self):
        """在后台线程中启动ROS节点"""
        if not self.action_server_plan_execution:
            logger.warning("ROS nodes not built. Cannot start.")
            return

        self.thread = threading.Thread(target=self._spin_in_background, daemon=True)
        self.thread.start()
        logger.info("ROS manager thread started.")

    def _spin_in_background(self):
        """后台运行ROS节点的实际工作函数"""
        nodes = [
            self.action_server_plan_execution,
            self.action_server_skill_client,
            self.scene_monitor_node,
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
