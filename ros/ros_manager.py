import threading

import rclpy
from rclpy.executors import MultiThreadedExecutor

from ros.node_action_server_plan_execution import NodeActionServerPlanExecution
from ros.node_scene_monitor import NodeSceneMonitor
from ros.node_server_planner_nav2 import NodeServerNav2Planner
from navigation.node_path_planner_ompl import NodePlannerOmpl
from navigation.node_trajectory_generator import NodeTrajectoryGenerator
from navigation.node_controller_mpc import NodeMpcController
from navigation.simple_robot import SimpleRobotSimulatorNode
from log.log_manager import LogManager

logger = LogManager.get_logger(__name__)


class RosManager:
    def __init__(self, loop=None, config: dict = None):
        self.config = config if config is not None else {}

        self.node = {}

        self.executor = MultiThreadedExecutor()
        self.loop = loop
        self.stop_event = threading.Event()
        self.thread = None

        self.build_nodes()

    def build_nodes(self) -> None:
        """构建ROS节点"""
        config_node_enable = self.config.get("node_enable", {})
        if config_node_enable.get("node_scene_monitor", False):
            self.node["node_scene_monitor"] = NodeSceneMonitor()
        if config_node_enable.get("node_action_server_plan_execution", False):
            node = NodeActionServerPlanExecution(loop=self.loop)
            self.node["node_action_server_plan_execution"] = node
            self.node["node_action_client_skill"] = node.action_client_skill
        if config_node_enable.get("node_server_nav2_planner", False):
            self.node["node_server_nav2_planner"] = NodeServerNav2Planner()
        if config_node_enable.get("navigation", False):
            self.node["node_planner_ompl"] = NodePlannerOmpl()
            self.node["node_trajectory_generator"] = NodeTrajectoryGenerator()
            self.node["node_controller_mpc"] = NodeMpcController()
            self.node["robot"] = SimpleRobotSimulatorNode()

        logger.info("ROS nodes built successfully.")

    def start(self):
        """在后台线程中启动ROS节点"""
        if not self.node:
            logger.warning("No ROS nodes was built. Cannot start.")
            return

        self.thread = threading.Thread(target=self._spin_in_background, daemon=True)
        self.thread.start()
        logger.info("ROS manager thread started.")

    def _spin_in_background(self):
        """后台运行ROS节点的实际工作函数"""

        try:
            for n in self.node.values():
                if n:
                    self.executor.add_node(n)
            while not self.stop_event.is_set():
                self.executor.spin_once(timeout_sec=0.05)
        finally:
            for n in self.node.values():
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
