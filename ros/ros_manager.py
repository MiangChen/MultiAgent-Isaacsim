import asyncio
import threading

import rclpy
from rclpy.executors import MultiThreadedExecutor

# rclpy.init(args=None)

from gsi2isaacsim.gsi_msgs_helper import Plan
from ros.node import PlanNode, SceneMonitorNode, SwarmNode, PlanExecutionServer, SkillServerNode
from log.log_manager import LogManager

logger = LogManager.get_logger(__name__)


def plan_cb_wrapper(msg):
    from containers import get_container
    skill_manager = get_container().skill_manager()
    skill_manager._plan_cb(msg)


class RosManager:
    def __init__(self, action_mode=True):

        self.plan_receiver_node = None
        self.scene_monitor_node = None
        self.swarm_node = None
        self.plan_execution_server = None
        self.skill_server_node = None
        self.executor = None
        self.stop_event = threading.Event()
        self.thread = None
        self.action_mode = action_mode

        self.build_nodes()

    def build_nodes(self):
        """构建所有ROS节点"""
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
        )

        self.scene_monitor_node = SceneMonitorNode()

        # if self.action_mode:
        loop = asyncio.get_event_loop()
        self.plan_execution_server = PlanExecutionServer(loop=loop)
        self.skill_server_node = SkillServerNode()
        self.swarm_node = SwarmNode()
        # else:
        #     self.plan_receiver_node = PlanNode()
        #     self.plan_receiver_node.create_subscription(Plan, '/Plan', plan_cb_wrapper, qos)
        #     self.swarm_node = SwarmNode()

        logger.info("ROS nodes built successfully.")

    def start(self):
        """在后台线程中启动ROS节点"""
        # if self.action_mode:
        if not self.plan_execution_server or not self.skill_server_node:
            logger.warning("ROS nodes not built. Cannot start.")
            return
        # else:
        #     if not self.plan_receiver_node:
        #         logger.warning("ROS nodes not built. Cannot start.")
        #         return

        self.thread = threading.Thread(
            target=self._spin_in_background, daemon=True
        )
        self.thread.start()
        logger.info("ROS manager thread started.")

    def _spin_in_background(self):
        """后台运行ROS节点的实际工作函数"""
        self.executor = MultiThreadedExecutor()

        nodes = [self.plan_execution_server, self.skill_server_node, self.plan_receiver_node, self.scene_monitor_node, self.swarm_node]
        # if self.action_mode:
        #     nodes = [self.plan_receiver_node, self.scene_monitor_node, self.swarm_node]
        # else:
        #     nodes = [self.plan_execution_server, self.skill_server_node, self.scene_monitor_node, self.swarm_node]

        for n in nodes:
            if n:
                self.executor.add_node(n)

        try:
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
