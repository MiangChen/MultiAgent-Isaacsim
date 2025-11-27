# =============================================================================
# ROS Manager Module - ROS2 Node and Executor Management
# =============================================================================

# Standard library imports
import threading

# Local project imports
from config.config_manager import config_manager
from log.log_manager import LogManager
from map.map_grid_map import GridMap
from ros.node_scene_monitor import NodeSceneMonitor

# ROS2 imports
import rclpy
from rclpy.executors import MultiThreadedExecutor

logger = LogManager.get_logger(__name__)


class RosManagerIsaac:
    def __init__(
        self,
        loop=None,
        config: dict = None,
    ):
        self.config = config if config is not None else {}
        self.executor = MultiThreadedExecutor()
        self.loop = loop
        self.node = {}
        self.grid_map = None
        self.stop_event = threading.Event()
        self.thread = None

        self.build_nodes()

    def build_nodes(self) -> None:
        config_node_enable = self.config.get("node_enable", {})

        if config_node_enable.get("node_scene_monitor", False):
            self.node["node_scene_monitor"] = NodeSceneMonitor()

        if config_node_enable.get("node_grid_map", True):
            map_cfg = config_manager.get("map")
            self.grid_map = GridMap(
                cell_size=map_cfg["cell_size"],
                start_point=map_cfg["start_point"],
                min_bound=map_cfg["min_bound"],
                max_bound=map_cfg["max_bound"],
                occupied_value=map_cfg["occupied_cell"],
                free_value=map_cfg["free_cell"],
                unknown_value=map_cfg["unknown_cell"],
            )
            self.node["node_grid_map"] = self.grid_map

        logger.info("ROS Node for Isaac built successfully.")

    def start(self):
        """在后台线程中启动ROS节点"""
        if not self.node:
            logger.warning("No ROS Node for Isaac was built. Cannot start.")
            return

        self.thread = threading.Thread(target=self._spin_in_background, daemon=True)
        self.thread.start()
        logger.info("ROS manager Isaac thread started.")

    def _spin_in_background(self):
        """后台运行ROS节点的实际工作函数"""

        try:
            for n in self.node.values():
                if n:
                    self.executor.add_node(n)
                    logger.debug(
                        f"Added node {n.get_name()} to ROS manager Isaac executor"
                    )

            while not self.stop_event.is_set():
                self.executor.spin_once(timeout_sec=0.05)
        finally:
            for n in self.node.values():
                if n and self.executor:
                    self.executor.remove_node(n)
                    n.destroy_node()
            logger.info(
                "ROS manager Isaac executor stopped (rclpy context preserved for robots)."
            )

    def stop(self):
        """停止ROS线程"""
        if self.thread and self.thread.is_alive():
            self.stop_event.set()
            self.thread.join(timeout=3.0)
            if self.thread.is_alive():
                logger.warning("ROS thread did not stop gracefully.")
        logger.info("ROS manager Isaac stopped.")

    def initialize_grid_map(self):
        """初始化GridMap（需要在world reset后调用）"""
        if self.grid_map:
            self.grid_map.initialize()
            logger.info("GridMap initialized.")

    def generate_grid_map(self, ground_height: float = 0.0, ground_tolerance: float = 0.2):
        """生成并发布地图"""
        if self.grid_map:
            self.grid_map.generate(ground_height, ground_tolerance)
            logger.info("GridMap generated and published.")

    def get_grid_map(self):
        """获取GridMap节点"""
        return self.grid_map
