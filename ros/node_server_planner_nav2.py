import pathlib
import subprocess

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from nav2_msgs.action import ComputePathToPose
from std_msgs.msg import Header


class NodeServerNav2Planner(Node):
    def __init__(self):
        super().__init__("node_server_nav2_planner")
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.publisher_map = self.create_publisher(OccupancyGrid, "/map", qos_profile)
        self.action_client_compute_path = ActionClient(
            self, ComputePathToPose, "/compute_path_to_pose"
        )
        current_dir = pathlib.Path(__file__).parent.parent
        config_path = current_dir / "config" / "nav2_simple.yaml"
        config_path = str(config_path)
        self.planner_process = subprocess.Popen(
            [
                "ros2",
                "run",
                "nav2_planner",
                "planner_server",
                "--ros-args",
                "--params-file",
                config_path,  # "--log-level", "DEBUG"
            ]
        )
        self.lifecycle_process = subprocess.Popen(
            [
                "ros2",
                "run",
                "nav2_lifecycle_manager",
                "lifecycle_manager",
                "--ros-args",
                "--params-file",
                config_path,
            ]
        )
        self.tf_process = subprocess.Popen(
            [
                "ros2",
                "run",
                "tf2_ros",
                "static_transform_publisher",
                "0",
                "0",
                "0",
                "0",
                "0",
                "0",
                "map",
                "base_link",
            ]
        )
        import time

        time.sleep(10)

    def publish_map(
        self, grid_map_2d: np.ndarray, cell_size: float, origin: list
    ) -> bool:
        """
        origin: the min bound of the map
        """
        msg = OccupancyGrid()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        msg.info.resolution = cell_size
        msg.info.width = grid_map_2d.shape[0]
        msg.info.height = grid_map_2d.shape[1]
        msg.info.origin.position.x = origin[0]
        msg.info.origin.position.y = origin[1]
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0

        data = []
        for y in range(grid_map_2d.shape[1]):
            for x in range(grid_map_2d.shape[0]):
                # data.append(int(grid_map_2d[x, y]))
                data.append(0)

        msg.data = data
        self.publisher_map.publish(msg)
        return True

    def compute_path(self, start_pos: list, goal_pos: list) -> list:
        if not self.action_client_compute_path.wait_for_server(timeout_sec=5.0):
            return None

        goal_msg = ComputePathToPose.Goal()

        goal_msg.start = PoseStamped()
        goal_msg.start.header.frame_id = "map"
        goal_msg.start.header.stamp = self.get_clock().now().to_msg()
        goal_msg.start.pose.position.x = float(start_pos[0])
        goal_msg.start.pose.position.y = float(start_pos[1])
        goal_msg.start.pose.position.z = 0.0
        goal_msg.start.pose.orientation.w = 1.0

        goal_msg.goal = PoseStamped()
        goal_msg.goal.header.frame_id = "map"
        goal_msg.goal.header.stamp = self.get_clock().now().to_msg()
        goal_msg.goal.pose.position.x = float(goal_pos[0])
        goal_msg.goal.pose.position.y = float(goal_pos[1])
        goal_msg.goal.pose.position.z = 0.0
        goal_msg.goal.pose.orientation.w = 1.0

        goal_msg.planner_id = "GridBased"

        future = self.action_client_compute_path.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if not future.done():
            return None

        goal_handle = future.result()
        if not goal_handle.accepted:
            return None

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=10.0)

        if not result_future.done():
            return None

        result = result_future.result()
        if not result.result.path.poses:
            return None

        path = []
        for pose_stamped in result.result.path.poses:
            x = pose_stamped.pose.position.x
            y = pose_stamped.pose.position.y
            z = 0.0
            path.append([x, y, z])

        return path

    def destroy_node(self):
        if self.planner_process:
            self.planner_process.terminate()
            self.planner_process.wait()
        super().destroy_node()
