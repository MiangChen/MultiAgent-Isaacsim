# =============================================================================
# Node Path Planner OMPL 2D Module - OMPL-Based 2D Path Planning
# =============================================================================
#
# This module provides ROS2 node implementation for 2D path planning using
# the Open Motion Planning Library (OMPL). It extracts the z=0 layer from
# the 3D grid map for planning, suitable for ground robots (cars, humanoids).
#
# The output path is still in 3D format (with z=ground_height) for consistency.
#
# Uses std_srvs/Trigger to request map updates, receives map via topics.
#
# =============================================================================

# Standard library imports
import json
import time
import threading

# Third-party library imports
import numpy as np
from ompl import base as ob
from ompl import geometric as og
import message_filters
import sensor_msgs_py.point_cloud2 as pc2

# ROS2 imports
from nav_msgs.msg import Path
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from diagnostic_msgs.msg import DiagnosticArray
from visualization_msgs.msg import Marker
from std_srvs.srv import Trigger


class NodePlannerOmpl2D(Node):
    def __init__(self, namespace: str):
        super().__init__(node_name="node_planner_ompl_2d", namespace=namespace)

        self.grid_map_2d = None
        self.cell_size = 0.1
        self.ground_height = 0.0

        self.map_info = None
        self.shape_2d = None
        self.min_bound = None
        self.max_bound = None
        self.occupied_value = None
        self.free_value = None

        self.space = None
        self.si = None
        self.map_ready = False
        self._map_update_event = threading.Event()

        # Service client for triggering map update
        self.client_update_map = self.create_client(Trigger, "/update_grid_map")

        # Subscribe to map topics (same as original planner)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        subscriber_map_info = message_filters.Subscriber(
            self, DiagnosticArray, "/map_info", qos_profile=qos_profile
        )
        subscriber_point_cloud = message_filters.Subscriber(
            self, PointCloud2, "/map_point_cloud", qos_profile=qos_profile
        )
        self.time_synchronizer = message_filters.ApproximateTimeSynchronizer(
            [subscriber_map_info, subscriber_point_cloud],
            queue_size=10,
            slop=0.5,
        )
        self.time_synchronizer.registerCallback(self.callback_map_received)

        # Path publisher
        self.publisher_path = self.create_publisher(Path, "planned_path_2d", 10)

        callback_group = ReentrantCallbackGroup()
        self.server_action = ActionServer(
            self,
            ComputePathToPose,
            "action_compute_path_to_pose_2d",
            execute_callback=self.callback_compute_path_to_pose,
            callback_group=callback_group,
        )

    def callback_map_received(self, msg_map_info: DiagnosticArray, msg_point_cloud: PointCloud2):
        """Callback when map data is received via topics."""
        self._process_map_info(msg_map_info)
        self._extract_2d_map_from_pointcloud(msg_point_cloud)
        self._setup_ompl_2d()
        self.map_ready = True
        self._map_update_event.set()

    def _process_map_info(self, msg_map_info: DiagnosticArray):
        """Process map info from DiagnosticArray message."""
        if not msg_map_info.status:
            return

        temp_map_info = {}
        key_values = msg_map_info.status[0].values
        for kv in key_values:
            temp_map_info[kv.key] = json.loads(kv.value)

        self.map_info = temp_map_info
        self.cell_size = self.map_info["cell_size"]
        self.min_bound = np.array(self.map_info["min_bound"], dtype=np.float32)
        self.max_bound = np.array(self.map_info["max_bound"], dtype=np.float32)
        self.occupied_value = self.map_info["occupied_value"]
        self.free_value = self.map_info["free_value"]
        shape_3d = self.map_info["shape"]
        self.shape_2d = (shape_3d[0], shape_3d[1])

    def request_map_update(self) -> bool:
        """
        Request map update via Trigger service.
        Map data will be received via topic callbacks.
        """
        if not self.client_update_map.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Map update service not available")
            return False

        self._map_update_event.clear()
        request = Trigger.Request()
        future = self.client_update_map.call_async(request)

        # Wait for service response
        start_time = time.time()
        while not future.done():
            if time.time() - start_time > 10.0:
                self.get_logger().error("Map update service timeout")
                return False
            time.sleep(0.1)

        response = future.result()
        if not response.success:
            self.get_logger().error(f"Map update failed: {response.message}")
            return False

        # Wait for map data via topics
        if not self._map_update_event.wait(timeout=5.0):
            self.get_logger().error("Timeout waiting for map data")
            return False

        self.get_logger().info("2D map updated successfully")
        return True

    def _extract_2d_map_from_pointcloud(self, point_cloud: PointCloud2):
        """Extract z=0 layer from 3D point cloud to create 2D occupancy grid."""
        if self.shape_2d is None:
            return

        self.grid_map_2d = np.full(self.shape_2d, self.free_value, dtype=np.int8)
        points = pc2.read_points_numpy(point_cloud, field_names=("x", "y", "z"))

        if points.size == 0:
            return

        # Filter points at z=0 layer (with tolerance)
        z_tolerance = self.cell_size * 1.5
        z_min = self.ground_height - z_tolerance
        z_max = self.ground_height + z_tolerance

        mask_z0 = (points[:, 2] >= z_min) & (points[:, 2] <= z_max)
        points_z0 = points[mask_z0]

        if points_z0.size == 0:
            self.get_logger().warn("No points found in z=0 layer")
            return

        indices = ((points_z0[:, :2] - self.min_bound[:2]) / self.cell_size).astype(int)
        valid_mask = np.all((indices >= 0) & (indices < self.shape_2d), axis=1)
        valid_indices = indices[valid_mask]

        self.grid_map_2d[valid_indices[:, 0], valid_indices[:, 1]] = self.occupied_value

    def _setup_ompl_2d(self):
        """Setup OMPL for 2D SE2 planning."""
        if self.min_bound is None or self.max_bound is None:
            return

        bounds = ob.RealVectorBounds(2)
        bounds.setLow(0, float(self.min_bound[0]))
        bounds.setHigh(0, float(self.max_bound[0]))
        bounds.setLow(1, float(self.min_bound[1]))
        bounds.setHigh(1, float(self.max_bound[1]))

        self.space = ob.SE2StateSpace()
        self.space.setBounds(bounds)

        self.si = ob.SpaceInformation(self.space)
        self.si.setStateValidityChecker(ob.StateValidityCheckerFn(self.is_valid_state_2d))
        self.si.setup()

    def is_valid_state_2d(self, state):
        """Check if 2D state is valid (not occupied)."""
        if self.grid_map_2d is None:
            return False
        x, y = state.getX(), state.getY()
        grid_x = int((x - self.min_bound[0]) / self.cell_size)
        grid_y = int((y - self.min_bound[1]) / self.cell_size)

        if 0 <= grid_x < self.shape_2d[0] and 0 <= grid_y < self.shape_2d[1]:
            return self.grid_map_2d[grid_x, grid_y] == self.free_value
        return False

    def callback_compute_path_to_pose(self, goal_handle):
        """Action callback for 2D path planning."""
        start_time = time.time()
        self.get_logger().info("Executing 2D navigation goal...")

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            return ComputePathToPose.Result()

        # Request latest map
        if not self.request_map_update():
            self.get_logger().error("Failed to get map for planning")
            goal_handle.abort()
            return ComputePathToPose.Result()

        if self.si is None:
            self.get_logger().error("Planner is not ready")
            goal_handle.abort()
            return ComputePathToPose.Result()

        start_pose = goal_handle.request.start.pose
        start_position = [start_pose.position.x, start_pose.position.y]
        start_yaw = self._quat_to_yaw(start_pose.orientation)

        goal_pose = goal_handle.request.goal.pose
        goal_position = [goal_pose.position.x, goal_pose.position.y]
        goal_yaw = self._quat_to_yaw(goal_pose.orientation)

        path = self.compute_path_2d(start_position, goal_position, start_yaw, goal_yaw, goal_handle)

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            return ComputePathToPose.Result()

        result = ComputePathToPose.Result()
        if path:
            duration = time.time() - start_time
            self.get_logger().info(f"2D Path found with {len(path)} waypoints in {duration:.2f}s")
            result.path = self._convert_path_to_ros_msg(path)
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    def compute_path_2d(self, start_pos, goal_pos, start_yaw=0.0, goal_yaw=0.0, goal_handle=None):
        """Compute 2D path using OMPL SE2 planner."""
        pdef = ob.ProblemDefinition(self.si)

        start_state = ob.State(self.space)
        start_state().setX(start_pos[0])
        start_state().setY(start_pos[1])
        start_state().setYaw(start_yaw)

        goal_state = ob.State(self.space)
        goal_state().setX(goal_pos[0])
        goal_state().setY(goal_pos[1])
        goal_state().setYaw(goal_yaw)

        pdef.setStartAndGoalStates(start_state, goal_state)

        planner = og.PRM(self.si)
        planner.setProblemDefinition(pdef)
        planner.setup()

        if goal_handle and goal_handle.is_cancel_requested:
            return []

        solved = planner.solve(5.0)

        if goal_handle and goal_handle.is_cancel_requested:
            return []

        path = []
        if solved:
            solution_path = pdef.getSolutionPath()
            solution_path = self._smooth_path(solution_path)
            solution_path.interpolate(5)

            for state in solution_path.getStates():
                x, y, yaw = state.getX(), state.getY(), state.getYaw()
                quat = self._yaw_to_quat(yaw)
                path.append([[x, y, self.ground_height], quat])

        if path:
            self.publish_path(path)
        return path

    def _smooth_path(self, path):
        if not path or path.getStateCount() == 0:
            return path
        simplifier = og.PathSimplifier(self.si)
        simplifier.simplifyMax(path)
        simplifier.smoothBSpline(path)
        return path

    def _quat_to_yaw(self, orientation):
        import math
        x, y, z, w = orientation.x, orientation.y, orientation.z, orientation.w
        return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

    def _yaw_to_quat(self, yaw):
        import math
        return [0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)]

    def _convert_path_to_ros_msg(self, path):
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"
        for point in path:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = point[0]
            pose.pose.orientation.x, pose.pose.orientation.y = point[1][0], point[1][1]
            pose.pose.orientation.z, pose.pose.orientation.w = point[1][2], point[1][3]
            path_msg.poses.append(pose)
        return path_msg

    def publish_path(self, path_points):
        if path_points:
            self.publisher_path.publish(self._convert_path_to_ros_msg(path_points))
