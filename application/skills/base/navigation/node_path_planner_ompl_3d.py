# =============================================================================
# Node Path Planner OMPL 3D Module - OMPL-Based 3D Path Planning
# =============================================================================
#
# This module provides ROS2 node implementation for 3D path planning using
# the Open Motion Planning Library (OMPL). Uses full 3D grid map for planning,
# suitable for aerial robots (drones).
#
# Uses std_srvs/Trigger to request map updates, receives map via topics.
#
# =============================================================================

# Standard library imports
import json
import time

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
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from diagnostic_msgs.msg import DiagnosticArray
from std_srvs.srv import Trigger


class NodePlannerOmpl3D(Node):
    def __init__(self, namespace: str):
        super().__init__(node_name="node_planner_ompl_3d", namespace=namespace)

        self.grid_map = None
        self.cell_size = 0.1

        self.map_info = None
        self.shape = None
        self.min_bound = None
        self.max_bound = None
        self.occupied_value = None
        self.free_value = None

        self.space = None
        self.si = None
        self.map_ready = False

        # Callback groups for parallel execution
        self._action_cb_group = ReentrantCallbackGroup()
        self._service_cb_group = MutuallyExclusiveCallbackGroup()

        # Service client (in separate callback group to avoid deadlock)
        self.client_update_map = self.create_client(
            Trigger, "/update_grid_map", callback_group=self._service_cb_group
        )

        # Subscribe to map topics
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

        self.publisher_path = self.create_publisher(Path, "planned_path", 10)

        self.server_action = ActionServer(
            self,
            ComputePathToPose,
            "action_compute_path_to_pose_3d",
            execute_callback=self.callback_compute_path_to_pose,
            callback_group=self._action_cb_group,
        )


    def callback_map_received(self, msg_map_info: DiagnosticArray, msg_point_cloud: PointCloud2):
        self._process_map_info(msg_map_info)
        self._build_3d_map_from_pointcloud(msg_point_cloud)
        self._setup_ompl_3d()
        self.map_ready = True
        self.get_logger().info("3D map received and processed")

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
        self.shape = tuple(self.map_info["shape"])

    def request_map_update(self) -> bool:
        if not self.client_update_map.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Map update service not available")
            return False

        self.map_ready = False
        request = Trigger.Request()
        future = self.client_update_map.call_async(request)

        # Wait for service response (works because service client is in different callback group)
        start_time = time.time()
        while not future.done():
            if time.time() - start_time > 10.0:
                self.get_logger().error("Map update service timeout")
                return False
            time.sleep(0.05)

        response = future.result()
        if not response.success:
            self.get_logger().error(f"Map update failed: {response.message}")
            return False

        # Wait for map data via topic callback
        start_time = time.time()
        while not self.map_ready:
            if time.time() - start_time > 5.0:
                self.get_logger().error("Timeout waiting for map data")
                return False
            time.sleep(0.05)

        return True

    def _build_3d_map_from_pointcloud(self, point_cloud: PointCloud2):
        """Build 3D occupancy grid from point cloud."""
        if self.shape is None:
            return

        self.grid_map = np.full(self.shape, self.free_value, dtype=np.int8)
        points = pc2.read_points_numpy(point_cloud, field_names=("x", "y", "z"))

        if points.size == 0:
            return

        indices = ((points - self.min_bound) / self.cell_size).astype(int)
        valid_mask = np.all((indices >= 0) & (indices < self.shape), axis=1)
        valid_indices = indices[valid_mask]

        self.grid_map[valid_indices[:, 0], valid_indices[:, 1], valid_indices[:, 2]] = self.occupied_value

    def _setup_ompl_3d(self):
        """Setup OMPL for 3D SE3 planning."""
        if self.min_bound is None or self.max_bound is None:
            return

        bounds = ob.RealVectorBounds(3)
        bounds.setLow(0, float(self.min_bound[0]))
        bounds.setHigh(0, float(self.max_bound[0]))
        bounds.setLow(1, float(self.min_bound[1]))
        bounds.setHigh(1, float(self.max_bound[1]))
        bounds.setLow(2, float(self.min_bound[2]))
        bounds.setHigh(2, float(self.max_bound[2]))

        self.space = ob.SE3StateSpace()
        self.space.setBounds(bounds)

        self.si = ob.SpaceInformation(self.space)
        self.si.setStateValidityChecker(ob.StateValidityCheckerFn(self.is_valid_state_3d))
        self.si.setup()

    def is_valid_state_3d(self, state):
        """Check if 3D state is valid (not occupied)."""
        if self.grid_map is None:
            return False
        x, y, z = state.getX(), state.getY(), state.getZ()
        grid_x = int((x - self.min_bound[0]) / self.cell_size)
        grid_y = int((y - self.min_bound[1]) / self.cell_size)
        grid_z = int((z - self.min_bound[2]) / self.cell_size)

        if (0 <= grid_x < self.shape[0] and 0 <= grid_y < self.shape[1] and 0 <= grid_z < self.shape[2]):
            return self.grid_map[grid_x, grid_y, grid_z] == self.free_value
        return False


    def callback_compute_path_to_pose(self, goal_handle):
        """Action callback for 3D path planning."""
        start_time = time.time()
        self.get_logger().info("Executing 3D navigation goal...")

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            return ComputePathToPose.Result()

        if not self.request_map_update():
            self.get_logger().error("Failed to get map for planning")
            goal_handle.abort()
            return ComputePathToPose.Result()

        if self.si is None:
            self.get_logger().error("Planner is not ready")
            goal_handle.abort()
            return ComputePathToPose.Result()

        start_pose = goal_handle.request.start.pose
        start_position = [start_pose.position.x, start_pose.position.y, start_pose.position.z]
        start_quat = [start_pose.orientation.x, start_pose.orientation.y,
                      start_pose.orientation.z, start_pose.orientation.w]

        goal_pose = goal_handle.request.goal.pose
        goal_position = [goal_pose.position.x, goal_pose.position.y, goal_pose.position.z]
        goal_quat = [goal_pose.orientation.x, goal_pose.orientation.y,
                     goal_pose.orientation.z, goal_pose.orientation.w]

        path = self.compute_path_3d(start_position, goal_position, start_quat, goal_quat, goal_handle)

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            return ComputePathToPose.Result()

        result = ComputePathToPose.Result()
        if path:
            duration = time.time() - start_time
            self.get_logger().info(f"3D Path found with {len(path)} waypoints in {duration:.2f}s")
            result.path = self._convert_path_to_ros_msg(path)
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    def compute_path_3d(self, start_pos, goal_pos, start_quat=None, goal_quat=None, goal_handle=None):
        """Compute 3D path using OMPL SE3 planner."""
        start_quat = start_quat or [0.0, 0.0, 0.0, 1.0]
        goal_quat = goal_quat or [0.0, 0.0, 0.0, 1.0]

        pdef = ob.ProblemDefinition(self.si)

        start_state = ob.State(self.space)
        s3 = start_state()
        s3.setXYZ(start_pos[0], start_pos[1], start_pos[2])
        s3.rotation().x, s3.rotation().y = float(start_quat[0]), float(start_quat[1])
        s3.rotation().z, s3.rotation().w = float(start_quat[2]), float(start_quat[3])

        goal_state = ob.State(self.space)
        g3 = goal_state()
        g3.setXYZ(goal_pos[0], goal_pos[1], goal_pos[2])
        g3.rotation().x, g3.rotation().y = float(goal_quat[0]), float(goal_quat[1])
        g3.rotation().z, g3.rotation().w = float(goal_quat[2]), float(goal_quat[3])

        self._normalize_quat(start_state)
        self._normalize_quat(goal_state)

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
                path.append([
                    [state.getX(), state.getY(), state.getZ()],
                    [state.rotation().x, state.rotation().y, state.rotation().z, state.rotation().w]
                ])

        if path:
            self.publish_path(path)
        return path

    def _normalize_quat(self, state):
        r = state().rotation()
        n = (r.x**2 + r.y**2 + r.z**2 + r.w**2) ** 0.5
        if n < 1e-12:
            r.x = r.y = r.z = 0.0
            r.w = 1.0
        else:
            r.x, r.y, r.z, r.w = r.x / n, r.y / n, r.z / n, r.w / n

    def _smooth_path(self, path):
        if not path or path.getStateCount() == 0:
            return path
        simplifier = og.PathSimplifier(self.si)
        simplifier.simplifyMax(path)
        simplifier.smoothBSpline(path)
        return path

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
