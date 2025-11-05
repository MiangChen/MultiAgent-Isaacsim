# =============================================================================
# Node Path Planner OMPL Module - OMPL-Based Path Planning
# =============================================================================
#
# This module provides ROS2 node implementation for path planning using
# the Open Motion Planning Library (OMPL) with 3D grid map integration.
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
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2, PointField
from diagnostic_msgs.msg import DiagnosticArray
from visualization_msgs.msg import Marker


class NodePlannerOmpl(Node):
    def __init__(self, namespace: str):
        super().__init__(node_name="node_planner_ompl", namespace=namespace)

        self.grid_map = None
        self.cell_size = 0.1

        self.map_info = None
        self.dimension = 3
        self.shape = None
        self.min_bound = None
        self.max_bound = None
        self.occupied_value = None
        self.free_value = None
        self.unknown_value = None

        self.space = None
        self.si = None

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        subscriber_map_info = message_filters.Subscriber(
            self,
            DiagnosticArray,
            "/map_info",
            # self.callback_info,
            qos_profile=qos_profile,
        )
        subscriber_point_cloud = message_filters.Subscriber(
            self,
            PointCloud2,
            "/map_point_cloud",
            qos_profile=qos_profile,
        )
        self.time_synchronizer = message_filters.ApproximateTimeSynchronizer(
            [subscriber_map_info, subscriber_point_cloud],
            queue_size=10,
            slop=0.5,  # 对于一次性消息，时间差可以放宽一些
        )
        self.time_synchronizer.registerCallback(self.callback)

        self.publisher_path = self.create_publisher(Path, "planned_path", qos_profile)

        callback_group = ReentrantCallbackGroup()
        self.server_action = ActionServer(
            self,
            ComputePathToPose,
            "action_compute_path_to_pose",
            execute_callback=self.callback_compute_path_to_pose,
            callback_group=callback_group,
        )

    def callback(self, msg_map_info: DiagnosticArray, msg_point_cloud: PointCloud2):
        self.callback_map_info(msg_map_info=msg_map_info)
        self.callback_point_cloud(msg_point_cloud=msg_point_cloud)

    def callback_map_info(self, msg_map_info: DiagnosticArray):
        """
        store information in self.map_info
        """
        temp_map_info = {}
        if not msg_map_info.status:
            return

        key_values = msg_map_info.status[0].values
        for kv in key_values:
            temp_map_info[kv.key] = json.loads(kv.value)

        self.map_info = temp_map_info
        self.dimension = self.map_info["dimension"]
        self.shape = self.map_info["shape"]
        self.cell_size = self.map_info["cell_size"]
        self.min_bound = self.map_info["min_bound"]
        self.max_bound = self.map_info["max_bound"]
        self.occupied_value = self.map_info["occupied_value"]
        self.free_value = self.map_info["free_value"]
        self.unknown_value = self.map_info["unknown_value"]

    def callback_point_cloud(self, msg_point_cloud: PointCloud2):

        self.grid_map = np.full(self.shape, self.free_value, dtype=np.int8)
        points = pc2.read_points_numpy(msg_point_cloud, field_names=("x", "y", "z"))
        if points.size > 0:
            indices = ((points - self.min_bound) / self.cell_size).astype(int)
            valid_mask = np.all((indices >= 0) & (indices < self.shape), axis=1)
            valid_indices = indices[valid_mask]
            self.grid_map[
                valid_indices[:, 0], valid_indices[:, 1], valid_indices[:, 2]
            ] = self.occupied_value

        bounds = ob.RealVectorBounds(3)
        bounds.setLow(0, self.min_bound[0])
        bounds.setHigh(0, self.max_bound[0])
        bounds.setLow(1, self.min_bound[1])
        bounds.setHigh(1, self.max_bound[1])
        bounds.setLow(2, self.min_bound[2])
        bounds.setHigh(2, self.max_bound[2])

        if self.dimension == 2:
            self.space = ob.SE2StateSpace()
        elif self.dimension == 3:
            self.space = ob.SE3StateSpace()
        else:
            raise False

        self.space.setBounds(bounds)

        self.si = ob.SpaceInformation(self.space)
        self.si.setStateValidityChecker(
            ob.StateValidityCheckerFn(self.is_valid_state_rigid_body)
        )
        self.si.setup()

        return True

    def callback_compute_path_to_pose(self, goal_handle):
        start_time = time.time()
        self.get_logger().info("Executing navigation goal...")

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            return ComputePathToPose.Result()

        if self.si is None:
            self.get_logger().error("Planner is not ready. Missing map or OMPL setup.")
            goal_handle.abort()
            return ComputePathToPose.Result()

        start_pose = goal_handle.request.start.pose
        start_position = [
            start_pose.position.x,
            start_pose.position.y,
            start_pose.position.z,
        ]
        start_quat = [
            start_pose.orientation.x,
            start_pose.orientation.y,
            start_pose.orientation.z,
            start_pose.orientation.w,
        ]

        goal_pose = goal_handle.request.goal.pose
        goal_position = [
            goal_pose.position.x,
            goal_pose.position.y,
            goal_pose.position.z,
        ]
        goal_quat = [
            goal_pose.orientation.x,
            goal_pose.orientation.y,
            goal_pose.orientation.z,
            goal_pose.orientation.w,
        ]

        path = self.compute_path(
            start_position, goal_position, start_quat, goal_quat, goal_handle
        )

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            return ComputePathToPose.Result()

        result = ComputePathToPose.Result()

        def convert_ompl_path_to_ros_msg(path: list, frame_id: str) -> Path:

            path_msg = Path()
            path_msg.header.stamp = self.get_clock().now().to_msg()
            path_msg.header.frame_id = frame_id

            for point in path:
                pose = PoseStamped()
                pose.header.stamp = path_msg.header.stamp
                pose.header.frame_id = frame_id

                pos = point[0]
                quat = point[1]

                pose.pose.position.x = float(pos[0])
                pose.pose.position.y = float(pos[1])
                pose.pose.position.z = float(pos[2])
                pose.pose.orientation.x = float(quat[0])
                pose.pose.orientation.y = float(quat[1])
                pose.pose.orientation.z = float(quat[2])
                pose.pose.orientation.w = float(quat[3])

                path_msg.poses.append(pose)

            return path_msg

        if path:
            duration = time.time() - start_time
            self.get_logger().info(
                f"Path found with {len(path)} waypoints in {duration:.2f} seconds."
            )
            msg_path = convert_ompl_path_to_ros_msg(path, frame_id="map")
            result.path = msg_path
            goal_handle.succeed()
        else:
            goal_handle.abort()

        return result

    def compute_path(
        self,
        start_pos: list,
        goal_pos: list,
        start_quat: list = [0.0, 0.0, 0.0, 1.0],
        goal_quat: list = [0.0, 0.0, 0.0, 1.0],
        goal_handle=None,
    ) -> list:

        pdef = ob.ProblemDefinition(self.si)

        start_state = ob.State(self.space)  # type(start_state)=ompl.base._base.State
        # start_state = self.space
        s3 = start_state()
        s3.setXYZ(start_pos[0], start_pos[1], start_pos[2])
        s3.rotation().x = float(start_quat[0])
        s3.rotation().y = float(start_quat[1])
        s3.rotation().z = float(start_quat[2])
        s3.rotation().w = float(start_quat[3])

        goal_state = ob.State(self.space)
        g3 = goal_state()
        g3.setXYZ(goal_pos[0], goal_pos[1], goal_pos[2])
        g3.rotation().x = float(goal_quat[0])
        g3.rotation().y = float(goal_quat[1])
        g3.rotation().z = float(goal_quat[2])
        g3.rotation().w = float(goal_quat[3])

        def normalize_quat(st):
            """Normalize quaternion inside OMPL SE3 state."""
            r = st().rotation()
            n = (r.x**2 + r.y**2 + r.z**2 + r.w**2) ** 0.5
            if n < 1e-12:
                r.x = r.y = r.z = 0.0
                r.w = 1.0
            else:
                r.x, r.y, r.z, r.w = r.x / n, r.y / n, r.z / n, r.w / n

        normalize_quat(start_state)
        normalize_quat(goal_state)

        pdef.setStartAndGoalStates(start_state, goal_state)

        planner = og.PRM(self.si)
        planner.setProblemDefinition(pdef)
        planner.setup()

        if goal_handle and goal_handle.is_cancel_requested:
            return []

        solved = planner.solve(5)

        if goal_handle and goal_handle.is_cancel_requested:
            return []

        path = []
        if solved:
            solution_path = pdef.getSolutionPath()
            solution_path = self.smooth_path_ompl(solution_path)
            solution_path.interpolate(5)

            states = solution_path.getStates()
            for state in states:
                if self.dimension == 2:
                    # todo
                    pass
                elif self.dimension == 3:
                    path.append(
                        [
                            [state.getX(), state.getY(), state.getZ()],
                            [
                                state.rotation().x,
                                state.rotation().y,
                                state.rotation().z,
                                state.rotation().w,
                            ],
                        ]
                    )

        self.publish_point(start_pos)
        self.publish_point(goal_pos)
        if path:
            self.publish_path(path)  # 2D/3D都发布路径

        return path

    def smooth_path_ompl(self, path: og.PathGeometric) -> og.PathGeometric:
        if not path or path.getStateCount() == 0:
            return path
        simplifier = og.PathSimplifier(self.si)
        simplifier.simplifyMax(path)
        simplifier.smoothBSpline(path)
        return path

    def is_valid_state(self, state):
        if self.dimension == 2:
            x, y = state[0], state[1]
            grid_x = int((x - self.min_bound[0]) / self.cell_size)
            grid_y = int((y - self.min_bound[1]) / self.cell_size)

            if (
                0 <= grid_x < self.grid_map.shape[0]
                and 0 <= grid_y < self.grid_map.shape[1]
            ):
                return self.grid_map[grid_x, grid_y] == 0

        elif self.dimension == 3:
            x, y, z = state[0][0], state[0][1], state[0][2]
            grid_x = int((x - self.min_bound[0]) / self.cell_size)
            grid_y = int((y - self.min_bound[1]) / self.cell_size)
            grid_z = int((z - self.min_bound[2]) / self.cell_size)

            if (
                0 <= grid_x < self.grid_map.shape[0]
                and 0 <= grid_y < self.grid_map.shape[1]
                and 0 <= grid_z < self.grid_map.shape[2]
            ):
                return self.grid_map[grid_x, grid_y, grid_z] == 0
        return False

    def is_valid_state_rigid_body(self, state):
        # todo: check collision of a rigid body with 体积
        # type(state)=ompl.base._base.SE3StateSpace.SE3StateInternal
        return self.is_valid_state(state)

    def publish_path(self, path_points: list):
        if not path_points:
            return

        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"

        for point in path_points:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = point[0][0]
            pose.pose.position.y = point[0][1]
            pose.pose.position.z = point[0][2]
            pose.pose.orientation.x = point[1][0]
            pose.pose.orientation.y = point[1][1]
            pose.pose.orientation.z = point[1][2]
            pose.pose.orientation.w = point[1][3]

            path_msg.poses.append(pose)

        self.publisher_path.publish(path_msg)

    def publish_point(
        self, pos: list, color: list = [0.0, 1.0, 0.0, 1.0], marker_id: int = None
    ):
        if not hasattr(self, "publisher_point"):
            self.publisher_point = self.create_publisher(Marker, "point_marker", 10)
            self.waypoint_counter = 0

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        # 自动分配 marker_id
        if marker_id is None:
            marker_id = self.waypoint_counter
            self.waypoint_counter += 1
        marker.id = marker_id
        marker.pose.position.x = float(pos[0])
        marker.pose.position.y = float(pos[1])
        marker.pose.position.z = float(pos[2]) if len(pos) > 2 else 0.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.2
        marker.color.r = float(color[0])
        marker.color.g = float(color[1])
        marker.color.b = float(color[2])
        marker.color.a = float(color[3])

        self.publisher_point.publish(marker)
