import numpy as np
from rclpy.node import Node
from ompl import base as ob
from ompl import geometric as og

from nav_msgs.msg import Path
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker
import struct


class NodeServerPlannerOmpl(Node):
    def __init__(self):
        super().__init__("node_server_planner_ompl")

        self.grid_map = None
        self.dimensions = 2
        self.cell_size = 0.1
        self.origin = [0.0, 0.0]
        self.bounds = [0.0, 0.0, 0.0, 0.0]
        self.space = None
        self.si = None
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.publisher_map = self.create_publisher(PointCloud2, '/map_pointcloud', qos_profile)
        self.publisher_path = self.create_publisher(Path, '/planned_path', qos_profile)

    def set_map(self, grid_map: np.ndarray, cell_size: float, min_bounds: list, max_bounds: list) -> bool:

        self.grid_map = grid_map
        self.cell_size = cell_size
        self.origin = min_bounds
        # 自动检测维度
        if len(grid_map.shape) == 2:
            # 2D 地图
            self.dimensions = 2
        elif len(grid_map.shape) == 3:
            # 3D 地图
            self.dimensions = 3
        else:
            return False

        self.bounds = min_bounds[:self.dimensions] + max_bounds[:self.dimensions]
        # 创建对应维度的状态空间
        self.space = ob.RealVectorStateSpace(self.dimensions)
        bounds_ompl = ob.RealVectorBounds(self.dimensions)

        for i in range(self.dimensions):
            bounds_ompl.setLow(i, self.bounds[i])
            bounds_ompl.setHigh(i, self.bounds[i + self.dimensions])

        self.space.setBounds(bounds_ompl)

        self.si = ob.SpaceInformation(self.space)
        self.si.setStateValidityChecker(ob.StateValidityCheckerFn(self.is_valid_state))
        self.si.setup()

        # 发布地图点云（2D/3D通用）
        if self.publisher_map:
            self.publish_map(grid_map, cell_size, min_bounds)

        return True

    def compute_path(self, start_pos: list, goal_pos: list) -> list:
        if self.si is None:
            return None

        pdef = ob.ProblemDefinition(self.si)

        start_state = ob.State(self.space)
        goal_state = ob.State(self.space)

        # 根据维度设置状态
        for i in range(self.dimensions):
            start_state[i] = float(start_pos[i])
            goal_state[i] = float(goal_pos[i])

        pdef.setStartAndGoalStates(start_state, goal_state)

        planner = og.RRTstar(self.si)
        planner.setProblemDefinition(pdef)
        planner.setup()

        solved = planner.solve(5.0)
        path = []
        if solved:
            solution_path = pdef.getSolutionPath()
            solution_path.interpolate(100)

            states = solution_path.getStates()
            for state in states:
                if self.dimensions == 2:
                    x, y = float(state[0]), float(state[1])
                    path.append([x, y, 0.0])  # 2D时z=0
                elif self.dimensions == 3:
                    x, y, z = float(state[0]), float(state[1]), float(state[2])
                    path.append([x, y, z])

        self.publish_waypoint(start_pos)
        self.publish_waypoint(goal_pos)
        if path:
            self.publish_path(path)  # 2D/3D都发布路径

        return path

    def is_valid_state(self, state):
        if self.dimensions == 2:
            x, y = state[0], state[1]
            grid_x = int((x - self.origin[0]) / self.cell_size)
            grid_y = int((y - self.origin[1]) / self.cell_size)

            if (0 <= grid_x < self.grid_map.shape[0] and
                    0 <= grid_y < self.grid_map.shape[1]):
                return self.grid_map[grid_x, grid_y] == 0

        elif self.dimensions == 3:
            x, y, z = state[0], state[1], state[2]
            grid_x = int((x - self.origin[0]) / self.cell_size)
            grid_y = int((y - self.origin[1]) / self.cell_size)
            grid_z = int((z - self.origin[2]) / self.cell_size)

            if (0 <= grid_x < self.grid_map.shape[0] and
                    0 <= grid_y < self.grid_map.shape[1] and
                    0 <= grid_z < self.grid_map.shape[2]):
                return self.grid_map[grid_x, grid_y, grid_z] == 0

        return False

    def publish_map(self, grid_map: np.ndarray, cell_size: float, min_bounds: list) -> bool:
        points = []

        if self.dimensions == 2:
            # 2D 地图：z=0
            for x in range(grid_map.shape[0]):
                for y in range(grid_map.shape[1]):
                    if grid_map[x, y] == 100:  # 占用格子
                        world_x = min_bounds[0] + x * cell_size
                        world_y = min_bounds[1] + y * cell_size
                        points.append([world_x, world_y, 0.0])

        elif self.dimensions == 3:
            # 3D 地图：真实 z 值
            for x in range(grid_map.shape[0]):
                for y in range(grid_map.shape[1]):
                    for z in range(grid_map.shape[2]):
                        if grid_map[x, y, z] == 100:  # 占用格子
                            world_x = min_bounds[0] + x * cell_size
                            world_y = min_bounds[1] + y * cell_size
                            world_z = min_bounds[2] + z * cell_size
                            points.append([world_x, world_y, world_z])

        if points:
            pc2_msg = self.create_pointcloud2(points)
            self.publisher_map.publish(pc2_msg)

        return True

    def create_pointcloud2(self, points):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        cloud_data = []
        for point in points:
            # 根据 z 轴高度设置强度
            intensity = point[2]  # 直接使用 z 坐标作为强度
            cloud_data.extend(struct.pack('ffff', point[0], point[1], point[2], intensity))

        pc2_msg = PointCloud2()
        pc2_msg.header = header
        pc2_msg.height = 1
        pc2_msg.width = len(points)
        pc2_msg.fields = fields
        pc2_msg.is_bigendian = False
        pc2_msg.point_step = 16  # 4 * 4 bytes
        pc2_msg.row_step = pc2_msg.point_step * len(points)
        pc2_msg.data = bytes(cloud_data)
        pc2_msg.is_dense = True

        return pc2_msg

    def publish_path(self, path_points: list):
        if not path_points:
            return

        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"

        for point in path_points:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = float(point[0])
            pose.pose.position.y = float(point[1])
            pose.pose.position.z = float(point[2])  # 支持3D路径
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.publisher_path.publish(path_msg)

    def publish_waypoint(self, pos: list, color: list = [0.0, 1.0, 0.0, 1.0], marker_id: int = None):
        if not hasattr(self, 'publisher_waypoint'):
            self.publisher_waypoint = self.create_publisher(Marker, '/waypoint_marker', 10)
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
        

        

        self.publisher_waypoint.publish(marker)
