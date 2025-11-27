# =============================================================================
# Map Grid Map Module - Grid-Based Mapping for Path Planning
# =============================================================================
#
# This module provides grid map generation for OMPL path planning, converting
# continuous 3D space into discrete grid representation with height filtering
# to remove ground obstacles.
#
# Supports both topic-based publishing and service-based on-demand map retrieval.
#
# =============================================================================

# Standard library imports
import json
import struct
from typing import List, Optional

# Third-party library imports
import numpy as np

# Local project imports
from physics_engine.omni_utils import omni
from physics_engine.isaacsim_utils import _omap

# ROS2 imports
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from std_srvs.srv import Trigger


class GridMap(Node):
    """
    Grid map generator for OMPL path planning.
    Converts continuous 3D space into discrete grid representation.
    Includes height filtering to remove ground obstacles.
    """

    def __init__(
            self,
            cell_size: float = 1.0,
            start_point: list = [0, 0, 0],
            min_bound: List[float] = [-20, -20, 0],
            max_bound: List[float] = [20, 20, 5],
            occupied_value: int = 100,
            free_value: int = 0,
            unknown_value: int = -1,
    ):
        """
        Initialize GridMap for path planning.

        Args:
            start_point: Reference point for map generation
            cell_size: Size of each grid cell in meters
            min_bound: [x_min, y_min, z_min] bounds of the map
            max_bound: [x_max, y_max, z_max] bounds of the map
            occupied_value: Value for occupied cells (obstacles)
            free_value: Value for free cells (navigable space)
            unknown_value: Value for unknown cells
        """

        super().__init__("node_map_grid")
        self.cell_size = cell_size
        self.start_point = np.array(start_point, dtype=np.float32)
        self.min_bound = np.array(min_bound, dtype=np.float32)
        self.max_bound = np.array(max_bound, dtype=np.float32)

        self.occupied_value = occupied_value
        self.free_value = free_value
        self.unknown_value = unknown_value

        # Internal state
        self.value_map: Optional[np.ndarray] = None
        self.generator: Optional[_omap.Generator] = None

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher_point_cloud = self.create_publisher(
            PointCloud2, "/map_point_cloud", qos_profile
        )

        self.publisher_info = self.create_publisher(
            DiagnosticArray, "/map_info", qos_profile
        )

        # Service for on-demand map update (uses std_srvs/Trigger)
        # Call this service to regenerate and republish map via topics
        self.srv_update_map = self.create_service(
            Trigger, "/update_grid_map", self.callback_update_map
        )

    def initialize(self) -> None:
        """
        Initialize the grid map generator. Must be called after world reset.
        """
        physx = omni.physx.acquire_physx_interface()
        stage_id = omni.usd.get_context().get_stage_id()
        self.generator = _omap.Generator(physx, stage_id)

        self.generator.update_settings(
            self.cell_size,
            float(self.occupied_value),
            float(self.free_value),
            float(self.unknown_value),
        )

        self.generator.set_transform(self.start_point, self.min_bound, self.max_bound)

    def callback_update_map(self, request, response):
        """
        Service callback for on-demand map update.
        Regenerates the map and republishes via topics.
        Uses std_srvs/Trigger - no custom srv needed.
        """
        try:
            # Regenerate map and publish to topics
            self.generate()
            self.publish_map()

            response.success = True
            response.message = "Map updated and published successfully"

        except Exception as e:
            response.success = False
            response.message = f"Failed to update map: {str(e)}"

        return response

    def _get_occupied_points(self) -> list:
        """
        Get all occupied points from the value map.
        Returns list of [x, y, z] world coordinates.
        """
        points = []
        if self.value_map is None:
            return points

        if len(self.value_map.shape) == 2:
            # 2D map
            for x in range(self.value_map.shape[0]):
                for y in range(self.value_map.shape[1]):
                    if self.value_map[x, y] == self.occupied_value:
                        world_x = self.min_bound[0] + x * self.cell_size
                        world_y = self.min_bound[1] + y * self.cell_size
                        points.append([world_x, world_y, 0.0])
        else:
            # 3D map
            for x in range(self.value_map.shape[0]):
                for y in range(self.value_map.shape[1]):
                    for z in range(self.value_map.shape[2]):
                        if self.value_map[x, y, z] == self.occupied_value:
                            world_x = self.min_bound[0] + x * self.cell_size
                            world_y = self.min_bound[1] + y * self.cell_size
                            world_z = self.min_bound[2] + z * self.cell_size
                            points.append([world_x, world_y, world_z])
        return points

    def publish_map(self) -> bool:
        """
        Publish map data via topics (for backward compatibility).
        """
        points = self._get_occupied_points()

        if points:
            stamp = self.get_clock().now().to_msg()
            msg_pc = self.create_point_cloud_msg(points, stamp)
            self.publisher_point_cloud.publish(msg_pc)
            msg_info = self.create_map_info(stamp)
            self.publisher_info.publish(msg_info)

        return True

    def create_point_cloud_msg(self, points, stamp):
        header = Header()
        header.stamp = stamp
        header.frame_id = "map"

        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(
                name="intensity", offset=12, datatype=PointField.FLOAT32, count=1
            ),
        ]

        cloud_data = []
        for point in points:
            # 根据 z 轴高度设置强度
            intensity = point[2]  # 直接使用 z 坐标作为强度
            cloud_data.extend(
                struct.pack("ffff", point[0], point[1], point[2], intensity)
            )

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

    def create_map_info(self, stamp) -> DiagnosticArray:
        """
        create aDiagnosticArray to carry 3d map information。
        """
        map_info_dict = self.get_map_info()

        key_values = []
        for key, value in map_info_dict.items():
            if not isinstance(value, str):
                value_str = json.dumps(value)
            else:
                value_str = value
            key_values.append(KeyValue(key=key, value=value_str))

        status = DiagnosticStatus(
            level=DiagnosticStatus.OK,
            name="GridMapInfo",
            message="3D Map Metadata",
            values=key_values,
        )

        msg = DiagnosticArray()
        msg.header.stamp = stamp
        msg.header.frame_id = "map"
        msg.status.append(status)

        return msg

    def generate(
            self, ground_height: float = 0.0, ground_tolerance: float = 0.2
    ) -> np.ndarray:
        """
        Generate 3D grid map with height-based ground filtering.

        Args:
            ground_height: Expected ground level height
            ground_tolerance: Height tolerance for ground detection

        Returns:
            The 3D value_map (occupancy grid) as a numpy array
        """
        if not self.generator:
            raise RuntimeError("GridMap not initialized. Call initialize() first.")

        # Generate 3D map
        self.generator.generate3d()

        # Get all occupied positions in 3D world coordinates
        occupied_positions = self.generator.get_occupied_positions()
        x_dim, y_dim, z_dim = self.generator.get_dimensions()

        # Initialize 3D map
        self.value_map = np.full((x_dim, y_dim, z_dim), self.free_value, dtype=np.uint8)

        # Populate map with all obstacles first
        if occupied_positions is not None and len(occupied_positions) > 0:
            occupied_indices = self.compute_index(occupied_positions)
            if occupied_indices is not None:
                self.value_map[tuple(occupied_indices.T)] = self.occupied_value

        # Apply height-based ground filtering
        self._remove_ground_obstacles(ground_height, ground_tolerance)

        return self.value_map

    def _remove_ground_obstacles(self, ground_height: float, tolerance: float) -> None:
        """
        Remove ground obstacles while preserving building bases.

        Args:
            ground_height: Expected ground level height
            tolerance: Height tolerance for ground detection
        """
        if self.value_map is None:
            return

        # Ensure parameters are float
        ground_height = float(ground_height)
        tolerance = float(tolerance)
        effective_tolerance = max(tolerance, self.cell_size / 2)

        x_dim, y_dim, z_dim = self.value_map.shape

        # Calculate how many z layers to check based on cell_size
        # Smaller cell_size -> check more layers
        max_ground_layers = max(1, int((effective_tolerance * 2) / self.cell_size))
        max_ground_layers = min(max_ground_layers, z_dim)

        for x in range(x_dim):
            for y in range(y_dim):
                for z in range(max_ground_layers):
                    if self.value_map[x, y, z] == self.occupied_value:
                        # Get world coordinates for this grid cell
                        min_bound = np.array(
                            self.generator.get_min_bound(), dtype=np.float32
                        )
                        world_pos = (
                                min_bound
                                + np.array([x, y, z], dtype=np.float32) * self.cell_size
                        )
                        actual_height = float(world_pos[2])

                        # Check if this cell is within ground height range
                        if abs(actual_height - ground_height) <= effective_tolerance:
                            # Check if there are obstacles above this position
                            # If the layer above (z+1) is empty, this is likely ground
                            check_z = z + 1
                            if check_z < z_dim:
                                is_empty_above = (
                                        self.value_map[x, y, check_z] == self.free_value
                                )
                            else:
                                # If we're at the top layer, consider it empty above
                                is_empty_above = True

                            # Only remove if it's empty above (indicating ground, not building base)
                            if is_empty_above:
                                self.value_map[x, y, z] = self.free_value

    def is_valid_position(self, position: List[float]) -> bool:
        """
        Check if a position is within map bounds.

        Args:
            position: [x, y, z] coordinates

        Returns:
            True if position is within bounds
        """
        return all(
            self.min_bound[i] <= position[i] <= self.max_bound[i]
            for i in range(len(position))
        )

    def is_occupied(self, position: List[float]) -> bool:
        """
        Check if a position is occupied (obstacle).

        Args:
            position: [x, y, z] coordinates

        Returns:
            True if position is occupied
        """
        if not self.is_valid_position(position):
            return True  # Out of bounds considered occupied

        indices = self.compute_index([position])
        if indices is None:
            return True

        idx = tuple(indices[0])
        return self.value_map[idx] == self.occupied_value

    def get_map_info(self) -> dict:
        """
        Get map metadata for OMPL planning.

        Returns:
            Dictionary with map information
        """
        if self.value_map is None:
            raise RuntimeError("Map not generated. Call generate() first.")

        return {
            "dimension": 3,
            "cell_size": self.cell_size,
            "min_bound": self.min_bound.tolist(),
            "max_bound": self.max_bound.tolist(),
            "occupied_value": self.occupied_value,
            "free_value": self.free_value,
            "unknown_value": self.unknown_value,
            "shape": self.value_map.shape,
        }

    def compute_index(self, positions) -> Optional[np.ndarray]:
        """
        Convert world coordinates to grid indices.

        Args:
            positions: List of [x, y, z] coordinates or single position

        Returns:
            Array of grid indices or None if invalid input
        """
        if positions is None or len(positions) == 0:
            return None

        if not self.generator:
            raise RuntimeError("GridMap not initialized.")

        # Get grid bounds
        min_bound = np.array(self.generator.get_min_bound(), dtype=np.float32)
        positions = np.array(positions, dtype=np.float32)

        # Convert to grid indices
        indices = ((positions - min_bound) / self.cell_size).astype(int)

        return indices


# Example usage for OMPL path planning
if __name__ == "__main__":
    # Create grid map
    grid_map = GridMap(cell_size=0.5, min_bound=[-10, -10, 0], max_bound=[10, 10, 5])

    # Initialize (must be called after world reset)
    grid_map.initialize()

    # Generate 2D map for ground robots with height filtering
    value_map = grid_map.generate("2d", min_height=0.1, max_height=2.0)

    # Generate 3D map for aerial robots
    # value_map_3d = grid_map.generate('3d')

    # Check if position is valid for planning
    start_pos = [0, 0, 0]
    goal_pos = [5, 5, 0]

    print(f"Start position valid: {grid_map.is_valid_position(start_pos)}")
    print(f"Goal position occupied: {grid_map.is_occupied(goal_pos)}")
    print(f"Map info: {grid_map.get_map_info()}")
    print(f"Map shape: {value_map.shape}")
