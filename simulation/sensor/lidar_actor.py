"""
LiDAR Sensor Actor - CARLA Style

LiDAR sensor actor implementation, following RobotActor pattern
Supports both Isaac LiDAR and Omni LiDAR implementations
"""

import numpy as np
from simulation.sensor_actor import SensorActor
from simulation.sensor.data import LidarData
from log.log_manager import LogManager

logger = LogManager.get_logger(__name__)


class LidarIsaacSensor(SensorActor):
    """
    Isaac LiDAR sensor actor (CARLA style)

    Wraps LidarIsaac implementation, following the same pattern as RobotActor:
    - RobotActor wraps Robot
    - LidarIsaacSensor wraps LidarIsaac

    Captures point cloud data and triggers callbacks
    """

    def __init__(self, lidar_impl, world, parent=None):
        """
        Initialize Isaac LiDAR sensor actor

        Args:
            lidar_impl: LidarIsaac instance from simulation.sensor.lidar.LidarIsaac
            world: World instance
            parent: Parent actor (robot)
        """
        super().__init__(lidar_impl, world, parent)
        
        # Frequency control
        self._frequency = getattr(lidar_impl.cfg_lidar, 'frequency', 10)  # Default 10Hz
        self._tick_interval = 1.0 / self._frequency if self._frequency > 0 else 0.0
        self._time_since_last_tick = 0.0

    def get_type_id(self) -> str:
        """Get sensor type ID (CARLA style)"""
        return "sensor.lidar.isaac"

    def tick(self, step_size: float):
        """
        Physics step callback - fetch point cloud and trigger callback

        Args:
            step_size: Physics time step
        """
        if not self._is_listening or self._callback is None:
            return
        
        # Frequency control: only process at specified frequency
        self._time_since_last_tick += step_size
        if self._tick_interval > 0 and self._time_since_last_tick < self._tick_interval:
            return
        
        self._time_since_last_tick = 0.0

        try:
            frame_data = self.sensor.get_current_frame()
            if not frame_data:
                return

            point_cloud = self._extract_point_cloud(frame_data)
            if point_cloud is None or len(point_cloud) == 0:
                return

            lidar_data = LidarData(
                frame=self._world.get_frame(),
                timestamp=self._world.get_simulation_time(),
                point_cloud=point_cloud,
            )

            self._trigger_callback(lidar_data)

        except Exception as e:
            logger.error(f"Isaac LiDAR tick error: {e}")

    def _extract_point_cloud(self, frame_data: dict) -> np.ndarray:
        """
        Extract point cloud from Isaac Sim LiDAR data

        Args:
            frame_data: Raw data from LidarIsaac.get_current_frame()

        Returns:
            Point cloud array [N, 3] or None
        """
        # Isaac Sim LiDAR returns data in different formats
        # Try to extract point cloud from various possible keys

        if "point_cloud" in frame_data:
            return frame_data["point_cloud"]

        # If we have azimuth, elevation, and distance, convert to XYZ
        if all(k in frame_data for k in ["azimuth", "elevation", "distance"]):
            azimuth = frame_data["azimuth"]
            elevation = frame_data["elevation"]
            distance = frame_data["distance"]

            # Convert spherical to Cartesian
            x = distance * np.cos(elevation) * np.cos(azimuth)
            y = distance * np.cos(elevation) * np.sin(azimuth)
            z = distance * np.sin(elevation)

            return np.stack([x, y, z], axis=-1)

        logger.warning(f"Unknown Isaac LiDAR data format: {frame_data.keys()}")
        return None

    def __repr__(self):
        return (
            f"LidarIsaacSensor(id={self._actor_id}, "
            f"path={self._prim_path}, "
            f"listening={self._is_listening})"
        )


class LidarOmniSensor(SensorActor):
    """
    Omni LiDAR sensor actor (CARLA style)

    Wraps LidarOmni implementation, following the same pattern as RobotActor:
    - RobotActor wraps Robot
    - LidarOmniSensor wraps LidarOmni

    Captures point cloud data and triggers callbacks
    """

    def __init__(self, lidar_impl, world, parent=None):
        """
        Initialize Omni LiDAR sensor actor

        Args:
            lidar_impl: LidarOmni instance from simulation.sensor.lidar.LidarOmni
            world: World instance
            parent: Parent actor (robot)
        """
        super().__init__(lidar_impl, world, parent)
        
        # Frequency control
        self._frequency = getattr(lidar_impl.cfg_lidar, 'frequency', 10)  # Default 10Hz
        self._tick_interval = 1.0 / self._frequency if self._frequency > 0 else 0.0
        self._time_since_last_tick = 0.0

    def get_type_id(self) -> str:
        """Get sensor type ID (CARLA style)"""
        return "sensor.lidar.omni"

    def tick(self, step_size: float):
        """
        Physics step callback - fetch point cloud and trigger callback

        Args:
            step_size: Physics time step
        """
        if not self._is_listening or self._callback is None:
            return
        
        # Frequency control: only process at specified frequency
        self._time_since_last_tick += step_size
        if self._tick_interval > 0 and self._time_since_last_tick < self._tick_interval:
            return
        
        self._time_since_last_tick = 0.0

        try:
            # Get point cloud from Omni LiDAR
            point_cloud = self.sensor.get_pointcloud()
            
            if point_cloud is None or point_cloud.size == 0:
                return

            # Reshape to [N, 3] if needed
            if len(point_cloud.shape) == 3:
                # Shape is [H, W, 3], flatten to [N, 3]
                point_cloud = point_cloud.reshape(-1, 3)

            lidar_data = LidarData(
                frame=self._world.get_simulation_time(),
                timestamp=self._world.get_simulation_time(),
                point_cloud=point_cloud,
            )

            self._trigger_callback(lidar_data)

        except Exception as e:
            logger.error(f"Omni LiDAR tick error: {e}")

    def __repr__(self):
        return (
            f"LidarOmniSensor(id={self._actor_id}, "
            f"path={self._prim_path}, "
            f"listening={self._is_listening})"
        )
