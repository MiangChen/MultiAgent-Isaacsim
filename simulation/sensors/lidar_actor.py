"""
LiDAR Sensor Actor - CARLA Style

LiDAR sensor actor implementation, following RobotActor pattern
"""

import numpy as np
from simulation.sensor_actor import SensorActor
from simulation.sensors.data import LidarData
from log.log_manager import LogManager

logger = LogManager.get_logger(__name__)


class LidarSensor(SensorActor):
    """
    LiDAR sensor actor (CARLA style)

    Wraps LidarIsaac implementation, following the same pattern as RobotActor:
    - RobotActor wraps Robot
    - LidarSensor wraps LidarIsaac

    Captures point cloud data and triggers callbacks
    """

    def __init__(self, lidar_impl, world, parent=None):
        """
        Initialize LiDAR sensor actor

        Args:
            lidar_impl: LidarIsaac instance from robot.sensor.lidar.LidarIsaac
            world: World instance
            parent: Parent actor (robot)
        """
        super().__init__(lidar_impl, world, parent)

    def get_type_id(self) -> str:
        """Get sensor type ID (CARLA style)"""
        return "sensor.lidar.ray_cast"

    def tick(self, step_size: float):
        """
        Physics step callback - fetch point cloud and trigger callback

        Args:
            step_size: Physics time step
        """
        if not self._is_listening or self._callback is None:
            return

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
            logger.error(f"LiDAR tick error: {e}")

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

        logger.warning(f"Unknown LiDAR data format: {frame_data.keys()}")
        return None

    def __repr__(self):
        return (
            f"LidarSensor(id={self._actor_id}, "
            f"path={self._prim_path}, "
            f"listening={self._is_listening})"
        )
