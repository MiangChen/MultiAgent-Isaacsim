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

            # 提取点云（已应用 LiDAR 自身的局部旋转）
            point_cloud = self._extract_point_cloud(frame_data)
            if point_cloud is None or len(point_cloud) == 0:
                return

            # 应用父对象的世界位姿变换
            point_cloud = self._apply_parent_transform(point_cloud)

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
        
        返回的点云已经应用了 LiDAR 的局部旋转（相对于父对象）

        Args:
            frame_data: Raw data from LidarIsaac.get_current_frame()

        Returns:
            Point cloud array [N, 3] or None
        """
        point_cloud = None

        # Isaac Sim LiDAR returns data in different formats
        # Try to extract point cloud from various possible keys

        if "point_cloud" in frame_data:
            point_cloud = frame_data["point_cloud"]

        # If we have azimuth, elevation, and distance, convert to XYZ
        elif all(k in frame_data for k in ["azimuth", "elevation", "distance"]):
            azimuth = frame_data["azimuth"]
            elevation = frame_data["elevation"]
            distance = frame_data["distance"]

            # Convert spherical to Cartesian
            x = distance * np.cos(elevation) * np.cos(azimuth)
            y = distance * np.cos(elevation) * np.sin(azimuth)
            z = distance * np.sin(elevation)

            point_cloud = np.stack([x, y, z], axis=-1)
        else:
            logger.warning(f"Unknown Isaac LiDAR data format: {frame_data.keys()}")
            return None

        # 应用 LiDAR 的局部旋转（相对于父对象的旋转）
        point_cloud = self._apply_local_rotation(point_cloud)

        return point_cloud

    def _apply_local_rotation(self, point_cloud: np.ndarray) -> np.ndarray:
        """
        应用 LiDAR 的局部旋转到点云
        
        Args:
            point_cloud: 点云 [N, 3]
            
        Returns:
            旋转后的点云 [N, 3]
        """
        # 获取 LiDAR 的局部旋转（相对于父对象）
        quat = self.sensor.cfg_lidar.quat  # (w, x, y, z)

        # 如果是单位四元数，不需要旋转
        if np.allclose(quat, [1, 0, 0, 0]):
            return point_cloud

        # 转换为旋转矩阵
        from scipy.spatial.transform import Rotation as R
        # scipy 使用 (x, y, z, w) 格式
        rotation = R.from_quat([quat[1], quat[2], quat[3], quat[0]])
        rotation_matrix = rotation.as_matrix()

        # 应用旋转: p_rotated = R * p
        points_rotated = (rotation_matrix @ point_cloud.T).T

        return points_rotated

    def _apply_parent_transform(self, point_cloud: np.ndarray) -> np.ndarray:
        """
        应用父对象（机器人）的世界位姿到点云
        
        Args:
            point_cloud: 点云 [N, 3]
            
        Returns:
            变换后的点云 [N, 3]
        """
        if self._parent is None:
            return point_cloud
        
        try:
            # 获取父对象的世界位姿
            parent_transform = self._parent.get_transform()
            pos = parent_transform.location
            
            # 获取父对象的旋转（四元数）
            rot = parent_transform.rotation
            quat = rot.to_quaternion()  # [x, y, z, w]
            
            # 转换为旋转矩阵
            from scipy.spatial.transform import Rotation as R
            rotation = R.from_quat(quat)
            rotation_matrix = rotation.as_matrix()
            
            # 应用变换: p_world = R_parent * p_local + t_parent
            points_transformed = (rotation_matrix @ point_cloud.T).T + np.array([pos.x, pos.y, pos.z])
            
            return points_transformed
            
        except Exception as e:
            logger.error(f"Failed to apply parent transform: {e}")
            return point_cloud

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
            # 注意: get_pointcloud() 已经应用了 LiDAR 自身的局部旋转
            # 但还需要应用父对象（机器人）的世界位姿
            point_cloud = self.sensor.get_pointcloud()
            # 应用父对象的世界位姿变换
            point_cloud = self._apply_parent_transform(point_cloud)
            lidar_data = LidarData(
                frame=self._world.get_frame(),
                timestamp=self._world.get_simulation_time(),
                point_cloud=point_cloud,
            )

            self._trigger_callback(lidar_data)

        except Exception as e:
            logger.error(f"Omni LiDAR tick error: {e}")

    def _apply_parent_transform(self, point_cloud: np.ndarray) -> np.ndarray:
        """
        应用父对象（机器人）的世界位姿到点云
        
        Args:
            point_cloud: 点云 [H, W, 3]
            
        Returns:
            变换后的点云（保持原始形状）
        """
        if self._parent is None:
            return point_cloud
        
        try:
            # 获取父对象的世界位姿
            parent_transform = self._parent.get_transform()
            pos = parent_transform.location
            
            # 获取父对象的旋转（四元数）
            rot = parent_transform.rotation
            quat = rot.to_quaternion()  # [x, y, z, w]
            
            # 转换为旋转矩阵
            from scipy.spatial.transform import Rotation as R
            rotation = R.from_quat(quat)
            rotation_matrix = rotation.as_matrix().astype(np.float32)  # 强制 float32
            
            # 保存原始形状和数据类型
            original_shape = point_cloud.shape
            original_dtype = point_cloud.dtype
            
            # Reshape 为 [N, 3] 进行变换
            points_flat = point_cloud.reshape(-1, 3)
            
            # 应用变换: p_world = R_parent * p_local + t_parent
            translation = np.array([pos.x, pos.y, pos.z], dtype=np.float32)
            points_transformed = (rotation_matrix @ points_flat.T).T + translation
            
            # 恢复原始形状和数据类型
            return points_transformed.reshape(original_shape).astype(original_dtype)
            
        except Exception as e:
            logger.error(f"Failed to apply parent transform: {e}")
            return point_cloud

    def __repr__(self):
        return (
            f"LidarOmniSensor(id={self._actor_id}, "
            f"path={self._prim_path}, "
            f"listening={self._is_listening})"
        )
