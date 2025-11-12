"""
Sensor ROS Bridge - CARLA Style

Bridges sensor data from simulation layer to ROS layer.
Follows CARLA pattern: sensor.listen(callback) -> ROS publish
"""

import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField, Image
from std_msgs.msg import Header
from log.log_manager import LogManager

logger = LogManager.get_logger(__name__)


class SensorRosBridge:
    """
    Base class for sensor ROS bridges
    
    Responsibilities:
    - Create ROS publishers for sensor data
    - Convert sensor data to ROS messages
    - Publish to ROS topics
    
    Usage (CARLA style):
        bridge = LidarRosBridge(node, namespace, topic_name)
        sensor.listen(bridge.publish)  # Connect sensor to ROS
    """
    
    def __init__(self, node: Node, namespace: str, topic_name: str):
        """
        Initialize sensor ROS bridge
        
        Args:
            node: ROS node
            namespace: Robot namespace
            topic_name: Topic name (e.g., 'lidar/points')
        """
        self.node = node
        self.namespace = namespace
        self.topic_name = topic_name
        self.publisher = None
        
    def publish(self, sensor_data):
        """
        Publish sensor data to ROS (to be overridden)
        
        This method is called by sensor.listen(callback)
        
        Args:
            sensor_data: Sensor data object (CameraData, LidarData, etc.)
        """
        raise NotImplementedError("Subclasses must implement publish()")


class LidarRosBridge(SensorRosBridge):
    """
    LiDAR ROS Bridge - Publishes PointCloud2
    
    Usage:
        bridge = LidarRosBridge(node, 'robot_0', 'lidar/points', sensor_actor)
        lidar_sensor.listen(bridge.publish)
    """
    
    def __init__(self, node: Node, namespace: str, topic_name: str = 'lidar/points', 
                 sensor_actor=None, frame_id: str = 'map'):
        """
        Initialize LiDAR ROS bridge
        
        Args:
            node: ROS node
            namespace: Robot namespace
            topic_name: Topic name
            sensor_actor: Sensor actor (for coordinate transformation)
            frame_id: Target frame_id ('map' or '{namespace}/lidar')
        """
        super().__init__(node, namespace, topic_name)
        
        self.sensor_actor = sensor_actor
        self.frame_id = frame_id
        
        # Create PointCloud2 publisher
        full_topic = f'/{namespace}/{topic_name}'
        self.publisher = node.create_publisher(PointCloud2, full_topic, 10)
        
        logger.info(f"LiDAR ROS bridge created: {full_topic} (frame: {frame_id})")
    
    def publish(self, lidar_data):
        """
        Publish LiDAR data as PointCloud2
        
        Args:
            lidar_data: LidarData object with point_cloud [N, 3] array
        """
        try:
            # Convert to PointCloud2 message
            msg = self._create_pointcloud2_msg(lidar_data)
            self.publisher.publish(msg)
            
        except Exception as e:
            logger.error(f"Failed to publish LiDAR data: {e}")
    
    def _transform_points_to_map(self, points: np.ndarray) -> np.ndarray:
        """
        Transform points from sensor frame to map frame
        
        Args:
            points: Points in sensor frame [N, 3]
            
        Returns:
            Points in map frame [N, 3]
        """
        if self.sensor_actor is None:
            logger.warning("No sensor_actor provided, cannot transform to map frame")
            return points
        
        try:
            # Get sensor world transform
            transform = self.sensor_actor.get_transform()
            pos = transform.location
            
            # Get rotation (quaternion)
            rot = transform.rotation
            quat = rot.to_quaternion()  # [x, y, z, w]
            
            # Convert quaternion to rotation matrix
            from scipy.spatial.transform import Rotation as R
            rotation = R.from_quat(quat)  # scipy uses [x, y, z, w]
            rotation_matrix = rotation.as_matrix()
            
            # Transform points: p_map = R * p_sensor + t
            points_map = (rotation_matrix @ points.T).T + np.array([pos.x, pos.y, pos.z])
            
            return points_map
            
        except Exception as e:
            logger.error(f"Failed to transform points to map frame: {e}")
            return points
    
    def _create_pointcloud2_msg(self, lidar_data) -> PointCloud2:
        """
        Create PointCloud2 message from LiDAR data
        
        Args:
            lidar_data: LidarData object
            
        Returns:
            PointCloud2 message
        """
        msg = PointCloud2()
        
        # Header
        msg.header = Header()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        
        # Point cloud data
        points = lidar_data.points  # [N, 3] numpy array
        
        # Transform to map frame if needed
        if self.frame_id == 'map' and self.sensor_actor is not None:
            points = self._transform_points_to_map(points)
        
        # PointCloud2 fields (x, y, z)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        
        # Point cloud properties
        msg.height = 1
        msg.width = len(points)
        msg.point_step = 12  # 3 floats * 4 bytes
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        msg.is_bigendian = False
        
        # Convert to bytes
        msg.data = points.astype(np.float32).tobytes()
        
        return msg


class CameraRosBridge(SensorRosBridge):
    """
    Camera ROS Bridge - Publishes Image
    
    Usage:
        bridge = CameraRosBridge(node, 'robot_0', 'camera/image_raw')
        camera_sensor.listen(bridge.publish)
    """
    
    def __init__(self, node: Node, namespace: str, topic_name: str = 'camera/image_raw'):
        super().__init__(node, namespace, topic_name)
        
        # Create Image publisher
        full_topic = f'/{namespace}/{topic_name}'
        self.publisher = node.create_publisher(Image, full_topic, 10)
        
        logger.info(f"Camera ROS bridge created: {full_topic}")
    
    def publish(self, camera_data):
        """
        Publish camera data as Image
        
        Args:
            camera_data: CameraData object with raw_data [H, W, 3] array
        """
        try:
            # Convert to Image message
            msg = self._create_image_msg(camera_data)
            self.publisher.publish(msg)
            
        except Exception as e:
            logger.error(f"Failed to publish camera data: {e}")
    
    def _create_image_msg(self, camera_data) -> Image:
        """
        Create Image message from camera data
        
        Args:
            camera_data: CameraData object
            
        Returns:
            Image message
        """
        msg = Image()
        
        # Header
        msg.header = Header()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = f'{self.namespace}/camera'
        
        # Image properties
        msg.height = camera_data.height
        msg.width = camera_data.width
        msg.encoding = 'rgb8'
        msg.step = msg.width * 3
        msg.is_bigendian = False
        
        # Image data
        if camera_data.raw_data.dtype != np.uint8:
            if camera_data.raw_data.max() <= 1.0:
                data = (camera_data.raw_data * 255).astype(np.uint8)
            else:
                data = camera_data.raw_data.astype(np.uint8)
        else:
            data = camera_data.raw_data
        
        msg.data = data.tobytes()
        
        return msg
