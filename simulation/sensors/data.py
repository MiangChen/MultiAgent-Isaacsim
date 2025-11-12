"""
Sensor Data Classes - CARLA Style

Provides data containers for sensor outputs (images, point clouds, etc.)
"""

import numpy as np
from typing import Optional


class SensorData:
    """Base class for sensor data (CARLA style)"""
    
    def __init__(self, frame: int, timestamp: float):
        """
        Initialize sensor data
        
        Args:
            frame: Frame number
            timestamp: Simulation timestamp
        """
        self.frame = frame
        self.timestamp = timestamp


class CameraData(SensorData):
    """
    Camera data (CARLA style)
    
    Contains RGB image data from camera sensor
    """
    
    def __init__(
        self, 
        frame: int, 
        timestamp: float, 
        width: int, 
        height: int, 
        raw_data: np.ndarray,
        fov: float = 90.0
    ):
        """
        Initialize camera data
        
        Args:
            frame: Frame number
            timestamp: Simulation timestamp
            width: Image width
            height: Image height
            raw_data: RGB numpy array [H, W, 3]
            fov: Field of view in degrees
        """
        super().__init__(frame, timestamp)
        self.width = width
        self.height = height
        self.raw_data = raw_data
        self.fov = fov
        
    def save_to_disk(self, path: str):
        """
        Save image to disk (CARLA style)
        
        Args:
            path: File path to save image
        """
        from PIL import Image
        
        # Ensure data type is uint8
        if self.raw_data.dtype != np.uint8:
            if self.raw_data.max() <= 1.0:
                data = (self.raw_data * 255).astype(np.uint8)
            else:
                data = self.raw_data.astype(np.uint8)
        else:
            data = self.raw_data
            
        Image.fromarray(data, 'RGB').save(path)
        
    def __repr__(self):
        return (
            f"CameraData(frame={self.frame}, "
            f"timestamp={self.timestamp:.3f}, "
            f"size={self.width}x{self.height})"
        )


class LidarData(SensorData):
    """
    LiDAR data (CARLA style)
    
    Contains point cloud data from LiDAR sensor
    """
    
    def __init__(
        self, 
        frame: int, 
        timestamp: float, 
        point_cloud: np.ndarray
    ):
        """
        Initialize LiDAR data
        
        Args:
            frame: Frame number
            timestamp: Simulation timestamp
            point_cloud: Point cloud numpy array [N, 3] or [N, 4]
        """
        super().__init__(frame, timestamp)
        self._point_cloud = point_cloud
        
    @property
    def points(self) -> np.ndarray:
        """Get point cloud (CARLA style property)"""
        return self._point_cloud
        
    def save_to_disk(self, path: str):
        """
        Save point cloud to disk
        
        Args:
            path: File path to save point cloud (.npy or .ply)
        """
        if path.endswith('.npy'):
            np.save(path, self._point_cloud)
        elif path.endswith('.ply'):
            self._save_as_ply(path)
        else:
            # Default to numpy
            np.save(path, self._point_cloud)
            
    def _save_as_ply(self, path: str):
        """Save as PLY format"""
        points = self._point_cloud
        
        with open(path, 'w') as f:
            # PLY header
            f.write("ply\n")
            f.write("format ascii 1.0\n")
            f.write(f"element vertex {len(points)}\n")
            f.write("property float x\n")
            f.write("property float y\n")
            f.write("property float z\n")
            f.write("end_header\n")
            
            # Point data
            for point in points:
                f.write(f"{point[0]} {point[1]} {point[2]}\n")
                
    def __repr__(self):
        return (
            f"LidarData(frame={self.frame}, "
            f"timestamp={self.timestamp:.3f}, "
            f"points={len(self._point_cloud)})"
        )
