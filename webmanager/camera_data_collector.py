"""
Camera frame data collection implementation for the WebManager.
Handles camera frame capture from Isaac Sim viewport, JPEG encoding, and binary streaming.
"""

import time
import logging
import threading
from typing import Dict, List, Optional, Any, Tuple
import numpy as np
import cv2
from io import BytesIO
from PIL import Image

from .models import BinaryDataType
from .data_collector import CameraDataSource

logger = logging.getLogger(__name__)


class IsaacSimCameraDataSource(CameraDataSource):
    """
    Camera data source that integrates with Isaac Sim viewport and camera systems.
    Provides real-time camera frame capture, encoding, and streaming capabilities.
    """
    
    def __init__(self, isaac_sim_app=None, viewport_manager=None):
        """
        Initialize the camera data source.
        
        Args:
            isaac_sim_app: Reference to Isaac Sim application instance
            viewport_manager: Reference to viewport manager for frame capture
        """
        self.isaac_sim_app = isaac_sim_app
        self.viewport_manager = viewport_manager
        
        # Camera configuration
        self.available_cameras: Dict[str, Dict[str, Any]] = {}
        self.active_cameras: Dict[str, bool] = {}
        self.camera_frame_cache: Dict[str, np.ndarray] = {}
        
        # Frame processing settings
        self.default_quality = 80
        self.default_format = 'JPEG'
        self.max_frame_size = (1920, 1080)
        self.compression_settings = {
            'JPEG': [cv2.IMWRITE_JPEG_QUALITY, 80],
            'PNG': [cv2.IMWRITE_PNG_COMPRESSION, 6],
            'WEBP': [cv2.IMWRITE_WEBP_QUALITY, 80]
        }
        
        # Performance tracking
        self.frame_capture_stats = {
            'total_frames': 0,
            'failed_captures': 0,
            'average_capture_time': 0.0,
            'average_encode_time': 0.0
        }
        
        # Initialize camera discovery
        self._discover_cameras()
        
        logger.info(f"IsaacSimCameraDataSource initialized with {len(self.available_cameras)} cameras")
    
    def get_data(self) -> Optional[np.ndarray]:
        """Get current camera frame from default camera."""
        return self.get_camera_frame()
    
    def is_available(self) -> bool:
        """Check if camera data source is available."""
        return (self.isaac_sim_app is not None or 
                self.viewport_manager is not None or 
                len(self.available_cameras) > 0)
    
    def get_camera_frame(self, camera_id: str = "default") -> Optional[np.ndarray]:
        """
        Get current camera frame from specified camera.
        
        Args:
            camera_id: Camera identifier to capture from
            
        Returns:
            Camera frame as numpy array (H, W, C) or None if unavailable
        """
        capture_start = time.time()
        
        try:
            # Get frame from Isaac Sim viewport
            if camera_id == "default" or camera_id == "viewport":
                frame = self._capture_viewport_frame()
            else:
                frame = self._capture_camera_frame(camera_id)
            
            if frame is not None:
                # Cache the frame
                self.camera_frame_cache[camera_id] = frame
                
                # Update statistics
                capture_time = time.time() - capture_start
                self._update_capture_stats(capture_time, success=True)
                
                return frame
            else:
                self._update_capture_stats(0, success=False)
                return None
                
        except Exception as e:
            logger.error(f"Error capturing frame from camera {camera_id}: {e}")
            self._update_capture_stats(0, success=False)
            return None
    
    def get_available_cameras(self) -> List[str]:
        """Get list of available camera IDs."""
        return list(self.available_cameras.keys())
    
    def _discover_cameras(self):
        """Discover available cameras in the Isaac Sim environment."""
        try:
            # Default viewport camera
            self.available_cameras["default"] = {
                'name': 'Default Viewport',
                'type': 'viewport',
                'resolution': (1920, 1080),
                'active': True
            }
            
            self.available_cameras["viewport"] = {
                'name': 'Main Viewport',
                'type': 'viewport',
                'resolution': (1920, 1080),
                'active': True
            }
            
            # Discover Isaac Sim cameras if available
            if self.isaac_sim_app:
                self._discover_isaac_sim_cameras()
            
            # Initialize active camera states
            for camera_id in self.available_cameras:
                self.active_cameras[camera_id] = True
            
            logger.info(f"Discovered cameras: {list(self.available_cameras.keys())}")
            
        except Exception as e:
            logger.error(f"Error discovering cameras: {e}")
    
    def _discover_isaac_sim_cameras(self):
        """Discover cameras from Isaac Sim scene."""
        try:
            # Method 1: Use viewport manager to discover cameras
            if self.viewport_manager:
                # Get camera mappings from viewport manager
                camera_mappings = self.viewport_manager.list_camera_mappings()
                
                for viewport_name, camera_path in camera_mappings.items():
                    camera_id = f"viewport_{viewport_name}"
                    self.available_cameras[camera_id] = {
                        'name': f'Viewport Camera ({viewport_name})',
                        'type': 'viewport_camera',
                        'camera_path': camera_path,
                        'viewport_name': viewport_name,
                        'resolution': (1920, 1080),
                        'active': True
                    }
                
                # Get all registered viewports
                viewports = self.viewport_manager.list_viewports()
                for viewport_name in viewports:
                    if f"viewport_{viewport_name}" not in self.available_cameras:
                        camera_id = f"viewport_{viewport_name}"
                        self.available_cameras[camera_id] = {
                            'name': f'Viewport ({viewport_name})',
                            'type': 'viewport',
                            'viewport_name': viewport_name,
                            'resolution': (1920, 1080),
                            'active': True
                        }
            
            # Method 2: Try to discover cameras from Isaac Sim scene
            if self.isaac_sim_app:
                self._discover_scene_cameras()
            
            # Method 3: Add some default robot cameras for testing
            self._add_default_robot_cameras()
            
        except Exception as e:
            logger.error(f"Error discovering Isaac Sim cameras: {e}")
    
    def _discover_scene_cameras(self):
        """Discover cameras from the Isaac Sim scene."""
        try:
            # This would use Isaac Sim's scene API to find camera prims
            logger.debug("Discovering cameras from Isaac Sim scene")
            
            # Placeholder for Isaac Sim scene camera discovery
            # In real implementation, this might be:
            # from omni.usd import get_context
            # stage = get_context().get_stage()
            # camera_prims = [prim for prim in stage.Traverse() if prim.GetTypeName() == "Camera"]
            
            # For now, add some common camera paths that might exist
            common_camera_paths = [
                "/World/semantic_camera",
                "/World/Camera",
                "/World/MainCamera",
                "/OmniverseKit_Persp",
                "/OmniverseKit_Front",
                "/OmniverseKit_Top"
            ]
            
            for i, camera_path in enumerate(common_camera_paths):
                camera_id = f"scene_camera_{i}"
                self.available_cameras[camera_id] = {
                    'name': f'Scene Camera ({camera_path.split("/")[-1]})',
                    'type': 'scene_camera',
                    'camera_path': camera_path,
                    'resolution': (1920, 1080),
                    'active': False  # Start inactive, activate when found
                }
            
        except Exception as e:
            logger.error(f"Error discovering scene cameras: {e}")
    
    def _add_default_robot_cameras(self):
        """Add default robot cameras for testing."""
        try:
            # Add some robot cameras that might be created by the swarm manager
            robot_camera_configs = [
                ("jetbot_0_camera", "Jetbot 0 Camera", "/World/jetbot_0/Camera"),
                ("jetbot_1_camera", "Jetbot 1 Camera", "/World/jetbot_1/Camera"),
                ("h1_0_camera", "H1 Robot 0 Camera", "/World/h1_0/Camera"),
                ("cf2x_0_camera", "CF2X Drone 0 Camera", "/World/cf2x_0/Camera"),
            ]
            
            for camera_id, camera_name, camera_path in robot_camera_configs:
                if camera_id not in self.available_cameras:
                    self.available_cameras[camera_id] = {
                        'name': camera_name,
                        'type': 'robot_camera',
                        'camera_path': camera_path,
                        'resolution': (640, 480),
                        'active': False  # Start inactive
                    }
            
        except Exception as e:
            logger.error(f"Error adding default robot cameras: {e}")
    
    def _capture_viewport_frame(self) -> Optional[np.ndarray]:
        """
        Capture frame from Isaac Sim main viewport.
        
        Returns:
            Viewport frame as numpy array or None if failed
        """
        try:
            # Method 1: Use viewport manager to get viewport and capture frame
            if self.viewport_manager:
                # Get the main viewport
                viewport = self.viewport_manager.get_viewport("Viewport")
                if viewport is not None:
                    frame = self._capture_from_viewport_object(viewport)
                    if frame is not None:
                        return self._process_frame(frame)
                
                # Try other registered viewports
                viewports = self.viewport_manager.list_viewports()
                for viewport_name in viewports:
                    viewport = self.viewport_manager.get_viewport(viewport_name)
                    if viewport is not None:
                        frame = self._capture_from_viewport_object(viewport)
                        if frame is not None:
                            return self._process_frame(frame)
            
            # Method 2: Use Isaac Sim application viewport directly
            elif self.isaac_sim_app:
                frame = self._capture_from_isaac_sim_viewport()
                if frame is not None:
                    return self._process_frame(frame)
            
            # Method 3: Generate test frame for development
            else:
                return self._generate_test_frame()
            
        except Exception as e:
            logger.error(f"Error capturing viewport frame: {e}")
            return None
    
    def _capture_from_viewport_object(self, viewport) -> Optional[np.ndarray]:
        """
        Capture frame from a specific viewport object.
        
        Args:
            viewport: Viewport object to capture from
            
        Returns:
            Captured frame as numpy array or None if failed
        """
        try:
            # Method 1: Try direct frame capture if viewport has the method
            if hasattr(viewport, 'capture_frame'):
                frame = viewport.capture_frame()
                if frame is not None:
                    return frame
            
            # Method 2: Try to get frame buffer if available
            elif hasattr(viewport, 'get_frame_buffer'):
                frame_buffer = viewport.get_frame_buffer()
                if frame_buffer is not None:
                    return np.array(frame_buffer)
            
            # Method 3: Try Isaac Sim specific viewport capture
            elif hasattr(viewport, 'get_texture'):
                texture = viewport.get_texture()
                if texture is not None:
                    # Convert texture to numpy array
                    return self._texture_to_numpy(texture)
            
            # Method 4: Use Isaac Sim viewport API if available
            else:
                return self._capture_from_isaac_sim_viewport_api(viewport)
            
        except Exception as e:
            logger.error(f"Error capturing from viewport object: {e}")
            return None
    
    def _capture_from_isaac_sim_viewport(self) -> Optional[np.ndarray]:
        """Capture frame directly from Isaac Sim viewport."""
        try:
            # Try to use Isaac Sim's viewport capture API
            logger.debug("Capturing from Isaac Sim viewport")
            
            # Method 1: Use omni.kit.viewport if available
            try:
                import omni.kit.viewport.utility as viewport_utils
                viewport = viewport_utils.get_viewport_from_window_name("Viewport")
                if viewport:
                    return self._capture_from_viewport_object(viewport)
            except ImportError:
                pass
            
            # Method 2: Use omni.kit.viewport.window if available
            try:
                from omni.kit.viewport.window import get_viewport_window
                viewport_window = get_viewport_window()
                if viewport_window:
                    frame = viewport_window.capture_frame()
                    if frame is not None:
                        return np.array(frame)
            except ImportError:
                pass
            
            # Method 3: Generate test frame as fallback
            return self._generate_test_frame()
            
        except Exception as e:
            logger.error(f"Error capturing from Isaac Sim viewport: {e}")
            return None
    
    def _capture_from_isaac_sim_viewport_api(self, viewport) -> Optional[np.ndarray]:
        """
        Capture frame using Isaac Sim viewport API.
        
        Args:
            viewport: Isaac Sim viewport object
            
        Returns:
            Captured frame or None if failed
        """
        try:
            # This would implement Isaac Sim specific viewport capture
            # The exact implementation depends on the Isaac Sim version and API
            
            # Placeholder for Isaac Sim viewport capture
            logger.debug("Using Isaac Sim viewport API for capture")
            
            # In a real implementation, this might be:
            # frame_data = viewport.get_render_product_data()
            # return np.frombuffer(frame_data, dtype=np.uint8).reshape(height, width, channels)
            
            return None
            
        except Exception as e:
            logger.error(f"Error using Isaac Sim viewport API: {e}")
            return None
    
    def _texture_to_numpy(self, texture) -> Optional[np.ndarray]:
        """
        Convert texture object to numpy array.
        
        Args:
            texture: Texture object from viewport
            
        Returns:
            Numpy array representation of texture
        """
        try:
            # This would convert Isaac Sim texture to numpy array
            # Implementation depends on texture format and Isaac Sim API
            
            logger.debug("Converting texture to numpy array")
            
            # Placeholder implementation
            return None
            
        except Exception as e:
            logger.error(f"Error converting texture to numpy: {e}")
            return None
    
    def _capture_camera_frame(self, camera_id: str) -> Optional[np.ndarray]:
        """
        Capture frame from specific camera in Isaac Sim.
        
        Args:
            camera_id: Specific camera to capture from
            
        Returns:
            Camera frame as numpy array or None if failed
        """
        try:
            if camera_id not in self.available_cameras:
                logger.warning(f"Camera {camera_id} not found in available cameras")
                return None
            
            camera_info = self.available_cameras[camera_id]
            camera_type = camera_info['type']
            
            if camera_type in ['viewport', 'viewport_camera']:
                # Use viewport manager to capture from specific viewport
                if 'viewport_name' in camera_info and self.viewport_manager:
                    viewport = self.viewport_manager.get_viewport(camera_info['viewport_name'])
                    if viewport:
                        return self._capture_from_viewport_object(viewport)
                return self._capture_viewport_frame()
            
            elif camera_type in ['robot_camera', 'scene_camera']:
                return self._capture_scene_camera_frame(camera_id)
            
            else:
                logger.warning(f"Unknown camera type: {camera_type}")
                return None
            
        except Exception as e:
            logger.error(f"Error capturing from camera {camera_id}: {e}")
            return None
    
    def _capture_scene_camera_frame(self, camera_id: str) -> Optional[np.ndarray]:
        """
        Capture frame from scene camera (robot or scene-mounted).
        
        Args:
            camera_id: Camera identifier
            
        Returns:
            Camera frame or None if failed
        """
        try:
            camera_info = self.available_cameras[camera_id]
            camera_path = camera_info.get('camera_path')
            
            if camera_path:
                # Try to capture from Isaac Sim camera prim
                frame = self._capture_from_camera_prim(camera_path)
                if frame is not None:
                    return frame
            
            # Fallback: generate test frame with camera info
            frame = self._generate_test_frame()
            if frame is not None:
                # Add camera ID and path overlay for identification
                cv2.putText(frame, f"Camera: {camera_id}", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                if camera_path:
                    cv2.putText(frame, f"Path: {camera_path}", (10, 60), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
            
            return frame
            
        except Exception as e:
            logger.error(f"Error capturing scene camera {camera_id}: {e}")
            return None
    
    def _capture_from_camera_prim(self, camera_path: str) -> Optional[np.ndarray]:
        """
        Capture frame from Isaac Sim camera prim.
        
        Args:
            camera_path: USD path to camera prim
            
        Returns:
            Camera frame or None if failed
        """
        try:
            # This would use Isaac Sim's camera API to capture from a specific camera prim
            logger.debug(f"Capturing from camera prim: {camera_path}")
            
            # Placeholder for Isaac Sim camera prim capture
            # In real implementation, this might be:
            # from omni.isaac.core.utils.camera import Camera
            # camera = Camera(prim_path=camera_path)
            # frame = camera.get_rgba()
            # return cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
            
            # For now, return None to fall back to test frame
            return None
            
        except Exception as e:
            logger.error(f"Error capturing from camera prim {camera_path}: {e}")
            return None
    
    def switch_camera_viewport(self, camera_id: str, viewport_name: str = "Viewport") -> bool:
        """
        Switch a viewport to show a specific camera.
        
        Args:
            camera_id: Camera to switch to
            viewport_name: Viewport to switch
            
        Returns:
            True if successful, False otherwise
        """
        try:
            if camera_id not in self.available_cameras:
                logger.warning(f"Camera {camera_id} not found")
                return False
            
            camera_info = self.available_cameras[camera_id]
            camera_path = camera_info.get('camera_path')
            
            if not camera_path:
                logger.warning(f"No camera path found for {camera_id}")
                return False
            
            if self.viewport_manager:
                success = self.viewport_manager.change_viewport(
                    camera_prim_path=camera_path,
                    viewport_name=viewport_name
                )
                
                if success:
                    logger.info(f"Switched viewport {viewport_name} to camera {camera_id}")
                    return True
                else:
                    logger.warning(f"Failed to switch viewport {viewport_name} to camera {camera_id}")
                    return False
            else:
                logger.warning("Viewport manager not available for camera switching")
                return False
            
        except Exception as e:
            logger.error(f"Error switching camera viewport: {e}")
            return False
    
    def _generate_test_frame(self) -> np.ndarray:
        """Generate a test frame for development and testing."""
        try:
            # Create a test pattern
            height, width = 480, 640
            frame = np.zeros((height, width, 3), dtype=np.uint8)
            
            # Add gradient background
            for y in range(height):
                for x in range(width):
                    frame[y, x] = [
                        int(255 * x / width),
                        int(255 * y / height),
                        int(255 * (x + y) / (width + height))
                    ]
            
            # Add timestamp
            timestamp = time.strftime("%H:%M:%S", time.localtime())
            cv2.putText(frame, f"Test Frame - {timestamp}", (10, height - 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # Add moving element for animation
            center_x = int(width / 2 + 100 * np.sin(time.time()))
            center_y = int(height / 2 + 50 * np.cos(time.time()))
            cv2.circle(frame, (center_x, center_y), 20, (0, 255, 0), -1)
            
            return frame
            
        except Exception as e:
            logger.error(f"Error generating test frame: {e}")
            return np.zeros((480, 640, 3), dtype=np.uint8)
    
    def _process_frame(self, frame: np.ndarray) -> np.ndarray:
        """
        Process captured frame (resize, color conversion, etc.).
        
        Args:
            frame: Raw frame data
            
        Returns:
            Processed frame
        """
        try:
            # Ensure frame is in correct format (BGR for OpenCV)
            if len(frame.shape) == 3 and frame.shape[2] == 4:
                # Convert RGBA to BGR
                frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
            elif len(frame.shape) == 3 and frame.shape[2] == 3:
                # Assume RGB, convert to BGR
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            
            # Resize if frame is too large
            height, width = frame.shape[:2]
            max_width, max_height = self.max_frame_size
            
            if width > max_width or height > max_height:
                scale = min(max_width / width, max_height / height)
                new_width = int(width * scale)
                new_height = int(height * scale)
                frame = cv2.resize(frame, (new_width, new_height), interpolation=cv2.INTER_AREA)
            
            return frame
            
        except Exception as e:
            logger.error(f"Error processing frame: {e}")
            return frame
    
    def encode_frame(self, frame: np.ndarray, format: str = None, quality: int = None) -> Optional[bytes]:
        """
        Encode frame to specified format for transmission.
        
        Args:
            frame: Frame to encode
            format: Image format ('JPEG', 'PNG', 'WEBP')
            quality: Compression quality (1-100)
            
        Returns:
            Encoded frame as bytes or None if failed
        """
        encode_start = time.time()
        
        try:
            format = format or self.default_format
            quality = quality or self.default_quality
            
            # Get compression settings
            if format.upper() in self.compression_settings:
                compression_params = self.compression_settings[format.upper()].copy()
                if format.upper() in ['JPEG', 'WEBP']:
                    compression_params[1] = quality
                elif format.upper() == 'PNG':
                    # PNG compression level (0-9, higher = more compression)
                    compression_params[1] = min(9, max(0, int((100 - quality) / 10)))
            else:
                compression_params = [cv2.IMWRITE_JPEG_QUALITY, quality]
            
            # Encode frame
            if format.upper() == 'JPEG':
                success, encoded_frame = cv2.imencode('.jpg', frame, compression_params)
            elif format.upper() == 'PNG':
                success, encoded_frame = cv2.imencode('.png', frame, compression_params)
            elif format.upper() == 'WEBP':
                success, encoded_frame = cv2.imencode('.webp', frame, compression_params)
            else:
                # Default to JPEG
                success, encoded_frame = cv2.imencode('.jpg', frame, 
                                                    [cv2.IMWRITE_JPEG_QUALITY, quality])
            
            if success:
                encoded_bytes = encoded_frame.tobytes()
                
                # Update encoding statistics
                encode_time = time.time() - encode_start
                self._update_encode_stats(encode_time)
                
                logger.debug(f"Encoded frame: {len(encoded_bytes)} bytes, "
                           f"format={format}, quality={quality}, time={encode_time:.3f}s")
                
                return encoded_bytes
            else:
                logger.error(f"Failed to encode frame in {format} format")
                return None
                
        except Exception as e:
            logger.error(f"Error encoding frame: {e}")
            return None 
   
    def get_encoded_frame(self, camera_id: str = "default", format: str = None, 
                         quality: int = None) -> Optional[bytes]:
        """
        Get encoded frame from specified camera.
        
        Args:
            camera_id: Camera to capture from
            format: Encoding format
            quality: Compression quality
            
        Returns:
            Encoded frame bytes or None if failed
        """
        frame = self.get_camera_frame(camera_id)
        if frame is not None:
            return self.encode_frame(frame, format, quality)
        return None
    
    def set_camera_active(self, camera_id: str, active: bool):
        """
        Set camera active/inactive state.
        
        Args:
            camera_id: Camera identifier
            active: Whether camera should be active
        """
        if camera_id in self.available_cameras:
            self.active_cameras[camera_id] = active
            self.available_cameras[camera_id]['active'] = active
            logger.info(f"Camera {camera_id} set to {'active' if active else 'inactive'}")
        else:
            logger.warning(f"Camera {camera_id} not found")
    
    def get_camera_info(self, camera_id: str) -> Optional[Dict[str, Any]]:
        """
        Get information about a specific camera.
        
        Args:
            camera_id: Camera identifier
            
        Returns:
            Camera information dictionary or None if not found
        """
        if camera_id in self.available_cameras:
            info = self.available_cameras[camera_id].copy()
            info['is_active'] = self.active_cameras.get(camera_id, False)
            info['has_cached_frame'] = camera_id in self.camera_frame_cache
            return info
        return None
    
    def get_all_camera_info(self) -> Dict[str, Dict[str, Any]]:
        """Get information about all available cameras."""
        camera_info = {}
        for camera_id in self.available_cameras:
            camera_info[camera_id] = self.get_camera_info(camera_id)
        return camera_info
    
    def set_compression_settings(self, format: str, settings: List[int]):
        """
        Set compression settings for a specific format.
        
        Args:
            format: Image format ('JPEG', 'PNG', 'WEBP')
            settings: OpenCV compression parameters
        """
        self.compression_settings[format.upper()] = settings
        logger.info(f"Updated compression settings for {format}: {settings}")
    
    def set_max_frame_size(self, width: int, height: int):
        """
        Set maximum frame size for processing.
        
        Args:
            width: Maximum frame width
            height: Maximum frame height
        """
        self.max_frame_size = (width, height)
        logger.info(f"Set max frame size to {width}x{height}")
    
    def get_capture_stats(self) -> Dict[str, Any]:
        """Get camera capture statistics."""
        return {
            'frame_stats': self.frame_capture_stats.copy(),
            'active_cameras': sum(1 for active in self.active_cameras.values() if active),
            'total_cameras': len(self.available_cameras),
            'cached_frames': len(self.camera_frame_cache),
            'timestamp': time.time()
        }
    
    def _update_capture_stats(self, capture_time: float, success: bool):
        """Update frame capture statistics."""
        self.frame_capture_stats['total_frames'] += 1
        
        if success:
            # Update average capture time
            total = self.frame_capture_stats['total_frames']
            current_avg = self.frame_capture_stats['average_capture_time']
            self.frame_capture_stats['average_capture_time'] = (
                (current_avg * (total - 1) + capture_time) / total
            )
        else:
            self.frame_capture_stats['failed_captures'] += 1
    
    def _update_encode_stats(self, encode_time: float):
        """Update frame encoding statistics."""
        # Update average encode time
        total = self.frame_capture_stats['total_frames']
        current_avg = self.frame_capture_stats['average_encode_time']
        self.frame_capture_stats['average_encode_time'] = (
            (current_avg * (total - 1) + encode_time) / total
        )
    
    def clear_frame_cache(self):
        """Clear all cached frames."""
        self.camera_frame_cache.clear()
        logger.info("Camera frame cache cleared")
    
    def benchmark_encoding(self, test_duration: float = 10.0) -> Dict[str, Any]:
        """
        Benchmark encoding performance for different formats and qualities.
        
        Args:
            test_duration: Duration of benchmark test in seconds
            
        Returns:
            Benchmark results dictionary
        """
        logger.info(f"Starting encoding benchmark for {test_duration}s")
        
        # Generate test frame
        test_frame = self._generate_test_frame()
        
        formats = ['JPEG', 'PNG', 'WEBP']
        qualities = [50, 70, 90] if test_duration > 5 else [70]
        
        results = {}
        
        for format in formats:
            results[format] = {}
            
            for quality in qualities:
                start_time = time.time()
                encode_count = 0
                total_size = 0
                
                while time.time() - start_time < (test_duration / len(formats) / len(qualities)):
                    encoded = self.encode_frame(test_frame, format, quality)
                    if encoded:
                        encode_count += 1
                        total_size += len(encoded)
                
                elapsed = time.time() - start_time
                
                results[format][quality] = {
                    'frames_encoded': encode_count,
                    'fps': encode_count / elapsed if elapsed > 0 else 0,
                    'average_size_bytes': total_size / encode_count if encode_count > 0 else 0,
                    'total_time': elapsed
                }
        
        logger.info("Encoding benchmark completed")
        return results


class CameraStreamManager:
    """
    Manager for handling multiple camera streams and their encoding/transmission.
    """
    
    def __init__(self, camera_source: IsaacSimCameraDataSource, web_server=None):
        """
        Initialize the camera stream manager.
        
        Args:
            camera_source: Camera data source instance
            web_server: Web server for streaming frames
        """
        self.camera_source = camera_source
        self.web_server = web_server
        
        # Streaming configuration
        self.streaming_cameras: Dict[str, Dict[str, Any]] = {}
        self.stream_settings = {
            'default_fps': 30,
            'default_quality': 80,
            'default_format': 'JPEG'
        }
        
        # Streaming control
        self.streaming_active = False
        self.stream_threads: Dict[str, threading.Thread] = {}
        
        logger.info("CameraStreamManager initialized")
    
    def start_camera_stream(self, camera_id: str, fps: int = None, 
                           quality: int = None, format: str = None):
        """
        Start streaming from a specific camera.
        
        Args:
            camera_id: Camera to stream from
            fps: Target streaming FPS
            quality: Encoding quality
            format: Encoding format
        """
        if camera_id in self.streaming_cameras:
            logger.warning(f"Camera {camera_id} is already streaming")
            return
        
        # Set up stream configuration
        stream_config = {
            'fps': fps or self.stream_settings['default_fps'],
            'quality': quality or self.stream_settings['default_quality'],
            'format': format or self.stream_settings['default_format'],
            'active': True,
            'frame_count': 0,
            'start_time': time.time()
        }
        
        self.streaming_cameras[camera_id] = stream_config
        
        # Start streaming thread
        stream_thread = threading.Thread(
            target=self._stream_camera_loop,
            args=(camera_id,),
            daemon=True
        )
        
        self.stream_threads[camera_id] = stream_thread
        stream_thread.start()
        
        logger.info(f"Started streaming camera {camera_id} at {stream_config['fps']}fps")
    
    def stop_camera_stream(self, camera_id: str):
        """
        Stop streaming from a specific camera.
        
        Args:
            camera_id: Camera to stop streaming
        """
        if camera_id not in self.streaming_cameras:
            logger.warning(f"Camera {camera_id} is not streaming")
            return
        
        # Mark stream as inactive
        self.streaming_cameras[camera_id]['active'] = False
        
        # Wait for thread to finish
        if camera_id in self.stream_threads:
            thread = self.stream_threads[camera_id]
            if thread.is_alive():
                thread.join(timeout=2)
            del self.stream_threads[camera_id]
        
        # Remove from streaming cameras
        del self.streaming_cameras[camera_id]
        
        logger.info(f"Stopped streaming camera {camera_id}")
    
    def stop_all_streams(self):
        """Stop all active camera streams."""
        camera_ids = list(self.streaming_cameras.keys())
        for camera_id in camera_ids:
            self.stop_camera_stream(camera_id)
        
        logger.info("Stopped all camera streams")
    
    def _stream_camera_loop(self, camera_id: str):
        """
        Main streaming loop for a specific camera.
        
        Args:
            camera_id: Camera identifier
        """
        if camera_id not in self.streaming_cameras:
            return
        
        config = self.streaming_cameras[camera_id]
        frame_interval = 1.0 / config['fps']
        
        logger.info(f"Starting stream loop for camera {camera_id}")
        
        while config.get('active', False):
            loop_start = time.time()
            
            try:
                # Capture and encode frame
                encoded_frame = self.camera_source.get_encoded_frame(
                    camera_id=camera_id,
                    format=config['format'],
                    quality=config['quality']
                )
                
                if encoded_frame and self.web_server:
                    # Send frame via WebSocket
                    metadata = {
                        'camera_id': camera_id,
                        'format': config['format'],
                        'quality': config['quality'],
                        'frame_number': config['frame_count']
                    }
                    
                    # Use asyncio to send the frame
                    import asyncio
                    try:
                        loop = asyncio.get_event_loop()
                        loop.create_task(
                            self.web_server.send_camera_frame(encoded_frame, camera_id, metadata)
                        )
                    except RuntimeError:
                        # No event loop running, skip this frame
                        pass
                
                config['frame_count'] += 1
                
                # Wait for next frame
                elapsed = time.time() - loop_start
                sleep_time = max(0, frame_interval - elapsed)
                if sleep_time > 0:
                    time.sleep(sleep_time)
                
            except Exception as e:
                logger.error(f"Error in stream loop for camera {camera_id}: {e}")
                time.sleep(frame_interval)
        
        logger.info(f"Stream loop ended for camera {camera_id}")
    
    def get_stream_status(self) -> Dict[str, Any]:
        """Get status of all active streams."""
        status = {}
        
        for camera_id, config in self.streaming_cameras.items():
            runtime = time.time() - config['start_time']
            actual_fps = config['frame_count'] / runtime if runtime > 0 else 0
            
            status[camera_id] = {
                'active': config['active'],
                'target_fps': config['fps'],
                'actual_fps': actual_fps,
                'frame_count': config['frame_count'],
                'runtime_seconds': runtime,
                'format': config['format'],
                'quality': config['quality']
            }
        
        return status
    
    def update_stream_settings(self, camera_id: str, **kwargs):
        """
        Update streaming settings for a camera.
        
        Args:
            camera_id: Camera identifier
            **kwargs: Settings to update (fps, quality, format)
        """
        if camera_id not in self.streaming_cameras:
            logger.warning(f"Camera {camera_id} is not streaming")
            return
        
        config = self.streaming_cameras[camera_id]
        
        for key, value in kwargs.items():
            if key in ['fps', 'quality', 'format']:
                config[key] = value
                logger.info(f"Updated {key} for camera {camera_id}: {value}")
    
    def set_default_stream_settings(self, **kwargs):
        """
        Set default streaming settings.
        
        Args:
            **kwargs: Default settings (default_fps, default_quality, default_format)
        """
        for key, value in kwargs.items():
            if key in self.stream_settings:
                self.stream_settings[key] = value
                logger.info(f"Updated default {key}: {value}")