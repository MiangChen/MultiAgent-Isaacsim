"""
Camera Sensor Actor - CARLA Style

RGB camera sensor actor implementation, following RobotActor pattern
"""

from simulation.sensor_actor import SensorActor
from simulation.sensors.data import CameraData
from log.log_manager import LogManager

logger = LogManager.get_logger(__name__)


class RGBCamera(SensorActor):
    """
    RGB Camera sensor actor (CARLA style)
    
    Wraps Camera implementation, following the same pattern as RobotActor:
    - RobotActor wraps Robot
    - RGBCamera wraps Camera
    
    Captures RGB images and triggers callbacks
    """
    
    def __init__(self, camera_impl, world, parent=None):
        """
        Initialize RGB camera actor
        
        Args:
            camera_impl: Camera instance from robot.sensor.camera.Camera
            world: World instance
            parent: Parent actor (robot)
        """
        super().__init__(camera_impl, world, parent)
        
    def get_type_id(self) -> str:
        """Get sensor type ID (CARLA style)"""
        return 'sensor.camera.rgb'
        
    def tick(self, step_size: float):
        """
        Physics step callback - fetch image and trigger callback
        
        Args:
            step_size: Physics time step
        """
        if not self._is_listening or self._callback is None:
            return
            
        try:
            rgb = self.sensor.get_rgb()
            if rgb is None:
                return
                
            camera_data = CameraData(
                frame=self._world.get_frame(),
                timestamp=self._world.get_simulation_time(),
                width=self.sensor.cfg_camera.resolution[0],
                height=self.sensor.cfg_camera.resolution[1],
                raw_data=rgb,
                fov=90.0  # TODO: Get from config
            )
            
            self._trigger_callback(camera_data)
            
        except Exception as e:
            logger.error(f"Camera tick error: {e}")
            
    def __repr__(self):
        return (
            f"RGBCamera(id={self._actor_id}, "
            f"path={self._prim_path}, "
            f"listening={self._is_listening})"
        )
