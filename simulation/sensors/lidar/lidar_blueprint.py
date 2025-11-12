"""
LiDAR Blueprints - CARLA Style

Blueprint definitions for LiDAR sensors
"""

from simulation.sensors.sensor_blueprint_base import SensorBlueprint


class RayCastLidarBlueprint(SensorBlueprint):
    """
    Ray-Cast LiDAR Blueprint (CARLA style)
    
    Blueprint for sensor.lidar.ray_cast
    """
    
    def __init__(self):
        from simulation.sensors.lidar_actor import LidarSensor
        super().__init__(
            blueprint_id='sensor.lidar.ray_cast',
            sensor_class=LidarSensor,
            tags=['sensor', 'lidar', 'ray_cast']
        )
        
    def _define_attributes(self):
        """Define LiDAR attributes (CARLA standard)"""
        # Channels
        self.set_attribute('channels', 32)
        
        # Range (meters)
        self.set_attribute('range', 100.0)
        
        # Points per second
        self.set_attribute('points_per_second', 56000)
        
        # Rotation frequency (Hz)
        self.set_attribute('rotation_frequency', 10.0)
        
        # Vertical FOV
        self.set_attribute('upper_fov', 10.0)
        self.set_attribute('lower_fov', -30.0)
        
        # Sensor tick
        self.set_attribute('sensor_tick', 0.0)
        
        # Isaac Sim specific: config file name
        self.set_attribute('config_file_name', 'Hesai_XT32_SD10')
