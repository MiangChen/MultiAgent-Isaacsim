"""
LiDAR Blueprints - CARLA Style

Blueprint definitions for LiDAR sensor (Isaac and Omni implementations)
"""

from simulation.sensor.sensor_blueprint_base import SensorBlueprint


class IsaacLidarBlueprint(SensorBlueprint):
    """
    Isaac LiDAR Blueprint (CARLA style)

    Blueprint for sensor.lidar.isaac
    Uses Isaac Sim's LidarRtx implementation
    """

    def __init__(self):
        from simulation.sensor.lidar_actor import LidarIsaacSensor

        super().__init__(
            blueprint_id="sensor.lidar.isaac",
            sensor_class=LidarIsaacSensor,
            tags=["sensor", "lidar", "isaac"],
        )

    def _define_attributes(self):
        """Define Isaac LiDAR attributes"""
        # Isaac Sim specific: config file name
        self.set_attribute("config_file_name", "Hesai_XT32_SD10")

        # Sensor tick (0 = every frame)
        self.set_attribute("sensor_tick", 0.0)

        # Frequency (Hz)
        self.set_attribute("frequency", 10)


class OmniLidarBlueprint(SensorBlueprint):
    """
    Omni LiDAR Blueprint (CARLA style)

    Blueprint for sensor.lidar.omni
    Uses Omni's RTX LiDAR implementation
    """

    def __init__(self):
        from simulation.sensor.lidar_actor import LidarOmniSensor

        super().__init__(
            blueprint_id="sensor.lidar.omni",
            sensor_class=LidarOmniSensor,
            tags=["sensor", "lidar", "omni"],
        )

    def _define_attributes(self):
        """Define Omni LiDAR attributes"""
        # Isaac Sim specific: config file name
        self.set_attribute("config_file_name", "Hesai_XT32_SD10")

        # Sensor tick (0 = every frame)
        self.set_attribute("sensor_tick", 0.0)

        # Equirectangular projection parameters
        # 注意: output_size 必须与 (erp_height, erp_width) 一致
        self.set_attribute("erp_width", 120)
        self.set_attribute("erp_height", 352)
        self.set_attribute("output_size", (352, 120))

        # FOV parameters
        self.set_attribute("erp_width_fov", 90.0)
        self.set_attribute("erp_height_fov", 270.0)

        # Max depth (meters)
        self.set_attribute("max_depth", 1000.0)

        # Frequency (Hz)
        self.set_attribute("frequency", 10)
