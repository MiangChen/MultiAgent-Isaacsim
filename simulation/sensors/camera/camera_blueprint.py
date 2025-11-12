"""
Camera Blueprints - CARLA Style

Blueprint definitions for camera sensors (RGB, Depth, etc.)
"""

from simulation.sensors.sensor_blueprint_base import SensorBlueprint


class RGBCameraBlueprint(SensorBlueprint):
    """
    RGB Camera Blueprint (CARLA style)

    Blueprint for sensor.camera.rgb
    """

    def __init__(self):
        from simulation.sensors.camera_actor import RGBCamera

        super().__init__(
            blueprint_id="sensor.camera.rgb",
            sensor_class=RGBCamera,
            tags=["sensor", "camera", "rgb"],
        )

    def _define_attributes(self):
        """Define camera attributes (CARLA standard)"""
        # Resolution
        self.set_attribute("image_size_x", 800)
        self.set_attribute("image_size_y", 600)

        # Field of view
        self.set_attribute("fov", 90.0)

        # Focal length (mm)
        self.set_attribute("focal_length", 20.0)

        # Sensor tick (0 = every frame)
        self.set_attribute("sensor_tick", 0.0)

        # Post-processing
        self.set_attribute("enable_postprocess_effects", False)

        # Semantic detection
        self.set_attribute("enable_semantic_detection", False)


class DepthCameraBlueprint(SensorBlueprint):
    """
    Depth Camera Blueprint (CARLA style)

    Blueprint for sensor.camera.depth
    """

    def __init__(self):
        from simulation.sensors.camera_actor import (
            RGBCamera,
        )  # TODO: Create DepthCamera

        super().__init__(
            blueprint_id="sensor.camera.depth",
            sensor_class=RGBCamera,  # Placeholder
            tags=["sensor", "camera", "depth"],
        )

    def _define_attributes(self):
        """Define depth camera attributes"""
        self.set_attribute("image_size_x", 800)
        self.set_attribute("image_size_y", 600)
        self.set_attribute("fov", 90.0)
