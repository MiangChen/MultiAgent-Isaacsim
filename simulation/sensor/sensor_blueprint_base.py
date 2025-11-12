"""
Sensor Blueprint Base Class

Provides common functionality for all sensor blueprints
"""

from simulation.actor_blueprint import ActorBlueprint


class SensorBlueprint(ActorBlueprint):
    """
    Base class for sensor blueprints

    Reduces code duplication by providing common initialization
    """

    def __init__(self, blueprint_id: str, sensor_class, tags: list):
        """
        Initialize sensor blueprint

        Args:
            blueprint_id: Blueprint ID (e.g., 'sensor.camera.rgb')
            sensor_class: Sensor actor class (e.g., RGBCamera)
            tags: Tags list (e.g., ['sensor', 'camera', 'rgb'])
        """
        super().__init__(blueprint_id, robot_class=None, tags=tags)
        self.sensor_class = sensor_class
        self._define_attributes()

    def _define_attributes(self):
        """
        Define sensor attributes

        Override in subclasses to set specific attributes
        """
        pass
