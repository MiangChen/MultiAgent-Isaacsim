from typing import List, Dict, Optional


class ActorBlueprint:
    """
    Actor Blueprint (CARLA style)
    
    Base class for all actor blueprints (robots, sensors, static props, etc.)
    """
    
    def __init__(self, blueprint_id: str, robot_class=None, tags: List[str] = None):
        self.id = blueprint_id
        self.robot_class = robot_class
        self.tags = tags or []
        self._attributes: Dict[str, any] = {}

    def set_attribute(self, key: str, value):
        self._attributes[key] = value
        return self

    def get_attribute(self, key: str, default=None):
        return self._attributes.get(key, default)

    def get_all_attributes(self) -> Dict:
        return self._attributes.copy()

    def has_tag(self, tag: str) -> bool:
        return tag in self.tags


# Alias for backward compatibility
Blueprint = ActorBlueprint


class BlueprintLibrary:
    def __init__(self):
        self._blueprints: Dict[str, ActorBlueprint] = {}
        self._register_default_robots()

    def _register_default_robots(self):
        """预注册所有已知的机器人类型 Pre-register all known robot types (CARLA style)"""
        from robot.robot_jetbot import RobotJetbot
        from robot.robot_h1 import RobotH1
        from robot.robot_g1 import RobotG1
        from robot.robot_drone_cf2x import RobotCf2x
        from robot.robot_drone_autel import RobotDroneAutel
        from robot.target import Target

        self.register_robot_class("jetbot", RobotJetbot)
        self.register_robot_class("h1", RobotH1)
        self.register_robot_class("g1", RobotG1)
        self.register_robot_class("cf2x", RobotCf2x)
        self.register_robot_class("drone_autel", RobotDroneAutel)
        self.register_robot_class("target", Target)

        # Register static props (CARLA style: static.prop.*)
        self._register_static_props()
        
        # Register sensors (CARLA style: sensor.*)
        self._register_sensors()

    def _register_static_props(self):
        """注册静态物体 Register static props (CARLA style)"""
        # Box
        bp = ActorBlueprint("static.prop.box", robot_class=None, tags=["static", "prop"])
        bp.set_attribute("shape_type", "cuboid")
        bp.set_attribute("entity_type", "visual")
        self._blueprints[bp.id] = bp

        # Car
        bp = ActorBlueprint(
            "static.prop.car", robot_class=None, tags=["static", "prop", "vehicle"]
        )
        bp.set_attribute("shape_type", "cuboid")
        bp.set_attribute("entity_type", "visual")
        self._blueprints[bp.id] = bp
        
    def _register_sensors(self):
        """注册传感器 Register sensors (CARLA style)"""
        from simulation.sensors.camera.camera_blueprint import (
            RGBCameraBlueprint,
            DepthCameraBlueprint,
        )
        from simulation.sensors.lidar.lidar_blueprint import (
            RayCastLidarBlueprint,
        )
        
        # RGB Camera
        self._blueprints['sensor.camera.rgb'] = RGBCameraBlueprint()
        
        # Depth Camera
        self._blueprints['sensor.camera.depth'] = DepthCameraBlueprint()
        
        # LiDAR
        self._blueprints['sensor.lidar.ray_cast'] = RayCastLidarBlueprint()

    def register_robot_class(self, robot_type: str, robot_class: type):
        tags = ["robot"]
        if robot_type in ["h1", "g1"]:
            tags.append("humanoid")
        elif robot_type in ["jetbot"]:
            tags.append("wheeled")
        elif robot_type in ["cf2x", "drone_autel"]:
            tags.append("drone")

        bp = ActorBlueprint(f"robot.{robot_type}", robot_class=robot_class, tags=tags)
        self._blueprints[bp.id] = bp

    def find(self, blueprint_id: str) -> Optional[ActorBlueprint]:
        return self._blueprints.get(blueprint_id)

    def filter(self, wildcard: str) -> List[ActorBlueprint]:
        import fnmatch

        return [
            bp
            for bp_id, bp in self._blueprints.items()
            if fnmatch.fnmatch(bp_id, wildcard)
        ]
