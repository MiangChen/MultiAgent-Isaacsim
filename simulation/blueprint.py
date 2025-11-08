from typing import List, Dict, Optional


class Blueprint:
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


class BlueprintLibrary:
    def __init__(self):
        self._blueprints: Dict[str, Blueprint] = {}
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
        self.register_robot_class("autel", RobotDroneAutel)
        self.register_robot_class("target", Target)
    
    def register_robot_class(self, robot_type: str, robot_class: type):
        tags = ['robot']
        if robot_type in ['h1', 'g1']:
            tags.append('humanoid')
        elif robot_type in ['jetbot']:
            tags.append('wheeled')
        elif robot_type in ['cf2x', 'autel']:
            tags.append('drone')
        
        bp = Blueprint(f'robot.{robot_type}', robot_class=robot_class, tags=tags)
        self._blueprints[bp.id] = bp
    
    def find(self, blueprint_id: str) -> Optional[Blueprint]:
        return self._blueprints.get(blueprint_id)
    
    def filter(self, wildcard: str) -> List[Blueprint]:
        import fnmatch
        return [bp for bp_id, bp in self._blueprints.items() if fnmatch.fnmatch(bp_id, wildcard)]
