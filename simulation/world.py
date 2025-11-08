from typing import List, Optional, Dict, TYPE_CHECKING
from physics_engine.isaacsim_utils import World as IsaacWorld

if TYPE_CHECKING:
    from simulation.actor import Actor


class World:
    
    def __init__(
        self, 
        simulation_app,
        physics_dt: float = 1.0/60.0,
        rendering_dt: float = 1.0/60.0,
        stage_units_in_meters: float = 1.0,
        sim_params: dict = None,
        backend: str = "torch"
    ):
        self._simulation_app = simulation_app
        self._isaac_world = IsaacWorld(
            physics_dt=physics_dt,
            rendering_dt=rendering_dt,
            stage_units_in_meters=stage_units_in_meters,
            sim_params=sim_params,
            backend=backend
        )
        self._actors: Dict[int, 'Actor'] = {}
        self._next_actor_id = 1
        self._blueprint_library = None
        self._scene_manager = None
        self._semantic_map = None
        self._grid_map = None
    
    def tick(self):
        self._isaac_world.step(render=True)
    
    def reset(self):
        self._isaac_world.reset()
    
    def get_actors(self) -> List['Actor']:
        return list(self._actors.values())
    
    def get_actor(self, actor_id: int) -> Optional['Actor']:
        return self._actors.get(actor_id)
    
    def find_actor_by_robot(self, robot) -> Optional['Actor']:
        """通过 Robot 实例查找对应的 Actor"""
        return getattr(robot, 'actor', None)
    
    def spawn_actor(self, blueprint, transform=None, attach_to=None):
        if blueprint.robot_class is None:
            raise ValueError(f"Blueprint {blueprint.id} has no robot_class")
        
        cfg_robot = blueprint.get_all_attributes()
        
        if transform is not None:
            cfg_robot['position'] = transform.location.to_list()
            cfg_robot['orientation'] = transform.rotation.to_quaternion()
        
        robot = blueprint.robot_class(cfg_robot=cfg_robot, scene_manager=self._scene_manager)
        
        self.scene.add(robot.body.robot_articulation)
        
        if self._semantic_map:
            self._semantic_map.dict_map_semantic[robot.cfg_robot.namespace] = robot.cfg_robot.path_prim_robot
            self._semantic_map.add_semantic(prim_path=robot.cfg_robot.path_prim_robot, semantic_label="robot")
        
        from simulation.robot_actor import RobotActor
        RobotActor(robot, world=self)
        
        return robot
    
    def get_blueprint_library(self):
        if self._blueprint_library is None:
            from simulation.blueprint import BlueprintLibrary
            self._blueprint_library = BlueprintLibrary()
        return self._blueprint_library
    
    def load_actors_from_config(self, config_path: str) -> List:
        import yaml
        
        with open(config_path, 'r') as file:
            config = yaml.safe_load(file)
        
        robots = []
        blueprint_library = self.get_blueprint_library()
        
        for robot_type, robot_configs in config.items():
            bp = blueprint_library.find(f'robot.{robot_type}')
            if not bp:
                raise ValueError(f"Unknown robot type: {robot_type}")
            
            for cfg in robot_configs:
                for key, value in cfg.items():
                    bp.set_attribute(key, value)
                robots.append(self.spawn_actor(bp))
        
        return robots
    
    def register_actor(self, actor: 'Actor') -> int:
        actor_id = self._next_actor_id
        self._next_actor_id += 1
        self._actors[actor_id] = actor
        return actor_id
    
    def unregister_actor(self, actor_id: int):
        if actor_id in self._actors:
            del self._actors[actor_id]
    
    def get_isaac_world(self):
        return self._isaac_world
    
    @property
    def scene(self):
        return self._isaac_world.scene
    
    def is_playing(self) -> bool:
        return self._isaac_world.is_playing()
    
    def play(self):
        self._isaac_world.play()
    
    def pause(self):
        self._isaac_world.pause()
    
    def stop(self):
        self._isaac_world.stop()
    
    def set_scene_manager(self, scene_manager):
        self._scene_manager = scene_manager
    
    def set_semantic_map(self, semantic_map):
        self._semantic_map = semantic_map
    
    def set_grid_map(self, grid_map):
        self._grid_map = grid_map
    
    def get_scene_manager(self):
        return self._scene_manager
    
    def get_semantic_map(self):
        return self._semantic_map
    
    def get_grid_map(self):
        return self._grid_map
    
    def initialize_robots(self):
        for actor in self.get_actors():
            if hasattr(actor, 'robot') and hasattr(actor.robot, 'initialize'):
                actor.robot.initialize()
    
    def initialize_map(self):
        if self._grid_map:
            self._grid_map.initialize()
