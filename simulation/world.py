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
        self._swarm_manager = None
        self._grid_map = None
    
    def tick(self):
        self._isaac_world.step(render=True)
    
    def reset(self):
        self._isaac_world.reset()
    
    def get_actors(self) -> List['Actor']:
        return list(self._actors.values())
    
    def get_actor(self, actor_id: int) -> Optional['Actor']:
        return self._actors.get(actor_id)
    
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
    
    # def get_blueprint_library(self):
    #     if self._blueprint_library is None:
    #         from simulation.blueprint import BlueprintLibrary
    #         self._blueprint_library = BlueprintLibrary()
    #     return self._blueprint_library
    
    def set_scene_manager(self, scene_manager):
        self._scene_manager = scene_manager
    
    def set_swarm_manager(self, swarm_manager):
        self._swarm_manager = swarm_manager
    
    def set_grid_map(self, grid_map):
        self._grid_map = grid_map
    
    def get_scene_manager(self):
        return self._scene_manager
    
    def get_swarm_manager(self):
        return self._swarm_manager
    
    def get_grid_map(self):
        return self._grid_map
    
    def initialize_robots(self):
        if self._swarm_manager is None:
            return
        for robot_class in self._swarm_manager.robot_warehouse.keys():
            for robot in self._swarm_manager.robot_warehouse[robot_class]:
                robot.initialize()
    
    def initialize_map(self):
        if self._grid_map is not None:
            self._grid_map.initialize()
