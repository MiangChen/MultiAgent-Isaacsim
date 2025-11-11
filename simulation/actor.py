from typing import TYPE_CHECKING
from simulation.transform import Transform, Location, Vector3D

if TYPE_CHECKING:
    from simulation.world import World


class Actor:

    def __init__(self, world: "World", prim_path: str):
        self._world = world
        self._prim_path = prim_path
        self._actor_id = world.register_actor(self)
        self._prim = None

    def get_id(self) -> int:
        return self._actor_id

    def get_type_id(self) -> str:
        return "actor.base"

    def get_transform(self) -> Transform:
        raise NotImplementedError("Subclass must implement get_transform()")

    def set_transform(self, transform: Transform):
        raise NotImplementedError("Subclass must implement set_transform()")

    def get_location(self) -> Location:
        return self.get_transform().location

    def set_location(self, location: Location):
        transform = self.get_transform()
        transform.location = location
        self.set_transform(transform)

    def get_velocity(self) -> Vector3D:
        raise NotImplementedError("Subclass must implement get_velocity()")

    def destroy(self):
        self._world.unregister_actor(self._actor_id)

    def get_prim_path(self) -> str:
        return self._prim_path

    def __repr__(self):
        return f"Actor(id={self._actor_id}, type={self.get_type_id()}, path={self._prim_path})"
