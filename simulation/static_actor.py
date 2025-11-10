from simulation.actor import Actor
from simulation.transform import Transform, Location, Vector3D
from physics_engine.isaacsim_utils import get_prim_at_path
from pxr import UsdGeom, Gf


class StaticActor(Actor):
    """Static prop actor (CARLA style)"""
    
    def __init__(self, prim_path: str, world=None, semantic_label: str = None):
        self._world = world
        self._prim_path = prim_path
        self._semantic_label = semantic_label
        self._actor_id = world.register_actor(self) if world else None
        self._prim = get_prim_at_path(prim_path)
    
    def get_type_id(self) -> str:
        return f"static.prop.{self._semantic_label or 'unknown'}"
    
    def get_transform(self) -> Transform:
        """Get current transform of the static prop"""
        if not self._prim or not self._prim.IsValid():
            return Transform()
        
        xformable = UsdGeom.Xformable(self._prim)
        if not xformable:
            return Transform()
        
        # Get world transform
        world_transform = xformable.ComputeLocalToWorldTransform(0)
        translation = world_transform.ExtractTranslation()
        
        return Transform(
            location=Location(translation[0], translation[1], translation[2])
        )
    
    def set_transform(self, transform: Transform):
        """Set transform of the static prop"""
        if not self._prim or not self._prim.IsValid():
            return
        
        xformable = UsdGeom.Xformable(self._prim)
        if not xformable:
            return
        
        # Set translation
        translate_op = xformable.AddTranslateOp()
        translate_op.Set(Gf.Vec3d(
            transform.location.x,
            transform.location.y,
            transform.location.z
        ))
    
    def get_velocity(self) -> Vector3D:
        """Static props have zero velocity"""
        return Vector3D(0, 0, 0)
    
    def get_semantic_label(self) -> str:
        return self._semantic_label
