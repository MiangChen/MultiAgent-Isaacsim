from simulation.actor import Actor
from simulation.transform import Transform, Location, Vector3D


class RobotActor(Actor):
    def __init__(self, robot, world=None):
        self.robot = robot
        self._world = world
        self._prim_path = robot.cfg_robot.path_prim_robot
        self._actor_id = world.register_actor(self) if world else None
        
        # 双向引用 Bidirectional reference: Robot <-> RobotActor
        robot.actor = self
    
    def get_type_id(self) -> str:
        return f"robot.{self.robot.cfg_robot.type}"
    
    def get_transform(self) -> Transform:
        pos, quat = self.robot.body.get_world_pose()
        return Transform(
            location=Location(pos[0].item(), pos[1].item(), pos[2].item())
        )
    
    def set_transform(self, transform: Transform):
        self.robot.body.robot_articulation.set_world_pose(
            position=[transform.location.x, transform.location.y, transform.location.z]
        )
    
    def get_velocity(self) -> Vector3D:
        vel = self.robot.vel_linear
        return Vector3D(vel[0].item(), vel[1].item(), vel[2].item())
