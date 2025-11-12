from log.log_manager import LogManager
from physics_engine.isaacsim_utils import (
    Scene,
    Articulation,
    define_prim,
    get_prim_at_path,
)
from physics_engine.pxr_utils import Usd, UsdGeom
from simulation.robot.body import BodyRobot
from simulation.robot.cfg import CfgRobot
from utils import to_torch

logger = LogManager.get_logger(__name__)


class BodyDroneAutel(BodyRobot):
    def __init__(self, cfg_robot: CfgRobot = None):
        super().__init__(cfg_robot=cfg_robot)
        self.create_robot_body()

    def create_robot_body(self):
        prim = get_prim_at_path(self.cfg_robot.path_prim_robot)
        if not prim.IsValid():
            prim = define_prim(self.cfg_robot.path_prim_robot, "Xform")
            if self.cfg_robot.path_usd:
                prim.GetReferences().AddReference(self.cfg_robot.path_usd)
            else:
                logger.error("unable to add robot usd, path_usd not provided")
        elif prim.IsA(UsdGeom.Xformable):
            xformable = UsdGeom.Xformable(prim)
            timecode = Usd.TimeCode.Default()
            local_to_world_matrix = xformable.ComputeLocalToWorldTransform(timecode)
            self.cfg_robot.position = list(local_to_world_matrix.ExtractTranslation())
            quat = local_to_world_matrix.ExtractRotationQuat()
            self.cfg_robot.quat = [quat.real] + list(quat.imaginary)
        else:
            if prim:
                logger.info(
                    f"Prim at {prim.GetPath()} is not Xformable or does not exist."
                )
            else:
                logger.info(f"Prim not found at path {prim.GetPath()}")

        self.robot_articulation = Articulation(
            prim_paths_expr=self.cfg_robot.path_prim_robot,
            name=self.cfg_robot.namespace,
            positions=to_torch(self.cfg_robot.position).reshape(1, 3),
            orientations=to_torch(self.cfg_robot.quat).reshape(1, 4),
        )
