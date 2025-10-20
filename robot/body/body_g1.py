from typing import Tuple

import torch

from physics_engine.isaacsim_utils import (
    Scene,
    Articulation,
    define_prim,
    get_prim_at_path,
)
from pxr import Usd, UsdGeom

from robot.body import BodyRobot
from robot.cfg import CfgRobot
from utils import to_torch
from log.log_manager import LogManager

logger = LogManager.get_logger(__name__)


class BodyG1(BodyRobot):
    def __init__(
        self,
        cfg_robot: CfgRobot = None,
        scene: Scene = None,
    ):
        super().__init__(cfg_robot=cfg_robot, scene=scene)
        self.create_robot_body()

    def create_robot_body(self):
        """
        初始化机器人关节树
        """

        prim = get_prim_at_path(self.cfg_robot.path_prim_robot)
        if not prim.IsValid():
            prim = define_prim(self.cfg_robot.path_prim_robot, "Xform")
            if self.cfg_robot.path_usd:
                # load USD model
                prim.GetReferences().AddReference(self.cfg_robot.path_usd)
            else:
                logger.error("unable to add robot usd, path_usd not provided")
        elif prim.IsA(UsdGeom.Xformable):
            # Convert the prim to an Xformable object
            xformable = UsdGeom.Xformable(prim)

            # Get the local-to-world transformation matrix
            timecode = Usd.TimeCode.Default()  # Use default time or simulation time
            local_to_world_matrix = xformable.ComputeLocalToWorldTransform(timecode)

            # The local_to_world_matrix is a Gf.Matrix4d
            # Extract the translation (position) from the matrix
            self.cfg_robot.position = list(
                local_to_world_matrix.ExtractTranslation()
            )  # Returns a Gf.Vec3d

            # Extract the rotation from the matrix
            quat = local_to_world_matrix.ExtractRotationQuat()  # Returns a Gf.Quatd
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
