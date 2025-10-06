import carb
from isaacsim.core.api.scenes import Scene
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.prims import define_prim, get_prim_at_path
from pxr import Usd, UsdGeom

from robot.robot_entity.robot_entity import RobotEntity, logger
from robot.robot_cfg import RobotCfg
from utils import to_torch


class EntityH1(RobotEntity):
    def __init__(
        self,
        cfg_articulation: RobotCfg = None,
        scene: Scene = None,
    ):
        super().__init__(cfg_articulation=cfg_articulation, scene=scene)

    def create_robot_entity(self):
        """
        初始化机器人关节树
        """
        self.cfg_articulation.prim_path = (
            self.cfg_articulation.prim_path
            + f"/{self.cfg_articulation.type}"
            + f"/{self.cfg_articulation.type}_{self.cfg_articulation.id}"
        )
        self.cfg_articulation.name = self.cfg_articulation.type + f"_{self.cfg_articulation.id}"
        self.name = self.cfg_articulation.name

        prim = get_prim_at_path(self.cfg_articulation.prim_path)
        if not prim.IsValid():
            prim = define_prim(self.cfg_articulation.prim_path, "Xform")
            if self.cfg_articulation.usd_path:
                prim.GetReferences().AddReference(
                    self.cfg_articulation.usd_path
                )  # 加载机器人USD模型
            else:
                carb.log_error("unable to add robot usd, usd_path not provided")
        elif prim.IsA(UsdGeom.Xformable):
            # Convert the prim to an Xformable object
            xformable = UsdGeom.Xformable(prim)

            # Get the local-to-world transformation matrix
            # CORRECTED METHOD NAME: ComputeLocalToWorldTransform
            # You need to specify a time code.
            timecode = Usd.TimeCode.Default()  # Use default time or simulation time
            local_to_world_matrix = xformable.ComputeLocalToWorldTransform(timecode)

            # The local_to_world_matrix is a Gf.Matrix4d
            # Extract the translation (position) from the matrix
            self.cfg_articulation.position = list(
                local_to_world_matrix.ExtractTranslation()
            )  # Returns a Gf.Vec3d

            # Extract the rotation from the matrix
            quat = local_to_world_matrix.ExtractRotationQuat()  # Returns a Gf.Quatd
            self.cfg_articulation.quat = [quat.real] + list(quat.imaginary)
        else:
            if prim:
                logger.info(
                    f"Prim at {prim.GetPath()} is not Xformable or does not exist."
                )
            else:
                logger.info(f"Prim not found at path {prim.GetPath()}")

        self.robot_articulation = Articulation(
            prim_paths_expr=self.cfg_articulation.prim_path,
            name=self.cfg_articulation.name,
            positions=to_torch(self.cfg_articulation.position).reshape(1, 3),
            orientations=to_torch(self.cfg_articulation.quat).reshape(1, 4),
        )
