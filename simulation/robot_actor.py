from simulation.actor import Actor
from simulation.transform import Transform, Location, Vector3D, Rotation


class RobotActor(Actor):
    def __init__(self, robot, world=None):
        self.robot = robot
        self._world = world
        self._prim_path = robot.cfg_robot.path_prim_robot
        self._actor_id = world.register_actor(self) if world else None

        # 双向引用 Bidirectional reference: Robot <-> RobotActor
        robot.actor = self

        self._setup_physics_properties()

    def get_type_id(self) -> str:
        return f"robot.{self.robot.cfg_robot.type}"

    def get_transform(self) -> Transform:
        pos, quat = self.robot.get_world_pose()  # tensor
        return Transform(location=pos, rotation=quat, order="wxyz")

    def set_transform(self, transform: Transform):
        if transform.location is not None:
            self.robot.set_world_pose(
                position=[
                    transform.location.x,
                    transform.location.y,
                    transform.location.z,
                ]
            )

    def get_velocity(self) -> Vector3D:
        vel = self.robot.get_linear_velocity()
        return Vector3D(vel[0].item(), vel[1].item(), vel[2].item())

    def _setup_physics_properties(self):
        """
        设置物理属性（如禁用重力）
        在 robot._body 创建完成后调用
        """
        if hasattr(self.robot, "cfg_robot") and self.robot.cfg_robot.disable_gravity:
            self._disable_gravity_for_hierarchy(self._prim_path)

    def _disable_gravity_for_hierarchy(self, root_prim_path: str):
        """
        递归地遍历指定路径下的所有Prim，并禁用任何找到的刚体的重力。

        这是处理从外部文件加载的、作为静态背景的场景的最佳方法。

        Args:
            root_prim_path (str): 您加载的USD场景的根路径 (e.g., "/World/MyBuilding").

        Returns:
            dict: 操作结果的状态字典，包含修改的Prim数量。
        """
        try:
            from physics_engine.omni_utils import omni
            from physics_engine.pxr_utils import Usd, PhysxSchema

            stage = omni.usd.get_context().get_stage()
            if not stage:
                return {"status": "error", "message": "No active USD stage."}

            root_prim = stage.GetPrimAtPath(root_prim_path)
            if not root_prim.IsValid():
                return {
                    "status": "error",
                    "message": f"Root prim '{root_prim_path}' not found.",
                }

            print(
                f"INFO: Starting to disable gravity for all rigid bodies under '{root_prim_path}'..."
            )

            modified_prims_count = 0

            # Usd.PrimRange 是一个高效的、可以遍历所有子孙节点的迭代器
            for prim in Usd.PrimRange(root_prim):

                # 1. 检查这个 Prim 是否是一个物理刚体
                if True:
                    # if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                    # 2. 应用 PhysX 专有的 API 以访问 'disableGravity' 属性
                    physx_rigid_body_api = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)

                    # 3. 获取 'disableGravity' 属性并将其设置为 True
                    #    (我们之前已经通过 dir() 验证了这个属性的存在)
                    disable_gravity_attr = physx_rigid_body_api.GetDisableGravityAttr()
                    disable_gravity_attr.Set(True)

                    modified_prims_count += 1
                    # print(f"  - Disabled gravity for: {prim.GetPath()}")

            print(
                f"INFO: Gravity disabled for a total of {modified_prims_count} rigid bodies."
            )
            return {
                "status": "success",
                "message": f"Disabled gravity for {modified_prims_count} prims under '{root_prim_path}'.",
                "modified_count": modified_prims_count,
            }

        except Exception as e:
            import traceback

            traceback.print_exc()
            print(f" Failed to disable gravity for {root_prim_path}, {e}")
            return {
                "status": "error",
                "message": f"An unexpected error occurred: {str(e)}",
            }
