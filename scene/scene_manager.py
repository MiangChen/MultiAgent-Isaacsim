# =============================================================================
# Scene Manager Module - Isaac Sim Backend
# =============================================================================
#
# 这个模块是Isaac Sim专用的场景管理器，直接使用Isaac Sim API。
# 它作为simulation层的后端实现，提供场景操作的底层功能。
#
# 使用规则：
# - 用户代码应该使用 simulation.World 提供的CARLA风格API
# - simulation/ 内部可以调用 scene_manager 的方法
# - scene_manager 可以自由使用 Isaac Sim API，不需要过度封装
#
# 职责：
# - 场景加载和管理（load_scene, save_scene）
# - 物体创建（create_shape_unified, create_robot）
# - 相机和传感器管理（add_camera）
# - 碰撞检测（check_prim_collision）
# - 语义标签管理（通过semantic_map）
# - USD prim操作（adjust_prim, adjust_pose）
#
# =============================================================================

# Standard library imports
import traceback
from pathlib import Path
from typing import Dict, Any, List, Union, Optional, Sequence

# Third-party library imports
import numpy as np
import torch
import carb

# Local project imports
from config.config_manager import config_manager
from physics_engine.isaacsim_utils import (
    cuboid,
    sphere,
    XFormPrim,
    World,
    add_update_semantics,
    remove_all_semantics,
    count_semantics_in_scene,
    Camera,
    define_prim,
    get_prim_at_path,
    add_reference_to_stage,
    get_stage_units,
    Articulation,
    RigidPrim,
    create_prim,
)
from physics_engine.omni_utils import omni
from physics_engine.pxr_utils import Gf, Sdf, UsdGeom, UsdPhysics, PhysxSchema, Usd

ASSET_PATH = config_manager.get("path_asset")


class SceneManager:
    """
    Scene Manager - Isaac Sim Backend
    
    这个模块是Isaac Sim专用的场景管理器，直接使用Isaac Sim API。
    它作为simulation层的后端实现，不应该被用户代码直接调用。
    
    用户代码应该使用 simulation.World 提供的CARLA风格API。
    
    职责：
    - 场景加载和管理
    - 物体创建（使用Isaac Sim原生API）
    - 相机和传感器管理
    - 碰撞检测
    - 语义标签管理
    """

    def __init__(self, world):
        self.prim_info = []
        self.stage = omni.usd.get_context().get_stage()
        self.world = world
        self.scene_repository_path = "./scenes"

    def delete_prim(self, prim_path: str):
        """
        删除指定路径的prim
        """
        try:
            # 检查Prim是否存在
            prim_to_delete = self.stage.GetPrimAtPath(prim_path)
            if not prim_to_delete.IsValid():
                return {
                    "status": "skipped",
                    "message": f"Prim at '{prim_path}' does not exist. Nothing to delete.",
                }

            if self.world.is_playing():
                self.world.pause()
            # self.stage.RemovePrim(Sdf.Path(prim_path)) # 不可以直接删除, 容易导致各种为难题, 建议使用deactive

            return {
                "status": "success",
                "message": f"Successfully deleted {prim_path}.",
            }

        except Exception as e:
            import traceback

            traceback.print_exc()
            return {
                "status": "error",
                "message": f"An error occurred while deleting {prim_path}: {str(e)}",
            }
        finally:
            # 确保在操作后恢复物理引擎的运行状态
            self.world.play()

    def active_prim(self, prim_path: str, active: bool = True):
        try:
            prim = self.stage.GetPrimAtPath(prim_path)

            if not prim.IsValid():
                return {
                    "status": "not_found",
                    "message": f"Prim '{prim_path}' 不存在，无法操作。",
                }

            prim.SetActive(active)

            status_str = "active" if active else "deactive"
            return {
                "status": "success",
                "message": f"成功 {status_str} Prim '{prim_path}'。",
            }

        except Exception as e:
            return {
                "status": "error",
                "message": f"操作Prim '{prim_path}' 时发生错误: {e}",
            }

    def get_scene_info(self, max_depth: int = 2) -> Dict[str, Any]:
        try:
            # 清空之前的信息
            self.prim_info = []

            for prim in self.stage.Traverse():
                try:
                    # 计算prim的层级深度
                    prim_path = str(prim.GetPath().pathString)
                    # 计算路径深度：去掉开头的'/'，然后按'/'分割计算层数
                    if prim_path == "/":
                        depth = 0
                    else:
                        depth = prim_path.strip("/").count("/") + 1

                    # 如果超过最大深度限制，跳过此prim
                    if depth > max_depth:
                        continue
                    # 获取世界变换矩阵并转换为可序列化的格式
                    world_transform = omni.usd.get_world_transform_matrix(prim)
                    transform_list = None
                    if world_transform is not None:
                        try:
                            # 将Matrix4d对象转换为嵌套列表
                            if hasattr(world_transform, "__iter__") and hasattr(
                                world_transform, "__getitem__"
                            ):
                                transform_list = []
                                for i in range(4):
                                    row = []
                                    for j in range(4):
                                        try:
                                            row.append(float(world_transform[i][j]))
                                        except:
                                            row.append(0.0)
                                    transform_list.append(row)
                            else:
                                # 如果无法访问矩阵元素，设为None
                                transform_list = None
                        except Exception as e:
                            print(
                                f"[Scene Info] Failed to convert transform matrix for {prim.GetPath()}: {e}"
                            )
                            transform_list = None

                    # 获取文档信息，确保是字符串格式
                    doc = prim.GetMetadata("doc")
                    doc_str = str(doc) if doc is not None else None

                    # 暂时跳过world_transform以避免序列化问题
                    # 可以后续单独提供获取变换矩阵的方法

                    info = {
                        "path": str(prim.GetPath().pathString),
                        "name": str(prim.GetName()),
                        "type": str(prim.GetTypeName()),
                        "is_active": bool(prim.IsActive()),
                        "is_defined": bool(prim.IsDefined()),
                        "depth": depth,  # 添加深度信息
                        # "world_transform": transform_list,  # 暂时注释掉
                        "doc": doc_str,
                    }
                    self.prim_info.append(info)
                except Exception as e:
                    print(
                        f"[Scene Info] Failed to extract info for prim: {prim.GetPath()} — {e}"
                    )
                    continue

            return {"status": "success", "result": self.prim_info}
        except Exception as e:
            print(f"[Scene Info] Unexpected error while getting prims: {e}")
            return {"status": "error", "message": str(e)}

    def get_selected_prim(self) -> Dict[str, str | Any]:
        ctx = omni.usd.get_context()
        selected_paths = ctx.get_selection().get_selected_prim_paths()
        return {"status": "success", "result": selected_paths}

    def check_prim_collision(self, prim_path: str = None, radius: float = None) -> bool:
        """Check collision for a drone at the specified prim path.

        Args:
            prim_path: Path to the object prim.

        Returns:
            bool: True if collision detected, False otherwise
        """
        radius = 0.5  # TODO(Kartik): Get radius from the drone model

        prim = self.stage.GetPrimAtPath(prim_path)
        if not prim.IsValid():
            print(f"Drone prim {prim_path} is not valid")
            return False

        origin = prim.GetAttribute("xformOp:translate").Get()
        if origin is None:
            print(f"Drone origin {prim_path} is not valid")
            return False

        # physX query to detect hits for a sphere
        ret = omni.physx.get_physx_scene_query_interface().overlap_sphere_any(
            radius, carb.Float3(origin[0], origin[1], origin[2])
        )
        if ret:
            print(f"WARNING: Collision detected for drone {prim_path}")
        return ret

    def check_prim_overlap(
        self, position: list, threshold: float = 0.1
    ) -> Dict[str, Any]:
        """
        检查给定位置附近是否已有 prim，避免重复创建

        :param position: [x, y, z] 世界坐标列表
        :param threshold: 半径阈值（与舞台单位一致）
        :return: 如果检测到重叠则返回 True，否则 False
        """
        # 将列表转为 Vec3f
        pos_vec = Gf.Vec3f(*position)

        for prim in self.stage.Traverse():
            if not prim.IsValid():
                continue

            xformable = UsdGeom.Xformable(prim)
            if not xformable:
                continue

            try:
                for op in xformable.GetOrderedXformOps():
                    # 只检查平移或整体变换
                    if op.GetOpType() in (
                        UsdGeom.XformOp.TypeTranslate,
                        UsdGeom.XformOp.TypeTransform,
                    ):
                        existing_pos = Gf.Vec3f(op.Get())
                        if (existing_pos - pos_vec).GetLength() < threshold:
                            return {
                                "status": "error",
                                "message": f"Prim already exists near position {position}",
                            }
            except Exception:
                continue
        return {
            "status": "success",
            "message": "No overlapping prims found at the specified position.",
        }

    def overlap_hits_target_ancestor(
        self, radius_cm: float = 500.0, pos=None, target_prim: str = None
    ) -> bool:
        """
        用一个半径=radius_cm 的球做 Overlap 检测。

        若任意命中的刚体/碰撞体 prim 的路径处于 target_prim 子树下（或本身），返回 True，否则 False。
        - radius_cm: 浮点半径（与场景单位一致；Isaac 文档示例以 cm 说明）
        - pos3: 3D 位置（tensor/numpy/list/tuple 都可）
        - target_prim: 要判定的祖先 prim 路径（如 "/World/MyRobot"）
        需要保证：仿真处于运行状态，且相关物体具备 collider。
        """
        tx = float(pos[0])
        ty = float(pos[1])
        tz = float(pos[2])
        origin = carb.Float3(tx, ty, tz)
        target = target_prim.rstrip("/")

        # 允许 target_prim 是 rigid_body/collision 的祖先或本身
        def _is_ancestor(hit_path: str) -> bool:
            if not hit_path:
                return False
            hp = hit_path.rstrip("/")
            return hp == target or hp.startswith(target + "/")

        # 回调：一旦发现命中属于 target 子树，立即早停
        found = {"v": False}

        def _report(hit) -> bool:
            # OverlapHit 暴露 rigid_body / collision 的 USD 路径（Python 绑定）
            rb = getattr(hit, "rigid_body", None) or getattr(hit, "rigidBody", None)
            col = getattr(hit, "collision", None) or getattr(
                hit, "collision_path", None
            )
            if _is_ancestor(rb) or _is_ancestor(col):
                found["v"] = True
                return False  # 早停
            return True  # 继续收集其它命中

        get_physx_scene_query_interface.overlap_sphere(
            float(radius_cm), origin, _report, False
        )

        return found["v"]

    def focus_on_prim(self, prim_path: str) -> Dict[str, Any]:
        from omni.kit.viewport.utility import (
            get_active_viewport,
            frame_viewport_selection,
        )

        try:
            ctx = omni.usd.get_context()
        except Exception as e:
            return {
                "status": "error",
                "message": f"Unable to retrieve USD context: {e}",
            }

        # 选中目标 prim
        try:
            ctx.get_selection().set_selected_prim_paths([prim_path], True)
        except Exception as e:
            return {
                "status": "error",
                "message": f"Failed to select prim '{prim_path}': {e}",
            }

        # 获取活跃视口
        try:
            viewport = get_active_viewport()
            if viewport is None:
                return {
                    "status": "error",
                    "message": f"No active viewport to focus on '{prim_path}'",
                }
        except Exception as e:
            return {
                "status": "error",
                "message": f"Error obtaining active viewport: {e}",
            }

        # 框选并聚焦
        try:
            frame_viewport_selection(viewport)
        except Exception as e:
            return {
                "status": "error",
                "message": f"Failed to frame prim '{prim_path}' in viewport: {e}",
            }

        return {
            "status": "success",
            "message": f"Prim '{prim_path}' focused in viewport",
        }

    def add_camera(
        self,
        translation: List[float],
        orientation: List[float],
        focal_length: float = 2.0,
        prim_path: str = "/World/MyCam",
    ) -> Dict[str, Any]:
        """
        使用 Isaac Sim 的高层 API 创建或封装一个相机。

        这个函数会实例化 Isaac Sim 的 Camera 类，该类会自动处理底层
        USD prim 的创建和属性设置。

        Args:
            translation (List[float]): 相机在世界坐标系中的位置 [x, y, z]。
            orientation (List[float]): 相机在世界坐标系中的朝向，使用四元数 [w, x, y, z] 格式。
            prim_path (str, optional): 要创建或封装的相机 Prim 的路径。
                                       默认为 "/World/MyCam"。

        Returns:
            Dict[str, Any]: 一个包含操作状态、消息和结果的字典。
                            - status (str): 'success' 或 'error'。
                            - message (str): 操作的描述性消息。
                            - result (dict | None): 成功时，返回一个包含两个关键对象的字典：
                                - 'usd_prim': 底层的 Pxr.Usd.Prim 对象。
                                - 'camera_instance': 高层的 isaac.core.prims.Camera 对象实例。
                              失败时为 None。
        """
        try:
            # 1. 使用 Isaac Sim 高层 API 创建或获取相机实例。

            camera_instance = Camera(
                prim_path=prim_path,
            )
            # set positon and quat here (important!)
            camera_instance.set_local_pose(
                translation=translation, orientation=orientation, camera_axes="usd"
            )
            camera_instance.set_focal_length(focal_length)

            return {
                "status": "success",
                "message": f"Isaac Sim Camera 实例已在 '{prim_path}' 创建或封装。",
                "result": {"prim_path": prim_path, "camera_instance": camera_instance},
            }

        except Exception as e:
            raise RuntimeError(f"add semantic camera failed '{prim_path}': {e}") from e

    def load_usd(
        self, usd_path: str, position: List[float], orientation: List[float]
    ) -> Dict[str, Any]:

        # 检查重合
        #        if self.check_prim_overlapping(position):
        #            return {"status": "error", "message": f"检测到重合，位置 {position} 已有对象，创建取消。"}

        # 获取场景中已有 prim 路径
        scene_info = self.get_scene_info()
        existing_paths = []
        if scene_info.get("status") == "success":
            for prim in scene_info.get("result", []):
                existing_paths.append(prim["path"])

        # 生成唯一 prim 路径与名称
        base_name = Path(usd_path).stem
        base_prim_path = f"/World/{base_name}"
        suffix = 0
        new_prim_path = base_prim_path
        new_name = base_name
        while new_prim_path in existing_paths:
            suffix += 1
            new_prim_path = f"{base_prim_path}_{suffix}"
            new_name = f"{base_name}_{suffix}"

        asset_path = ASSET_PATH + "/Isaac/" + usd_path

        # 将 asset 引用添加到 Stage
        add_reference_to_stage(usd_path=asset_path, prim_path=new_prim_path)

        # 设置初始位姿（使用 adjust_pose 方法）
        result = self.adjust_pose(
            new_prim_path,
            position,
            self.eluer_to_quaternion(orientation[0], orientation[1], orientation[2]),
        )
        if result.get("status") != "success":
            return result

        return {"status": "success", "result": new_prim_path}

    def create_robot(
        self, robot_type: str = "g1", position: List[float] = [0, 0, 0]
    ) -> Dict[str, Any]:

        from config.variables import ASSET_PATH

        ROBOT_CONFIGS = {
            "franka": {
                "path_usd": "/Isaac/Robots/Franka/franka.usd",
                "base_prim_path": "/World/Arm",
                "base_name": "my_arm",
            },
            "jetbot": {
                "path_usd": "/Isaac/Robots/Jetbot/jetbot.usd",
                "base_prim_path": "/World/Jetbot",
                "base_name": "my_jetbot",
            },
            "carter": {
                "path_usd": "/Isaac/Robots/NVIDIA/Carter/nova_carter/nova_carter.usd",
                "base_prim_path": "/World/Car",
                "base_name": "my_car",
            },
            "g1": {
                "path_usd": "/Isaac/Robots/Unitree/G1/g1.usd",
                "base_prim_path": "/World/G1",
                "base_name": "my_g1",
            },
            "go1": {
                "path_usd": "/Isaac/Robots/Unitree/Go1/go1.usd",
                "base_prim_path": "/World/Go1",
                "base_name": "my_go1",
            },
            "h1": {
                "path_usd": "/Isaac/Robots/Unitree/H1/h1.usd",
                "base_prim_path": "/World/H1",
                "base_name": "my_h1",
            },
        }

        robot_type = robot_type.lower()
        config = ROBOT_CONFIGS.get(robot_type, ROBOT_CONFIGS["franka"])
        asset_path = ASSET_PATH + config["path_usd"]

        # 获取当前 prim 信息
        scene_info = self.get_scene_info()
        existing_paths = []
        if scene_info["status"] == "success":
            for prim in scene_info["result"]:
                existing_paths.append(prim["path"])

        # 生成唯一 prim_path 和 name
        base_path = config["base_prim_path"]
        base_name = config["base_name"]
        suffix = 0
        new_prim_path = base_path
        new_name = base_name

        while new_prim_path in existing_paths:
            suffix += 1
            new_prim_path = f"{base_path}_{suffix}"
            new_name = f"{base_name}_{suffix}"

        # 添加到 Stage
        add_reference_to_stage(usd_path=asset_path, prim_path=new_prim_path)

        # 创建 Articulation 对象
        robot = Articulation(prim_paths_expr=new_prim_path, name=new_name)

        # 设置初始位姿
        robot.set_world_poses(positions=np.array([position]) / get_stage_units())
        return {
            "status": "success",
            "message": f"{robot_type} robot created at {new_prim_path}",
        }

    def create_shape_unified(
        self,
        shape_type: str,
        prim_path: str,
        name: str = None,
        position: Optional[Sequence[float]] = None,
        translation: Optional[Sequence[float]] = None,
        orientation: Optional[Sequence[float]] = None,
        scale: Optional[Sequence[float]] = None,
        color: Optional[np.ndarray] = None,
        size: Optional[float] = None,
        radius: Optional[float] = None,
        height: Optional[float] = None,
        mass: Optional[float] = None,
        density: Optional[float] = None,
        entity_type: Optional[str] = "visual",  # rigid\fixed\visual
    ) -> Dict[str, Any]:
        """
        在场景中创建一个几何形状，此版本统一使用 isaacsim.core 高层级API。
        """

        shape_type = shape_type.lower()
        position = np.array(position) if position is not None else None
        orientation = np.array(orientation) if orientation is not None else None
        scale = np.array(scale) if scale is not None else None
        color = np.array(color) if color is not None else None
        size = float(size) if size is not None else None
        radius = float(radius) if radius is not None else None
        height = float(height) if height is not None else None

        created_object = None
        if shape_type in ["cube", "cuboid"]:
            if entity_type == "rigid":
                # DynamicCuboid 会自动应用 RigidBodyAPI, CollisionAPI, MassAPI
                created_object = cuboid.DynamicCuboid(
                    prim_path=prim_path,
                    name=name,
                    position=position,
                    orientation=orientation,
                    size=size,
                    scale=scale,
                    color=color,
                    mass=mass,
                )
            elif entity_type == "fixed":
                # FixedCuboid 会自动应用 CollisionAPI，但不会应用 RigidBodyAPI
                created_object = cuboid.FixedCuboid(
                    prim_path=prim_path,
                    name=name,
                    position=position,
                    orientation=orientation,
                    size=size,
                    scale=scale,
                    color=color,
                )
            else:
                created_object = cuboid.VisualCuboid(
                    prim_path=prim_path,
                    name=name,
                    position=position,
                    orientation=orientation,
                    size=size,
                    scale=scale,
                    color=color,
                )
        elif shape_type == "sphere":
            if entity_type == "rigid":
                created_object = sphere.DynamicSphere(
                    prim_path=prim_path,
                    name=name,
                    position=position,
                    orientation=orientation,
                    radius=radius,
                    color=color,
                    scale=scale,
                    mass=mass,
                )
            elif entity_type == "fixed":
                created_object = sphere.FixedSphere(
                    prim_path=prim_path,
                    name=name,
                    position=position,
                    orientation=orientation,
                    radius=radius,
                    color=color,
                )
            else:
                created_object = sphere.VisualSphere(
                    prim_path=prim_path,
                    name=name,
                    position=position,
                    orientation=orientation,
                    radius=radius,
                    color=color,
                )
        elif shape_type == "cylinder":
            if entity_type == "rigid":
                created_object = sphere.DynamicSphere(
                    prim_path=prim_path,
                    name=name,
                    position=position,
                    orientation=orientation,
                    radius=radius,
                    height=height,
                    color=color,
                    mass=mass,
                )
            elif entity_type == "fixed":
                created_object = sphere.FixedSphere(
                    prim_path=prim_path,
                    name=name,
                    position=position,
                    orientation=orientation,
                    radius=radius,
                    height=height,
                    color=color,
                )
            else:
                created_object = sphere.VisualSphere(
                    prim_path=prim_path,
                    name=name,
                    position=position,
                    orientation=orientation,
                    radius=radius,
                    height=height,
                    color=color,
                )

        else:
            raise f"Unsupported shape type: {shape_type}"

        # 将创建的对象添加到场景中，使其“生效”
        scene = World.instance().scene
        if name != None and scene.object_exists(name) == False:
            scene.add(created_object)

        return {
            "status": "success",
            "message": f"Successfully created shape '{shape_type}' at {prim_path} using unified API.",
            "prim_path": prim_path,
            "object": created_object,
        }

    def create_shape_components(
        self,
        prim_path: str,
        name: str = None,
        position: List[float] = [0, 0, 0],
        orientation: List[float] = [1, 0, 0, 0],  # WXYZ
        # size: Union[float, List[float]] = 1.0,
        # color: List[float] = [0.8, 0.1, 0.1],
        # physics_preset: str = 'dynamic_rigid_body',  # 'visual', 'static_collider', 'dynamic_rigid_body'
        components: Optional[Dict[str, Dict]] = None,
    ) -> Dict[str, Any]:

        xform = UsdGeom.Xform.Define(self.stage, prim_path)
        prim = XFormPrim(
            prim_paths_expr=prim_path,
            positions=torch.tensor([position], dtype=torch.float32),
            orientations=torch.tensor([orientation], dtype=torch.float32),
            name=name,
        )

        scene = World.instance().scene
        scene.add(prim)
        for child_name, comp_data in components.items():
            child_path = f"{prim_path}/{child_name}"
            self.create_shape_unified(
                prim_path=child_path,
                shape_type=comp_data.get("shape_type", "cuboid"),
                position=comp_data.get("position", [0, 0, 0]),
                orientation=comp_data.get("orientation", [1, 0, 0, 0]),
                size=comp_data.get("size", 1.0),
                color=comp_data.get("color", [0.5, 0.5, 0.5]),
                # physics_preset=physics_preset,  # 复合体的所有部分共享同一个物理预设
                mass=comp_data.get("mass", 0.1),
                name=comp_data.get("name", None),
                entity_type=comp_data.get("entity_type", "visual"),
            )

        return {"status": "success", "result": prim_path, "usd_prim": xform.GetPrim()}

    def adjust_prim(
        self,
        prim_path: str = None,
        position: list = None,
        quat: list = None,
        scale: list = None,
    ) -> dict:
        try:
            geom_prim = self.stage.GetPrimAtPath(prim_path)
            if not geom_prim.IsValid():
                return {"status": "error", "message": f"Prim {prim_path} not found."}

            xform = UsdGeom.Xformable(geom_prim)

            if position:
                if not geom_prim.HasAttribute("xformOp:translate"):
                    xform.AddTranslateOp().Set(position)
                else:
                    geom_prim.GetAttribute("xformOp:translate").Set(position)
            if quat:
                if not geom_prim.HasAttribute("xformOp:orient"):
                    xform.AddOrientOp().Set(quat)
                else:
                    geom_prim.GetAttribute("xformOp:orient").Set(quat)

            if scale:
                if not geom_prim.HasAttribute("xformOp:scale"):
                    xform.AddScaleOp().Set(scale)  # Gf.Vec3f(*scale)
                else:
                    geom_prim.GetAttribute("xformOp:scale").Set(scale)

            return {"status": True}
        except Exception as e:
            return {
                "status": False,
                "message": str(e),
            }

    def adjust_pose(self, prim_path: str, position: list, orientation: list) -> dict:

        try:
            prim = self.stage.GetPrimAtPath(prim_path)
            if not prim or not prim.IsValid():
                return {"status": "error", "message": f"Prim {prim_path} not found."}

            is_articulation = False

            # 检查是否是 Articulation 类型
            try:
                robot = Articulation(prim_path)
                if robot.is_valid():
                    is_articulation = True
                    # 设置位置（Articulation 不支持旋转，需用 Xformable 设置）
                    import numpy as np

                    position_np = np.array([position]) / get_stage_units()
                    robot.set_world_poses(positions=position_np)
            except Exception as e:
                print(f"[DEBUG] Not an articulation or failed to init: {e}")

            xform = UsdGeom.Xformable(prim)

            # 设置旋转（四元数）
            quat = Gf.Quatf(*orientation)
            orient_op = None
            for op in xform.GetOrderedXformOps():
                if op.GetOpType() == UsdGeom.XformOp.TypeOrient:
                    orient_op = op
                    break
            if not orient_op:
                orient_op = xform.AddOrientOp()
            orient_op.Set(quat)

            # 非 articulation 类型，再设置位置
            if not is_articulation:
                translate_op = None
                for op in xform.GetOrderedXformOps():
                    if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                        translate_op = op
                        break
                if not translate_op:
                    translate_op = xform.AddTranslateOp()
                translate_op.Set(Gf.Vec3f(*position))

            return {
                "status": "success",
                "message": f"Set position and orientation for {prim_path}",
            }

        except Exception as e:
            return {"status": "error", "message": f"Failed to set transform: {str(e)}"}

    def set_prim_activate_state(self, prim_path: str, activate: bool = True) -> dict:
        """Set the active state of a prim."""
        try:
            # 获取指定 prim
            prim = self.stage.GetPrimAtPath(prim_path)
            if not prim.IsValid():
                return {"status": "error", "message": f"Prim '{prim_path}' not found."}

            # 设置激活状态
            prim.SetActive(activate)

            state_str = "activated" if activate else "deactivated"
            return {"status": "success", "message": f"Prim '{prim_path}' {state_str}."}

        except Exception as e:
            return {
                "status": "error",
                "message": f"Failed to set prim active state: {str(e)}",
            }

    def set_collision_offsets(
        self, prim_path: str, contact_offset: float, rest_offset: float
    ) -> dict:
        try:
            # 查找 prim
            prim = self.stage.GetPrimAtPath(prim_path)
            if not prim or not prim.IsValid():
                return {"status": "error", "message": f"Prim '{prim_path}' not found."}

            # 应用基础碰撞 API
            UsdPhysics.CollisionAPI.Apply(prim)

            # 应用 PhysX 扩展碰撞 API，设置偏移
            physx_api = PhysxSchema.PhysxCollisionAPI.Apply(prim)
            physx_api.CreateContactOffsetAttr().Set(contact_offset)
            physx_api.CreateRestOffsetAttr().Set(rest_offset)

            return {
                "status": "success",
                "message": f"Set contactOffset={contact_offset}, restOffset={rest_offset} for '{prim_path}'",
                "result": prim_path,
            }

        except Exception as e:
            return {
                "status": "error",
                "message": f"Failed to set collision offsets: {e}",
            }

    def set_collision_enabled(self, prim_path: str, collision_enabled: bool) -> dict:
        # 禁用碰撞属性
        rigid_prim = RigidPrim(prim_paths_expr=prim_path)
        collision_api = UsdPhysics.CollisionAPI.Apply(rigid_prim.prims[0])
        collision_api.GetCollisionEnabledAttr().Set(collision_enabled)

    def set_physics_properties(
        self, prim_path: str, mass: float, velocity: list, gravity_enabled: bool
    ) -> dict:
        try:
            # 查找 prim
            prim = self.stage.GetPrimAtPath(prim_path)
            if not prim or not prim.IsValid():
                return {"status": "error", "message": f"Prim '{prim_path}' not found."}

            # 1. 应用基础刚体 API 并设置初速度
            rigid_api = UsdPhysics.RigidBodyAPI.Apply(prim)
            rigid_api.CreateVelocityAttr().Set(Gf.Vec3f(*velocity))

            # 2. 应用 PhysX 扩展刚体 API，设置 disableGravity（取反 gravity_enabled）
            physx_rigid_api = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
            physx_rigid_api.CreateDisableGravityAttr().Set(not gravity_enabled)

            # 3. 应用 MassAPI，创建并设置质量属性
            mass_api = UsdPhysics.MassAPI.Apply(prim)
            mass_api.CreateMassAttr().Set(mass)

            return {
                "status": "success",
                "message": f"Physics properties set for '{prim_path}'",
                "result": prim_path,
            }

        except Exception as e:
            return {
                "status": "error",
                "message": f"Failed to set physics properties: {e}",
            }

    def set_collision_approximation(self, prim_path: str, approximation: str) -> dict:
        """
        在当前 USD Stage 上，为指定 Mesh prim 设置碰撞近似（approximation）属性。
        参数:
        prim_path: prim 路径，如 "/World/Cube"
        approximation: one of {
            "none", "meshSimplification", "convexHull",
            "convexDecomposition", "boundingSphere",
            "boundingCube", "sdf"
        }
        """
        try:
            # 查找 prim
            prim = self.stage.GetPrimAtPath(prim_path)
            if not prim or not prim.IsValid():
                return {"status": "error", "message": f"Prim '{prim_path}' not found."}

            # 1. 应用基础 CollisionAPI
            UsdPhysics.CollisionAPI.Apply(prim)

            # 2. 仅对 Mesh prim 应用 MeshCollisionAPI
            if prim.GetTypeName() != "Mesh":
                return {
                    "status": "error",
                    "message": f"Prim '{prim_path}' is not a Mesh.",
                }
            mesh_api = UsdPhysics.MeshCollisionAPI.Apply(prim)
            mesh_api.CreateApproximationAttr().Set(approximation)

            # 3. 若选择 SDF，需要额外设置 SDF 分辨率
            if approximation == "sdf":
                sdf_api = PhysxSchema.PhysxSDFMeshCollisionAPI.Apply(prim)
                sdf_api.CreateSdfResolutionAttr().Set(300)

            return {
                "status": "success",
                "message": f"Set collision approximation '{approximation}' for '{prim_path}'",
                "result": prim_path,
            }

        except Exception as e:
            return {
                "status": "error",
                "message": f"Failed to set collision approximation: {e}",
            }

    def set_material_properties(
        self,
        material_path: str,
        static_friction: float,
        dynamic_friction: float,
        restitution: float,
        static_friction_mode: str,
        dynamic_friction_mode: str,
        restitution_mode: str,
    ) -> dict:
        """
        在当前 USD Stage 上，为指定 Material prim 设置物理材质属性：
        - staticFriction: 静摩擦系数
        - dynamicFriction: 动摩擦系数
        - restitution: 恢复系数
        - staticFrictionCombineMode: 静摩擦合并方式
        - dynamicFrictionCombineMode: 动摩擦合并方式
        - restitutionCombineMode: 恢复系数合并方式
        """
        try:
            # 查找 material prim
            prim = self.stage.GetPrimAtPath(material_path)
            if not prim or not prim.IsValid():
                return {
                    "status": "error",
                    "message": f"Material '{material_path}' not found.",
                }

            # 1. 应用 MaterialAPI 设置摩擦与恢复系数
            mat_api = UsdPhysics.MaterialAPI.Apply(prim)
            mat_api.CreateStaticFrictionAttr().Set(static_friction)
            mat_api.CreateDynamicFrictionAttr().Set(dynamic_friction)
            mat_api.CreateRestitutionAttr().Set(restitution)

            # 2. 应用 PhysX 材质扩展设置合并模式
            physx_api = PhysxSchema.PhysxMaterialAPI.Apply(prim)
            physx_api.CreateStaticFrictionCombineModeAttr().Set(static_friction_mode)
            physx_api.CreateDynamicFrictionCombineModeAttr().Set(dynamic_friction_mode)
            physx_api.CreateRestitutionCombineModeAttr().Set(restitution_mode)

            return {
                "status": "success",
                "message": f"Material properties updated for '{material_path}'",
                "result": material_path,
            }

        except Exception as e:
            return {
                "status": "error",
                "message": f"Failed to set material properties: {e}",
            }

    def set_physics_scene_config(
        self, gravity: list, unit_system: str, scale: float, default_material: str = ""
    ) -> dict:
        """
        在当前 USD Stage 上配置全局物理场景：
        - gravity: 重力方向向量 [x, y, z]
        - unit_system: "metric" 或 "imperial"（仅供记录）
        - scale: 每个单位代表的米数 (metersPerUnit)
        - default_material: 默认物理材质 prim 路径（可选）
        """
        try:
            scene_path = "/physicsScene"
            scene_prim = self.stage.GetPrimAtPath(scene_path)
            if not scene_prim or not scene_prim.IsValid():
                scene_prim = self.stage.DefinePrim(scene_path, "PhysicsScene")
            scene = UsdPhysics.Scene(scene_prim)

            # 设置重力
            vec = Gf.Vec3f(*gravity)
            scene.CreateGravityDirectionAttr().Set(vec)
            scene.CreateGravityMagnitudeAttr().Set(vec.GetLength())

            # 设置舞台线性单位
            UsdGeom.SetStageMetersPerUnit(self.stage, scale)

            # 绑定默认物理材质（可选）
            if default_material:
                PhysxSchema.PhysxSceneAPI.Apply(scene_prim)
                mat_binding = UsdShade.MaterialBindingAPI.Apply(scene_prim)
                mat_prim = self.stage.GetPrimAtPath(default_material)
                if not mat_prim or not mat_prim.IsValid():
                    return {
                        "status": "error",
                        "message": f"Default material '{default_material}' not found.",
                    }
                material = UsdShade.Material(mat_prim)
                mat_binding.Bind(material, materialPurpose="physics")

            return {
                "status": "success",
                "message": "Physics scene configuration updated.",
                "result": scene_path,
            }

        except Exception as e:
            return {
                "status": "error",
                "message": f"Failed to set physics scene config: {e}",
            }

    def create_joint(
        self,
        joint_path: str,
        joint_type: str,
        body0: str,
        body1: str,
        axis: list,
        local_pos0: list,
        local_pos1: list,
        lower_limit: float = None,
        upper_limit: float = None,
    ) -> dict:
        try:
            # 校验两个刚体 prim 是否有效
            if not self.stage.GetPrimAtPath(body0).IsValid():
                return {"status": "error", "message": f"Body0 '{body0}' not found."}
            if not self.stage.GetPrimAtPath(body1).IsValid():
                return {"status": "error", "message": f"Body1 '{body1}' not found."}

            # 创建 joint prim 并选择类型
            joint_type = joint_type.lower()
            if joint_type == "revolute":
                joint = UsdPhysics.RevoluteJoint.Define(self.stage, joint_path)
            elif joint_type == "prismatic":
                joint = UsdPhysics.PrismaticJoint.Define(self.stage, joint_path)
            elif joint_type == "fixed":
                joint = UsdPhysics.FixedJoint.Define(self.stage, joint_path)
            else:
                return {
                    "status": "error",
                    "message": f"Invalid joint type: {joint_type}",
                }

            # 绑定连接体
            joint.CreateBody0Rel().SetTargets([body0])
            joint.CreateBody1Rel().SetTargets([body1])

            # 设置局部连接点
            joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*local_pos0))
            joint.CreateLocalPos1Attr().Set(Gf.Vec3f(*local_pos1))

            # 设置轴（仅 Revolute / Prismatic）
            if joint_type in ["revolute", "prismatic"]:
                joint.CreateAxisAttr().Set(Gf.Vec3f(*axis).GetNormalized())

                # 设置限制（可选）
                if lower_limit is not None:
                    joint.CreateLowerLimitAttr().Set(lower_limit)
                if upper_limit is not None:
                    joint.CreateUpperLimitAttr().Set(upper_limit)

            return {
                "status": "success",
                "message": f"{joint_type.capitalize()} joint created at {joint_path}",
                "result": joint_path,
            }

        except Exception as e:
            return {"status": "error", "message": f"Failed to create joint: {str(e)}"}

    def set_drive_parameters(
        self,
        joint_path: str,
        drive_name: str,
        target_position: float,
        target_velocity: float,
        stiffness: float,
        damping: float,
        max_force: float,
    ) -> dict:
        try:
            joint = self.stage.GetPrimAtPath(joint_path)
            if not joint.IsValid():
                return {
                    "status": "error",
                    "message": f"Joint prim '{joint_path}' not found.",
                }

            # 应用 DriveAPI 到关节上（名称可以是 "angular" 或 "linear"）
            drive_api = UsdPhysics.DriveAPI.Apply(joint, drive_name)

            drive_api.CreateTargetPositionAttr().Set(target_position)
            drive_api.CreateTargetVelocityAttr().Set(target_velocity)
            drive_api.CreateStiffnessAttr().Set(stiffness)
            drive_api.CreateDampingAttr().Set(damping)
            drive_api.CreateMaxForceAttr().Set(max_force)

            return {
                "status": "success",
                "message": f"Drive parameters set on {joint_path} ({drive_name})",
                "result": joint_path,
            }

        except Exception as e:
            return {
                "status": "error",
                "message": f"Failed to set drive parameters: {str(e)}",
            }

    ###############################################################################################################
    ###############################################################################################################

    # TODO:扩展到isaacsim官方Assets,支持更多的场景，或者使用Asset browser mcp的resource 功能。
    def browse_scene_repository(
        self,
    ) -> Dict[str, Any]:
        """Browse the scene repository and return a list of files."""
        try:
            import os

            # 获取场景目录
            scene_dir = self.scene_repository_path
            print("scene_dir: ", scene_dir)

            # 获取目录下的所有文件和子目录
            items = os.listdir(scene_dir)
            # 过滤出所有 usd 文件以及其绝对路径
            usd_files = [
                os.path.join(scene_dir, item)
                for item in items
                if item.endswith(".usd") or item.endswith(".usda")
            ]
            print("usd_files: ", usd_files)

            return {
                "status": "success",
                "message": "Scene repository browsed successfully",
                "result": usd_files,
            }

        except Exception as e:
            return {
                "status": "error",
                "message": f"Failed to browse scene repository: {e}",
                "result": None,
            }

    def disable_gravity_for_hierarchy(self, root_prim_path: str) -> dict:
        """
        递归地遍历指定路径下的所有Prim，并禁用任何找到的刚体的重力。

        这是处理从外部文件加载的、作为静态背景的场景的最佳方法。

        Args:
            root_prim_path (str): 您加载的USD场景的根路径 (e.g., "/World/MyBuilding").

        Returns:
            dict: 操作结果的状态字典，包含修改的Prim数量。
        """
        try:
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
            return {
                "status": "error",
                "message": f"An unexpected error occurred: {str(e)}",
            }

    def load_scene(self, usd_path: str, prim_path_root: str):
        """
        Create a scene from config.(But just input usd file yet.)
        Args:
            usd_path (str): path to scene config file(use to be a .usd file)
            prim_path_root (str): path to root prim
        """
        if (
            usd_path.endswith("usd")
            or usd_path.endswith("usda")
            or usd_path.endswith("usdc")
        ):

            root_prim = create_prim(
                prim_path_root,
                usd_path=usd_path,
                # scale=self.simulation.scene_scale,
                scale=[1, 1, 1],
                # translation=[0, 0, 0.81696],
                # orientation=[0.610, -0.789, -0.05184, 0.040] # wxyz, xyz还是zyx顺序不确定
            )
            # for prim in Usd.PrimRange(root_prim):
            #     UsdPhysics.RigidBodyAPI.Apply(prim)
            # self.disable_gravity_for_hierarchy(root_prim_path=prim_path_root)

        else:
            raise RuntimeError("Env file path needs to end with .usd, .usda or .usdc .")
        return

    def save_scene(
        self,
        scene_name: str = None,
        save_directory: str = None,
        flatten_scene: bool = True,
    ) -> Dict[str, Any]:
        """
        Save the current scene to a specified directory.

        Args:
            scene_name (str, optional): Name of the scene file (without extension).
            save_directory (str, optional): Absolute path to save directory.
                                          If None, uses self.scene_repository_path.
            flatten_scene (bool, optional): If True, flattens the scene to include all
                                          referenced assets. If False, saves only references.

        Returns:
            Dict[str, Any]: Status and result information.
        """
        try:
            import os

            # 确定保存目录
            if save_directory is None:
                save_directory = self.scene_repository_path

            save_directory = os.path.abspath(save_directory)

            # 确保保存目录存在
            os.makedirs(save_directory, exist_ok=True)

            # 如果没有提供场景名称，使用时间戳
            if scene_name is None:
                import datetime

                scene_name = (
                    f"scene_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}"
                )

            # 构建完整的保存路径
            if not scene_name.endswith(".usd"):
                scene_name += ".usd"
            save_path = os.path.join(save_directory, scene_name)

            # 获取当前stage
            stage = omni.usd.get_context().get_stage()
            if not stage:
                return {
                    "status": "error",
                    "message": "No active USD stage to save",
                    "result": None,
                }

            if flatten_scene:
                # 方法1: 扁平化保存 - 将所有引用的资产嵌入到单个文件中
                print(f"Flattening and saving scene to: {save_path}")

                # 创建一个新的stage用于扁平化
                flattened_stage = Usd.Stage.CreateNew(save_path)

                # 复制根层的所有内容
                root_layer = stage.GetRootLayer()
                flattened_layer = flattened_stage.GetRootLayer()

                # 扁平化stage - 这会将所有引用的内容合并到一个文件中
                stage.Flatten().Export(save_path)

                # 获取文件大小信息
                file_size = os.path.getsize(save_path) / (1024 * 1024)  # MB

                return {
                    "status": "success",
                    "message": f"Scene flattened and saved successfully as {scene_name} at {save_path} (Size: {file_size:.2f} MB)",
                    "result": {
                        "path": save_path,
                        "size_mb": file_size,
                        "flattened": True,
                    },
                }
            else:
                # 方法2: 引用保存 - 只保存引用，文件较小
                print(f"Saving scene with references to: {save_path}")
                omni.usd.get_context().save_as_stage(save_path, None)

                # 获取文件大小信息
                file_size = os.path.getsize(save_path) / 1024  # KB

                return {
                    "status": "success",
                    "message": f"Scene saved with references as {scene_name} at {save_path} (Size: {file_size:.2f} KB)",
                    "result": {
                        "path": save_path,
                        "size_kb": file_size,
                        "flattened": False,
                    },
                }

        except Exception as e:
            import traceback

            traceback.print_exc()
            return {
                "status": "error",
                "message": f"Failed to save scene: {e}",
                "result": None,
            }

    def enable_raycasting_for_prim(self, prim_path: str) -> dict:
        """
        为一个Prim显式地启用场景查询功能，使其可以被光线投射检测到。

        Args:
            prim_path (str): 目标Prim的USD路径。

        Returns:
            dict: 操作结果的状态字典。
        """
        try:
            stage = omni.usd.get_context().get_stage()
            if not stage:
                return {"status": "error", "message": "No active USD stage."}

            prim = stage.GetPrimAtPath(prim_path)
            if not prim.IsValid():
                return {
                    "status": "error",
                    "message": f"Prim at '{prim_path}' is not valid.",
                }

            # 1. 场景查询的属性属于 UsdPhysics.CollisionAPI
            if not prim.HasAPI(UsdPhysics.CollisionAPI):
                print(
                    f"INFO: CollisionAPI not found on '{prim_path}'. Applying it now."
                )
                collision_api = UsdPhysics.CollisionAPI.Apply(prim)
            else:
                collision_api = UsdPhysics.CollisionAPI(prim)

            # 2. 核心操作：获取并设置'sceneQueryEnabled'属性为 True
            #    这个属性默认是 False！
            # collision_api.GetSceneQueryEnabledAttr().Set(True)

            return {
                "status": "success",
                "message": f"Raycasting (Scene Query) successfully enabled for '{prim_path}'.",
            }

        except Exception as e:
            import traceback

            traceback.print_exc()
            return {
                "status": "error",
                "message": f"An unexpected error occurred: {str(e)}",
            }
