# =============================================================================
# Camera Module - Camera Sensor Implementation
# =============================================================================
#
# This module provides camera sensor implementation with image capture,
# processing, and integration with Isaac Sim's rendering pipeline.
#
# =============================================================================

# Standard library imports
from typing import Tuple, Sequence

# Third-party library imports
import numpy as np
from PIL import Image

# Local project imports
from log.log_manager import LogManager
from physics_engine.isaacsim_utils import (
    Camera as IsaacCamera,
    rotations,
    define_prim,
    get_prim_at_path,
)
from simulation.sensor.camera import CfgCamera
from utils import to_torch

logger = LogManager.get_logger(__name__)


class Camera:
    def __init__(self, path_prim_parent: str, cfg_camera: CfgCamera):
        self.cfg_camera = cfg_camera
        self.path_prim_parent = path_prim_parent
        self.camera = None
        self.count_for_sim_update = 0
        
        # 延迟初始化标志
        self._initialized = False

    def initialize_on_physics_step(self) -> None:
        """
        在 physics step 中延迟初始化 camera
        这是最可靠的方式，避免传感器创建时的时序问题
        """
        from containers import get_container

        container = get_container()
        world = container.world_configured()

        # 订阅 physics step 事件，在下一个物理步进时创建传感器
        def _create_on_step(step_size):
            if not self._initialized:
                self._create_camera_internal()
                self._initialized = True
                # 取消订阅
                callback_name = f"camera_creation_{self.cfg_camera.type}_{self.cfg_camera.id}"
                world.get_isaac_world().remove_physics_callback(callback_name)

        callback_name = f"camera_creation_{self.cfg_camera.type}_{self.cfg_camera.id}"
        world.get_isaac_world().add_physics_callback(callback_name, _create_on_step)
        logger.info("Scheduled camera creation on next physics step")

    def _create_camera_internal(self) -> None:
        """
        内部方法：实际创建 camera 传感器
        应该在 physics step 回调中调用，确保时序正确
        使用 xform + rigid body + fixed joint 的方式连接到机器人
        """
        from pxr import UsdGeom, UsdPhysics
        from physics_engine.omni_utils import omni
        from physics_engine.pxr_utils import Gf
        from containers import get_container

        container = get_container()
        world = container.world_configured()

        try:
            # 确定 camera 的路径
            self.cfg_camera.name = self.cfg_camera.type + "_" + str(self.cfg_camera.id)
            
            if self.cfg_camera.use_existing_camera:
                # 使用已存在的 camera，不创建 xform 和 joint
                self.cfg_camera.path_prim_absolute = (
                    self.path_prim_parent + self.cfg_camera.path_prim_relative
                )
                self.camera = IsaacCamera(
                    prim_path=self.cfg_camera.path_prim_absolute,
                    name=self.cfg_camera.name,
                    frequency=self.cfg_camera.frequency,
                    dt=self.cfg_camera.dt,
                    resolution=self.cfg_camera.resolution,
                    render_product_path=None,
                )
                logger.info(f"Using existing camera at: {self.cfg_camera.path_prim_absolute}")
            else:
                # 创建新的 camera，使用 xform + rigid body + fixed joint
                relative_path = self.cfg_camera.path_prim_relative.lstrip("/")
                
                # 1. 创建 xform 节点作为容器
                xform_path = f"{self.path_prim_parent}/{relative_path}"
                stage = omni.usd.get_context().get_stage()
                xform_prim = UsdGeom.Xform.Define(stage, xform_path).GetPrim()

                # 2. 添加 rigid body 到 xform
                rigid_body_api = UsdPhysics.RigidBodyAPI.Apply(xform_prim)
                rigid_body_api.CreateRigidBodyEnabledAttr(True)

                # 3. 在 xform 下创建 camera
                camera_path = f"{xform_path}/{self.cfg_camera.name}"
                self.cfg_camera.path_prim_absolute = camera_path
                
                self.camera = IsaacCamera(
                    prim_path=camera_path,
                    name=self.cfg_camera.name,
                    frequency=self.cfg_camera.frequency,
                    dt=self.cfg_camera.dt,
                    resolution=self.cfg_camera.resolution,
                    render_product_path=None,
                )

                # 设置 camera 的局部位姿
                self.set_local_pose(
                    translation=to_torch(self.cfg_camera.translation),
                    orientation=to_torch(self.cfg_camera.orientation),
                    camera_axes="usd",
                )

                # 4. 禁用 xform 层级的重力
                scene_manager = container.scene_manager()
                scene_manager.disable_gravity_for_hierarchy(xform_path)

                # 5. 创建 Fixed Joint 连接到机器人
                if self.cfg_camera.attach_prim_relative_path is not None:
                    attach_prim_path = self.path_prim_parent + self.cfg_camera.attach_prim_relative_path
                    joint_path = f"/World/camera_joint_{relative_path.replace('/', '_')}"
                    joint_prim = stage.GetPrimAtPath(joint_path)

                    if not joint_prim.IsValid():
                        world.create_joint(
                            joint_path=joint_path,
                            joint_type="fixed",
                            body0=attach_prim_path,
                            body1=xform_path,
                            local_pos_0=(0, 0, 0),
                            local_pos_1=(0, 0, 0),
                            axis=(1, 0, 0),
                        )
                        joint_prim = stage.GetPrimAtPath(joint_path)

                    # Enable joint
                    joint = UsdPhysics.Joint(joint_prim)
                    joint.GetLocalPos0Attr().Set(Gf.Vec3f(0, 0, 0))
                    joint.GetLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))
                    joint.GetJointEnabledAttr().Set(True)

                    logger.info(f"Created camera with xform, rigid body, fixed joint at: {xform_path}")
                else:
                    logger.warning("attach_prim_relative_path not specified, camera created without joint connection")
                    logger.info(f"Created camera with xform, rigid body at: {xform_path}")

        except Exception as e:
            logger.error(f"Failed to create camera: {e}")
            raise



    def is_initialized(self) -> bool:
        """检查传感器是否已初始化"""
        return self._initialized

    def initialize(self) -> None:
        """
        Returns:
            True if the view object was initialized (after the first call of .initialize()). False otherwise.
        """
        if not self._initialized:
            logger.warning("Camera not initialized yet, skipping initialize call")
            return
            
        self.camera.initialize()
        if self.cfg_camera.enable_semantic_detection:
            self.camera.add_bounding_box_2d_loose_to_frame()

    def set_local_pose(
        self,
        translation: Sequence[float] = None,
        orientation: Sequence[float] = None,
        camera_axes: str = "usd",
    ) -> None:
        self.camera.set_local_pose(
            translation=translation, orientation=orientation, camera_axes=camera_axes
        )
        return None

    def get_current_frame(self):
        if not self._initialized:
            logger.warning("Camera not initialized yet, returning None")
            return None
        return self.camera.get_current_frame()

    def get_depth(self):
        if not self._initialized:
            logger.warning("Camera not initialized yet, returning None")
            return None
        return self.camera.get_depth()

    def get_point_cloud(self):
        if not self._initialized:
            logger.warning("Camera not initialized yet, returning None")
            return None
        return self.camera.get_point_cloud()

    def get_rgb(self) -> np.ndarray:
        if not self._initialized:
            logger.warning("Camera not initialized yet, returning None")
            return None
        return self.camera.get_rgb()

    def get_semantic_detection(self) -> np.ndarray:
        if not self._initialized:
            logger.warning("Camera not initialized yet, returning None")
            return None
            
        if self.cfg_camera.enable_semantic_detection in [False, None]:
            self.cfg_camera.enable_semantic_detection = True
            self.camera.add_bounding_box_2d_loose_to_frame()
            self.count_for_sim_update = 100
            return f"第一次使用语义检测功能, 第一次进行初始化中, 下次调用生效"
        if self.count_for_sim_update:
            self.count_for_sim_update -= 1
            return f"刚启动语义检测功能, 等待仿真器响应"

        return self.camera.get_current_frame()["bounding_box_2d_loose"]

    def get_local_pose(self, camera_axes: str = "usd"):
        if not self._initialized:
            logger.warning("Camera not initialized yet, returning None")
            return None, None
        return self.camera.get_local_pose(camera_axes=camera_axes)

    def get_world_pose(self, camera_axes: str = "usd") -> Tuple[np.ndarray, np.ndarray]:
        if not self._initialized:
            logger.warning("Camera not initialized yet, returning None")
            return None, None
        return self.camera.get_world_pose(camera_axes=camera_axes)

    def save_rgb_to_file(self, rgb: np.ndarray, file_path: str = None) -> str:
        """
        保存RGB图像到文件

        Args:
            rgb: numpy array RGB数据 [H, W, C]
            file_path: 保存路径

        """
        try:
            # 确保数据类型为uint8
            if rgb.dtype != np.uint8:
                if rgb.max() <= 1.0:
                    rgb = (rgb * 255).astype(np.uint8)
                else:
                    rgb = rgb.astype(np.uint8)

            Image.fromarray(rgb, "RGB").save(file_path)

            return f"图像已成功保存到: {file_path}"
        except Exception as e:
            return f"Error: {repr(e)}"
