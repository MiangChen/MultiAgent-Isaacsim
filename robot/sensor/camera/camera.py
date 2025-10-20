from typing import List, Optional, Tuple, Sequence

import numpy as np
import torch
from torchvision.utils import save_image

from physics_engine.isaacsim_utils import (
    Camera as IsaacCamera,
    rotations,
    define_prim,
    get_prim_at_path,
)
from pxr import Usd, UsdGeom, Gf

from log.log_manager import LogManager
from robot.sensor.camera import CfgCamera
from utils import to_torch

logger = LogManager.get_logger(__name__)


class Camera:
    def __init__(self, path_prim_parent: str, cfg_camera: CfgCamera):
        self.cfg_camera = cfg_camera
        self.path_prim_parent = path_prim_parent
        self.create_camera()

    def create_camera(self):
        self.cfg_camera.name = self.cfg_camera.type + "_" + str(self.cfg_camera.id)
        if self.cfg_camera.use_existing_camera == True:
            self.cfg_camera.path_prim_absolute = (
                self.path_prim_parent + self.cfg_camera.path_prim_relative_to_robot
            )
            self.camera = IsaacCamera(
                prim_path=self.cfg_camera.path_prim_absolute,
                name=self.cfg_camera.name,
                frequency=self.cfg_camera.frequency,
                dt=self.cfg_camera.dt,
                resolution=self.cfg_camera.resolution,
                # position=self.cfg_camera.position,
                # orientation=self.cfg_camera.orientation,
                # translation=self.cfg_camera.translation,
                render_product_path=None,
            )
        else:
            self.cfg_camera.path_prim_absolute = (
                self.path_prim_parent
                + self.cfg_camera.path_prim_relative_to_robot
                + "/"
                + self.cfg_camera.name
            )

            self.camera = IsaacCamera(
                prim_path=self.cfg_camera.path_prim_absolute,
                name=self.cfg_camera.name,
                frequency=self.cfg_camera.frequency,
                dt=self.cfg_camera.dt,
                resolution=self.cfg_camera.resolution,
                # position=self.cfg_camera.position,
                # orientation=self.cfg_camera.orientation,
                # translation=self.cfg_camera.translation,
                render_product_path=None,
            )

            self.set_local_pose(
                translation=to_torch(self.cfg_camera.translation),
                orientation=to_torch(self.cfg_camera.orientation),
                camera_axes="usd",
            )

        return

    def initialize(self) -> None:
        """
        Returns:
            True if the view object was initialized (after the first call of .initialize()). False otherwise.
        """
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
        return self.camera.get_current_frame()

    def get_depth(self):
        return self.camera.get_depth()

    def get_point_cloud(self):
        return self.camera.get_point_cloud()

    def get_rgb(self) -> torch.Tensor:
        """

        Returns:
            containing the RGB data for each camera. Shape is (num_cameras, height, width, 3) with type torch.float32.

        """
        return self.camera.get_rgb()

    def get_local_pose(self, camera_axes: str = "usd"):
        return self.camera.get_local_pose(camera_axes=camera_axes)

    def get_world_pose(self, camera_axes: str = "usd") -> Tuple[np.ndarray, np.ndarray]:
        return self.camera.get_world_pose(camera_axes=camera_axes)

    def save_rgb_to_file(
        self, rgb_tensor_gpu: torch.Tensor, file_path: str = None
    ) -> bool:
        """
        使用 torchvision.utils.save_image 简化版本

        Args:
            rgb_tensor_gpu: [height, width, 3] 或 [batch, height, width, 3] RGB Tensor
            file_path: 保存路径

        Returns:
            bool: 是否保存成功
        """
        try:
            if not isinstance(rgb_tensor_gpu, torch.Tensor):
                logger.error(
                    f"输入无效：期望一个 torch.Tensor，但收到了 {type(rgb_tensor_gpu)}。"
                )
                return False

            if not isinstance(file_path, str) or not file_path:
                logger.error(
                    f"文件路径无效：路径必须是一个非空字符串，但收到了 '{file_path}'。"
                )
                return False

            # 如果是4维张量 (batch, H, W, C)，则只取第一张图
            if rgb_tensor_gpu.ndim == 4:
                logger.warning(
                    f"输入为4维张量，将只保存第一张图像。形状: {rgb_tensor_gpu.shape}"
                )
                rgb_tensor_gpu = rgb_tensor_gpu[0]

            # 核心检查：必须是3维张量
            if rgb_tensor_gpu.ndim != 3 or rgb_tensor_gpu.shape[2] != 3:
                logger.error(
                    f"张量形状错误：期望 [H, W, 3]，但收到了 {rgb_tensor_gpu.shape}"
                )
                return False

            logger.info(f"开始处理图像，准备保存到 {file_path}...")

            # save_image 要求浮点张量在 [0,1] 范围内，或直接是 uint8 张量
            if rgb_tensor_gpu.max() > 1.0:
                rgb_tensor_gpu = rgb_tensor_gpu / 255.0
            # torchvision 需要 [C, H, W] 格式，因此需要重排维度, permute(2, 0, 1) 将 [H, W, C] 变为 [C, H, W]
            tensor_chw = rgb_tensor_gpu.permute(2, 0, 1)

            save_image(tensor_chw, file_path)

            logger.info(f"图像已成功保存到: {file_path}")
            return True

        except Exception as e:
            # 捕获任何可能发生的异常
            logger.error(f"保存文件到 {file_path} 时发生未知错误: {e}", exc_info=True)
            return False
