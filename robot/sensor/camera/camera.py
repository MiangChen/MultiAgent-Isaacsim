from typing import List, Optional, Tuple

import numpy as np
from pxr import Usd, UsdGeom, Gf
import torch
from torchvision.utils import save_image

from isaacsim.sensors.camera import CameraView
from isaacsim.core.utils.numpy import rotations
from isaacsim.core.utils.prims import define_prim, get_prim_at_path

from robot.sensor.camera.cfg_camera import CfgCamera
from robot.robot_cfg import CfgRobot
from log.log_manager import LogManager
from utils import to_torch

logger = LogManager.get_logger(__name__)


class Camera:

    def __init__(self, cfg_body: CfgRobot, cfg_camera: CfgCamera):
        self.cfg_camera = cfg_camera
        self.cfg_body = cfg_body

    def create_camera(self, camera_path: str = None):
        if camera_path is None:
            self.cfg_camera.prim_path_absolute = (
                self.cfg_body.prim_path_swarm + "/camera/Camera"
            )
        else:
            self.cfg_camera.prim_path_absolute = camera_path
        prim = get_prim_at_path(self.cfg_camera.prim_path_absolute)

        if prim.IsValid():
            self.camera_view = CameraView(
                prim_paths_expr=self.cfg_camera.prim_path_absolute,
                output_annotators=["rgb"],
            )

        else:
            prim = define_prim(self.cfg_camera.prim_path_absolute, "Xform")
            if prim.IsA(UsdGeom.Xformable):

                # 获取值（默认时间或指定时间）
                timecode = Usd.TimeCode.Default()

                translate_attr = prim.GetAttribute("xformOp:translate")
                if not translate_attr:
                    print("Prim 未定义 xformOp:translate 属性")
                else:  # 静态用默认时间，动态用 Usd.TimeCode(frame)
                    translate_value: Gf.Vec3d = translate_attr.Get(timecode)
                    # print(f"平移值: {tra/slate_value}")  # 例如 (1.0, 2.0, 3.0)
                    self.cfg_camera.position = list(translate_value)

                quat_attr = prim.GetAttribute("xformOp:orient")
                if not quat_attr:
                    print("Prim 未定义 xformOp:orient 属性")
                else:
                    quat_value = quat_attr.Get(timecode)
                    self.cfg_camera.quat = [quat_value.real] + list(
                        quat_value.imaginary
                    )
                    self.cfg_camera.euler_degree = None
            else:
                if prim:
                    print(
                        f"Prim at {prim.GetPath()} is not Xformable or does not exist."
                    )
                else:
                    print(f"Prim not found at path {prim.GetPath()}")

            self.camera_view = CameraView(
                prim_paths_expr=self.cfg_camera.prim_path_absolute,
                # frequency=self.cfg_camera.frequency,
                # resolution=self.cfg_camera.resolution,
                translations=to_torch(self.cfg_camera.position).reshape(1, 3),
                orientations=to_torch(self.cfg_camera.quat).reshape(1, 4),
                output_annotators=["rgb"],
            )

            self.set_local_pose(
                positions=to_torch(self.cfg_camera.position).reshape(1, 3),
                orientations=to_torch(self.cfg_camera.quat).reshape(1, 4),
                camera_axes="usd",
            )
        return

    def initialize(self) -> bool:
        """
        Returns:
            True if the view object was initialized (after the first call of .initialize()). False otherwise.
        """
        self.camera_view.initialize()
        # 设置深度功能
        # self.camera.add_distance_to_camera_to_frame() # camera view不用
        # 物品检测功能
        # self.camera.add_bounding_box_2d_loose_to_frame() # camera view不用
        return self.camera_view.initialized

    def set_local_pose(
        self,
        positions: Tuple[float, float, float],
        orientations: Tuple[float, float, float, float],
        camera_axes: str = "usd",
    ) -> None:
        self.camera_view.set_local_poses(
            positions=positions, orientations=orientations, camera_axes=camera_axes
        )
        return None

    def get_current_frame(self):
        return self.camera_view.get_current_frame()

    def get_depth(self):
        return self.camera_view.get_depth()

    def get_point_cloud(self):
        return self.camera_view.get_point_cloud()

    def get_rgb(self) -> torch.Tensor:
        """

        Returns:
            containing the RGB data for each camera. Shape is (num_cameras, height, width, 3) with type torch.float32.

        """
        return self.camera_view.get_rgb()

    def get_local_pose(self, camera_axes: str = "usd"):
        return self.camera_view.get_local_pose(camera_axes=camera_axes)

    def get_world_pose(self, camera_axes: str = "usd") -> Tuple[np.ndarray, np.ndarray]:
        return self.camera_view.get_world_pose(camera_axes=camera_axes)

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
