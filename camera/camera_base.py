import logging
import os
from typing import List, Optional, Tuple



import numpy as np
from pxr import Usd, UsdGeom, Gf
from PIL import Image
import torch

from isaacsim.sensors.camera import CameraView
from isaacsim.core.utils.numpy import rotations
from isaacsim.core.utils.prims import define_prim, get_prim_at_path

from camera.camera_cfg import CameraCfg
from robot.robot_cfg import RobotCfg


class CameraBase:

    def __init__(self, cfg_body: RobotCfg, cfg_camera: CameraCfg):
        self.cfg_camera = cfg_camera
        self.cfg_body = cfg_body

    def create_camera(self, camera_path: str=None):
        if camera_path is None:
            self.cfg_camera.prim_path = self.cfg_body.prim_path + '/camera/Camera'
        else:
            self.cfg_camera.prim_path = camera_path
        prim = get_prim_at_path(self.cfg_camera.prim_path)

        if prim.IsValid():
            self.camera_view = CameraView(
                prim_paths_expr=self.cfg_camera.prim_path,
                output_annotators=['rgb']
            )

        else:
            prim = define_prim(self.cfg_camera.prim_path, "Xform")
            if prim.IsA(UsdGeom.Xformable):

                # 获取值（默认时间或指定时间）
                timecode = Usd.TimeCode.Default()

                translate_attr = prim.GetAttribute('xformOp:translate')
                if not translate_attr:
                    print("Prim 未定义 xformOp:translate 属性")
                else:  # 静态用默认时间，动态用 Usd.TimeCode(frame)
                    translate_value: Gf.Vec3d = translate_attr.Get(timecode)
                    # print(f"平移值: {tra/slate_value}")  # 例如 (1.0, 2.0, 3.0)
                    self.cfg_camera.position = list(translate_value)

                quat_attr = prim.GetAttribute('xformOp:orient')
                if not quat_attr:
                    print("Prim 未定义 xformOp:orient 属性")
                else:
                    quat_value = quat_attr.Get(timecode)
                    self.cfg_camera.quat = [quat_value.real] + list(quat_value.imaginary)
                    self.cfg_camera.euler_degree = None
            else:
                if prim:
                    print(f"Prim at {prim.GetPath()} is not Xformable or does not exist.")
                else:
                    print(f"Prim not found at path {prim.GetPath()}")

            # 配置相机角度必须是4元数
            if self.cfg_camera.euler_degree is not None:
                # 注意角度和弧度模式
                self.cfg_camera.quat = rotations.euler_angles_to_quats(np.array(self.cfg_camera.euler_degree), degrees=True)
            else:
                self.cfg_camera.euler_degree = rotations.quats_to_euler_angles(np.array(self.cfg_camera.quat), degrees=True)

            self.camera_view = CameraView(
                prim_paths_expr=self.cfg_camera.prim_path,
                # frequency=self.cfg_camera.frequency,
                # resolution=self.cfg_camera.resolution,
                translations=[self.cfg_camera.position],
                orientations=[self.cfg_camera.quat],
                output_annotators=['rgb']
            )
            print(self.cfg_camera.quat)
            self.set_local_pose(translation=self.cfg_camera.position, orientation=self.cfg_camera.quat,
                                camera_axes='usd')

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

    def set_local_pose(self, translation: Tuple[float, float, float],
                       orientation: Tuple[float, float, float, float],
                       camera_axes: str = 'usd') -> None:
        self.camera_view.set_local_pose(translation=translation, orientation=orientation, camera_axes=camera_axes)
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

    def get_local_pose(self, camera_axes: str = 'usd'):
        return self.camera_view.get_local_pose(camera_axes=camera_axes)

    def get_world_pose(self, camera_axes: str = 'usd') -> Tuple[np.ndarray, np.ndarray]:
        return self.camera_view.get_world_pose(camera_axes=camera_axes)

    def save_rgb_to_file(self, rgb_tensor_gpu:torch.Tensor, file_path:str=None) -> bool:
        """

        Args:
            rgb_tensor_gpu: [height, width, 3] RGB
            file_path: str

        Returns:

        """
        try:
            # --- 1. 输入验证 ---
            if not isinstance(rgb_tensor_gpu, torch.Tensor):
                logging.error(f"输入无效：期望一个 torch.Tensor，但收到了 {type(rgb_tensor_gpu)}。")
                return False

            # **【核心要求】检查张量是否为3个维度**
            if rgb_tensor_gpu.ndim != 3:
                if rgb_tensor_gpu[0].ndim != 3:
                    logging.error(
                        f"张量维度错误：期望3个维度 [H, W, C]，但收到了 {rgb_tensor_gpu.ndim} 个维度。"
                        f" 张量形状为: {rgb_tensor_gpu.shape}"
                    )
                    return False
                else:
                    rgb_tensor_gpu = rgb_tensor_gpu[0]


            # 检查通道数是否为3 (RGB)
            if rgb_tensor_gpu.shape[2] != 3:
                logging.error(
                    f"通道数错误：期望最后一个维度为 3 (RGB)，但收到了 {rgb_tensor_gpu.shape[2]}。"
                    f" 张量形状为: {rgb_tensor_gpu.shape}"
                )
                return False

            if not isinstance(file_path, str) or not file_path:
                logging.error(f"文件路径无效：路径必须是一个非空字符串，但收到了 '{file_path}'。")
                return False

            # --- 2. 数据处理与转换 ---
            logging.info(f"开始处理图像，准备保存到 {file_path}...")

            # 将数据从 GPU 移至 CPU
            rgb_tensor_cpu = rgb_tensor_gpu.cpu()

            # 将 PyTorch 张量转换为 NumPy 数组
            rgb_numpy_array = rgb_tensor_cpu.numpy()

            # 稳健地将 float 类型转换为 uint8
            # 如果原始数据是 0-1 范围的浮点数，先乘以 255
            if np.issubdtype(rgb_numpy_array.dtype, np.floating) and rgb_numpy_array.max() <= 1.0:
                rgb_numpy_array = rgb_numpy_array * 255.0

            # 使用 np.clip 确保数值在 0-255 范围内，然后才转换类型，防止数据溢出
            image_uint8_array = np.clip(rgb_numpy_array, 0, 255).astype(np.uint8)

            # --- 3. 文件系统操作 ---
            # 从完整文件路径中获取目录路径
            directory = os.path.dirname(file_path)

            # 如果存在目录路径，则创建它（如果它还不存在）
            if directory:
                os.makedirs(directory, exist_ok=True)

            # 从 NumPy 数组创建 Pillow 图像对象
            image = Image.fromarray(image_uint8_array, 'RGB')

            # 保存图像
            image.save(file_path)

            logging.info(f"图像已成功保存到: {file_path}")
            return True

        except Exception as e:
            # 捕获任何可能发生的异常（如权限错误、磁盘已满等）
            logging.error(f"保存文件到 {file_path} 时发生未知错误: {e}", exc_info=True)
            return False
