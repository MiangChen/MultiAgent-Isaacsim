# =============================================================================
# Camera Module - Camera Sensor Implementation
# =============================================================================
#
# This module provides camera sensor implementation with image capture,
# processing, and integration with Isaac Sim's rendering pipeline.
#
# =============================================================================

# Standard library imports
from typing import List, Optional, Tuple, Sequence

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

    def get_rgb(self) -> np.ndarray:
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
            self, rgb: np.ndarray, file_path: str = None
    ) -> str:
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

            Image.fromarray(rgb, 'RGB').save(file_path)

            return f"图像已成功保存到: {file_path}"
        except Exception as e:
            return f"Error: {repr(e)}"
