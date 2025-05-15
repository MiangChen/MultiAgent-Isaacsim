from typing import List, Optional, Tuple

from camera.camera_cfg import CameraCfg
from robot.robot_cfg import RobotCfg
from isaacsim.sensors.camera import Camera
from isaacsim.core.utils.numpy import rotations
import numpy as np


class CameraBase:

    def __init__(self, cfg_body: RobotCfg, cfg_camera: CameraCfg):
        self.cfg_camera = cfg_camera
        self.cfg_body = cfg_camera
        # 配置相机角度必须是4元数
        if cfg_camera.euler_degree is not None:
            # 注意角度和弧度模式
            cfg_camera.quat = rotations.euler_angles_to_quats(np.array(cfg_camera.euler_degree), degrees=True)
        else:
            cfg_camera.euler_degree = rotations.quat_to_euler_angles(np.array(cfg_camera.quat), degreeds=True)

    def create_camera(self):
        # 设置姿态
        self.camera = Camera(
            prim_path=self.cfg_camera.prim_path,
            # position=np.array(self.cfg_camera.position), # position单独设置
            # orientation=self.cfg_camera.quat,
            frequency=self.cfg_camera.frequency,
            resolution=self.cfg_camera.resolution,
        )
        self.set_local_pose(translation=self.cfg_camera.position, orientation=self.cfg_camera.quat)
        return

    def initialize(self):
        self.camera.initialize()
        # 设置深度功能
        self.camera.add_distance_to_camera_to_frame()
        # 物品检测功能
        self.camera.add_bounding_box_2d_loose_to_frame()



    def set_local_pose(self, translation: Tuple[float, float, float],
                       orientation: Tuple[float, float, float, float]) -> None:
        self.camera.set_local_pose(translation=translation, orientation=orientation)
        return None

    def get_current_frame(self):
        return self.camera.get_current_frame()

    def get_depth(self):
        return self.camera.get_depth()

    def get_point_cloud(self):
        return self.camera.get_point_cloud()

    def get_rgb(self):
        return self.camera.get_rgb()

    def get_world_pose(self) -> Tuple[np.ndarray, np.ndarray]:
        return self.camera.get_world_pose()
