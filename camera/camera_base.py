from typing import List, Optional, Tuple

from camera.camera_cfg import CameraCfg
from robot.robot_cfg import RobotCfg

import carb
from pxr import Usd, UsdGeom, Gf
from isaacsim.sensors.camera import Camera
from isaacsim.core.utils.numpy import rotations
from isaacsim.core.utils.prims import define_prim, get_prim_at_path

import numpy as np


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

        if not prim.IsValid():
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
            # Convert the prim to an Xformable object
            # xformable = UsdGeom.Xformable(prim)
            #
            # # Get the local-to-world transformation matrix
            # # CORRECTED METHOD NAME: ComputeLocalToWorldTransform
            # # You need to specify a time code.
            # timecode = Usd.TimeCode.Default()  # Use default time or simulation time
            # local_to_world_matrix = xformable.ComputeLocalToWorldTransform(timecode)
            #
            # # The local_to_world_matrix is a Gf.Matrix4d
            # # Extract the translation (position) from the matrix
            # self.cfg_camera.position = list(local_to_world_matrix.ExtractTranslation())  # Returns a Gf.Vec3d
            #
            # # Extract the rotation from the matrix
            # quat = local_to_world_matrix.ExtractRotationQuat()  # Returns a Gf.Quatd
            # self.cfg_camera.quat = [quat.real] + list(quat.imaginary)
            # self.cfg_camera.euler_degree = None
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

            self.camera = Camera(
                prim_path=self.cfg_camera.prim_path,
                frequency=self.cfg_camera.frequency,
                resolution=self.cfg_camera.resolution,
                translation=self.cfg_camera.position,
                orientation=self.cfg_camera.quat
            )
            print(self.cfg_camera.quat)
            self.set_local_pose(translation=self.cfg_camera.position, orientation=self.cfg_camera.quat,
                                camera_axes='usd')
        else:
            self.camera = Camera(
                prim_path=self.cfg_camera.prim_path
            )
        return

    def initialize(self):
        self.camera.initialize()
        # 设置深度功能
        self.camera.add_distance_to_camera_to_frame()
        # 物品检测功能
        self.camera.add_bounding_box_2d_loose_to_frame()

    def set_local_pose(self, translation: Tuple[float, float, float],
                       orientation: Tuple[float, float, float, float],
                       camera_axes: str = 'usd') -> None:
        self.camera.set_local_pose(translation=translation, orientation=orientation, camera_axes=camera_axes)
        return None

    def get_current_frame(self):
        return self.camera.get_current_frame()

    def get_depth(self):
        return self.camera.get_depth()

    def get_point_cloud(self):
        return self.camera.get_point_cloud()

    def get_rgb(self):
        return self.camera.get_rgb()

    def get_local_pose(self, camear_axes: str = 'usd'):
        return self.camera.get_local_pose(camera_axes=camera_axes)

    def get_world_pose(self, camera_axes: str = 'usd') -> Tuple[np.ndarray, np.ndarray]:
        return self.camera.get_world_pose(camera_axes=camera_axes)
