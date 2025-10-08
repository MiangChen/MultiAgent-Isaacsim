from typing import Optional, Tuple

from config.cfg_base import CfgBase


class CfgCamera(CfgBase):

    type: str = "camera"
    path_prim_relative: str = None
    path_prim_absolute: str = None
    path_prim_joint_target_relative: str = None
    path_prim_joint_target_absolute: str = None
    position: Tuple[float, float, float] = (
        0.0,
        0.0,
        0.0,
    )  # 相对于joint target的偏移距离
    quat: Tuple[float, float, float, float] = (
        0.0,
        0.0,
        0.0,
        1.0,
    )
    scale: Optional[Tuple[float, float, float]] = (1.0, 1.0, 1.0)
    frequency: Optional[int] = 30  # 相机帧率
    resolution: Optional[Tuple[int, int]] = None  # 相机分辨率
    focal_length: Optional[int] = 30  # 相机的焦段 mm
    lens_aperture: Optional[float] = 0.0  # 相机的光圈 0 表示没有景深 全部清晰
    focus_distance: Optional[float] = 0.0  # 相机的对焦距离
