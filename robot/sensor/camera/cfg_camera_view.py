# Standard library imports
from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass
class CfgCamera:
    type: str = "camera"
    path_prim_relative: Optional[str] = None
    path_prim_absolute: Optional[str] = None
    path_prim_joint_target_relative: Optional[str] = None
    path_prim_joint_target_absolute: Optional[str] = None
    # 相对于joint target的偏移距离
    position: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    quat: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    scale: Tuple[float, float, float] = (1.0, 1.0, 1.0)
    # 相机帧率
    frequency: int = 30
    # 相机分辨率 width x height
    resolution: Optional[Tuple[int, int]] = None
    # 相机的焦段 mm
    focal_length: int = 30
    # 相机的光圈 0 表示没有景深 全部清晰
    lens_aperture: float = 0.0
    # 相机的对焦距离
    focus_distance: float = 0.0