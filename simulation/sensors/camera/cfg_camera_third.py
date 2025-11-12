# Standard library imports
from dataclasses import dataclass, field
from typing import List


@dataclass
class CfgCameraThird:
    enabled: bool = False
    # 相机相对于机器人的位置，主要用于确定角度
    relative_position: List[float] = field(default_factory=lambda: [-5.0, 0.0, 3.0])
    # 相机的朝向确定好后， 再进行一个水平的平移
    transform_position: List[float] = field(default_factory=lambda: [0, 0, 1])
    # 视口在屏幕上的x, y位置
    viewport_position: List[int] = field(default_factory=lambda: [800, 0])
    # 视口的宽度和高度
    viewport_size: List[int] = field(default_factory=lambda: [360, 360])
