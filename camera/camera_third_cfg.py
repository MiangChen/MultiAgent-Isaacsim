# In a file like camera/camera_cfg.py
from pydantic import BaseModel, Field
from typing import List, Optional


class CameraThirdCfg(BaseModel):
    enabled: bool = Field(default=False, description="是否启用此相机")
    relative_position: List[float] = Field(
        default=[-5.0, 0.0, 3.0], description="相机相对于机器人的位置，主要用于确定角度"
    )
    transform_position: List[float] = Field(
        default=[0, 0, 1], description="相机的朝向确定好后， 再进行一个水平的平移"
    )
    viewport_position: List[int] = Field(
        default=[800, 0], description="视口在屏幕上的x, y位置"
    )
    viewport_size: List[int] = Field(default=[360, 360], description="视口的宽度和高度")
