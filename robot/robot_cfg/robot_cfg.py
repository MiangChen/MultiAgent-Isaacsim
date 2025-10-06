from typing import Dict, Optional, Tuple
from pydantic import BaseModel

from camera.camera_cfg import CameraCfg
from camera.camera_third_cfg import CameraThirdCfg
from config.config_manager import config_manager

ASSET_PATH = config_manager.get("asset_path")


class BaseCfg(BaseModel):
    def update(self, **kwargs):
        return self.model_copy(update=kwargs, deep=True)


class RobotCfg(BaseCfg):
    # meta info
    name: str = ""
    type: str = "robot"
    prim_path: str = "/World/robot"
    path_prim_robot: Optional[str] = "/World/robot"
    usd_path: str = None

    # common config
    position: Optional[Tuple[float, float, float]] = (0.0, 0.0, 0.0)
    quat: Optional[Tuple[float, float, float, float]] = (
        0.0,
        0.0,
        0.0,
        1.0,
    )  # 使用4元数
    scale: Optional[Tuple[float, float, float]] = (1.0, 1.0, 1.0)
    # controllers: Optional[List[ControllerCfg]] = None
    # cameras: Optional[List[SensorCfg]] = None

    camera: Optional[Dict[str, CameraCfg]] = {}
