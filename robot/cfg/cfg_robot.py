from typing import Dict, Optional, Tuple

from robot.sensor.camera import CfgCamera, CfgCameraThird
from config.config_manager import config_manager
from config.cfg_base import CfgBase
from robot.sensor.lidar import CfgLidar

ASSET_PATH = config_manager.get("path_asset")


class CfgRobot(CfgBase):
    # meta info
    type: str = "robot"
    id: int = 0
    namespace: str = f"{type}_{id}"
    path_prim_swarm: str = "/World/robot"
    path_prim_robot: str = None
    path_usd: str = None

    cfg_dict_camera: Optional[Dict[str, CfgCamera]] = {}
    cfg_dict_lidar: Optional[Dict[str, CfgLidar]] = {}
    cfg_dict_camera_third: Optional[Dict[str, CfgCameraThird]] = {}
    # controllers: Optional[List[ControllerCfg]] = None

    # common config
    position: Optional[Tuple[float, float, float]] = (0.0, 0.0, 0.0)
    quat: Optional[Tuple[float, float, float, float]] = (
        1.0,
        0.0,
        0.0,
        0.0,
    )
    scale: Optional[Tuple[float, float, float]] = (1.0, 1.0, 1.0)
    base: Optional[Tuple[float, float, float]] = (5.0, 3.0, 0.0)
