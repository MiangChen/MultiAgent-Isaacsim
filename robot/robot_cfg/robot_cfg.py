from typing import Dict, Optional, Tuple
from pydantic import BaseModel

from camera.camera_cfg import CameraCfg
from camera.camera_third_cfg import CameraThirdCfg
from config.config_manager import config_manager
from config.cfg_base import CfgBase
from lidar.lidar_cfg import LidarCfg

ASSET_PATH = config_manager.get("asset_path")



class RobotCfg(CfgBase):
    # meta info
    type: str = "robot"
    name: str = "robot"
    id: int = 0
    prim_path_swarm: str = "/World/robot"
    prim_path_robot: Optional[str] = "/World/robot"
    usd_path: str = None

    cfg_dict_camera: Optional[Dict[str, CameraCfg]] = {}
    cfg_dict_lidar: Optional[Dict[str, LidarCfg]] = {}
    cfg_dict_camera_third: Optional[Dict[str, CameraThirdCfg]] = {}
    # controllers: Optional[List[ControllerCfg]] = None

    # common config
    position: Optional[Tuple[float, float, float]] = (0.0, 0.0, 0.0)
    quat: Optional[Tuple[float, float, float, float]] = (
        0.0,
        0.0,
        0.0,
        1.0,
    )
    scale: Optional[Tuple[float, float, float]] = (1.0, 1.0, 1.0)
