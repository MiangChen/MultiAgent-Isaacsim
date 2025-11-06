# Standard library imports
from dataclasses import dataclass, field
from typing import Dict, Optional, Tuple

# Local project imports
from config.config_manager import config_manager
from robot.sensor.camera import CfgCamera, CfgCameraThird
from robot.sensor.lidar import CfgLidar

ASSET_PATH = config_manager.get("path_asset")


@dataclass
class CfgRobot:
    type: str = "robot"
    id: int = 0
    namespace: str = "robot"
    path_prim_swarm: str = "/World/robot"
    path_prim_robot: Optional[str] = None
    path_usd: Optional[str] = None

    cfg_dict_camera: Dict[str, CfgCamera] = field(default_factory=dict)
    cfg_dict_lidar: Dict[str, CfgLidar] = field(default_factory=dict)
    cfg_dict_camera_third: Dict[str, CfgCameraThird] = field(default_factory=dict)

    position: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    quat: Tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
    scale: Tuple[float, float, float] = (1.0, 1.0, 1.0)
    base: Tuple[float, float, float] = (5.0, 3.0, 0.0)

    disable_gravity: bool = False # 这个参数通常是在不使用物理引擎动力学的条件下用
    use_simplified_controller: bool = False
