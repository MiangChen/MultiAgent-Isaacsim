from typing import Optional, List, Dict

from .robot_cfg import RobotCfg, ASSET_PATH


class RobotCfgH1(RobotCfg):
    # meta info
    name_prefix: Optional[str] = "h1"
    type: Optional[str] = "h1"
    prim_path: Optional[str] = "/World/robot/h1"

    id: int = 0
    usd_path: Optional[str] = ASSET_PATH + "/Isaac/Robots/Unitree/H1/h1.usd"
    relative_camera_path: Optional[List[str]] = None
    camera_path: Optional[List[str]] = None
