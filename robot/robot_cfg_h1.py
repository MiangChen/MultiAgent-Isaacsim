from typing import Optional, List

from robot.robot_cfg import RobotCfg, assets_root_path


class RobotCfgH1(RobotCfg):
    # meta info
    name_prefix: Optional[str] = 'h1'
    type: Optional[str] = 'h1'
    prim_path: Optional[str] = '/World/robot/h1'

    id: int = 0
    usd_path: Optional[str] = assets_root_path + "/Isaac/Robots/Unitree/H1/h1.usd"
    position: List = [0, 0, 1.05]

