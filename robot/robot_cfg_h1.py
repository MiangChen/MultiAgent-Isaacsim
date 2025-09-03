from typing import Optional, List, Dict

from robot.robot_cfg import RobotCfg

from config.variables import ASSET_PATH


class RobotCfgH1(RobotCfg):
    # meta info
    name_prefix: Optional[str] = 'h1'
    type: Optional[str] = 'h1'

    id: int = 0
    usd_path: Optional[str] = ASSET_PATH + "/Isaac/Robots/Unitree/H1/h1.usd"

