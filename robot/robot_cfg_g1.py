from typing import Optional

from robot.robot_cfg import RobotCfg, ASSET_PATH

class RobotCfgG1(RobotCfg):
    # meta info
    name_prefix: Optional[str] = 'g1'
    type: Optional[str] = 'g1'
    prim_path: Optional[str] = '/World/robot/g1'

    id: int = 0
    usd_path: Optional[str] = ASSET_PATH + "/Isaac/Robots/G1/g1.usd"

