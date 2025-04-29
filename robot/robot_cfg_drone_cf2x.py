from typing import Optional

from robot.robot_cfg import RobotCfg, assets_root_path


class RobotCfgCf2x(RobotCfg):
    # meta info
    name_prefix: Optional[str] = 'cf2x'
    type: Optional[str] = 'cf2x'
    prim_path: Optional[str] = '/World/robot/cf2x'

    id: int = 0
    usd_path: Optional[str] = assets_root_path + "/Isaac/Robots/Crazyflie/cf2x.usd"
