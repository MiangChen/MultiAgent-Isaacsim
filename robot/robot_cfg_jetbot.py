from typing import Optional

from robot.robot_cfg import RobotCfg, assets_root_path


class RobotCfgJetbot(RobotCfg):
    # meta info
    name_prefix: Optional[str] = 'jetbot'
    type: Optional[str] = 'jetbot'
    prim_path: Optional[str] = '/World/robot/jetbot'

    id: int = 0
    usd_path: Optional[str] = assets_root_path + "/Isaac/Robots/Jetbot/jetbot.usd"
