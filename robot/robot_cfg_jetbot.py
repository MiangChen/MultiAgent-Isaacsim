from typing import Optional

from robot.robot_cfg import RobotCfg, ASSET_PATH


class RobotCfgJetbot(RobotCfg):
    # meta info
    name_prefix: Optional[str] = 'jetbot'
    type: Optional[str] = 'jetbot'
    prim_path: Optional[str] = '/World/robot/jetbot'

    id: int = 0
    usd_path: Optional[str] = ASSET_PATH + "/Isaac/Robots/Jetbot/jetbot.usd"
    camera_path: Optional[str] = '/World/robot/jetbot/jetbot/jetbot_0/chassis/rgb_camera/jetbot_camera'