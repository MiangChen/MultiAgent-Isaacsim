from dataclasses import dataclass

# Local project imports
from .cfg_robot import CfgRobot, ASSET_PATH


@dataclass
class CfgJetbot(CfgRobot):
    type: str = "jetbot"
    path_prim_robot: str = "/World/robot/jetbot"
    path_usd: str = ASSET_PATH + "/Isaac/Robots/Jetbot/jetbot.usd"
    detection_radius: float = 1.0
    robot_radius: float = 0.2
