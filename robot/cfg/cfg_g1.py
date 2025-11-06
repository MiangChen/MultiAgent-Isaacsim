from dataclasses import dataclass

from .cfg_robot import CfgRobot, ASSET_PATH


@dataclass
class CfgG1(CfgRobot):
    type: str = "g1"
    path_prim_robot: str = "/World/robot/g1"
    path_usd: str = ASSET_PATH + "/Isaac/Robots/Unitree/G1/g1.usd"
    detection_radius: float = 1.0
    robot_radius: float = 0.5
