from dataclasses import dataclass

from .cfg_robot import CfgRobot, ASSET_PATH


@dataclass
class CfgH1(CfgRobot):
    type: str = "h1"
    path_prim_robot: str = "/World/robot/h1"
    path_usd: str = ASSET_PATH + "/Isaac/Robots/Unitree/H1/h1.usd"
