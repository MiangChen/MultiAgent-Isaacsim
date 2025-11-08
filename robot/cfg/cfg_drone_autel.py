from dataclasses import dataclass

# Local project imports
from .cfg_robot import CfgRobot, ASSET_PATH


@dataclass
class CfgDroneAutel(CfgRobot):
    type: str = "auteldrone"
    path_prim_robot: str = "/World/robot/drone"
    path_usd: str = None
    # todo 换成autel的无人机模型
    path_usd: str = ASSET_PATH + "/Isaac/Robots/Crazyflie/cf2x.usd"
