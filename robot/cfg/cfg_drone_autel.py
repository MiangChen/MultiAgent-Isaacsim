from dataclasses import dataclass

# Local project imports
from .cfg_robot import CfgRobot, ASSET_PATH


@dataclass
class CfgDroneAutel(CfgRobot):
    type: str = "drone_autel"
    path_prim_robot: str = "/World/robot/drone_autel"
    path_usd: str = None
    # todo 换成autel的无人机模型
    path_usd: str = ASSET_PATH + "/Isaac/Robots/Crazyflie/cf2x.usd"

    default_speed: float = 1.0
    takeoff_height: float = 1.0
    land_height: float = 0.0
    control_mode: str = "velocity"
    detection_radius: float = 1.0
    robot_radius: float = 0.2
