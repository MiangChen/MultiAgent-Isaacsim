from dataclasses import dataclass

from .cfg_robot import CfgRobot, ASSET_PATH


@dataclass
class CfgDroneCf2X(CfgRobot):
    type: str = "cf2x"
    path_prim_robot: str = "/World/robot/cf2x"
    path_usd: str = ASSET_PATH + "/Isaac/Robots/Crazyflie/cf2x.usd"

    default_speed: float = 1.0
    takeoff_height: float = 1.0
    land_height: float = 0.0
    control_mode: str = "velocity"
    detection_radius: float = 1.0
    robot_radius: float = 0.2
