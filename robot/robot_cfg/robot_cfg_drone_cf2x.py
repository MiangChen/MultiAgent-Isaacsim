from typing import Optional

from .robot_cfg import RobotCfg, ASSET_PATH


class RobotCfgCf2x(RobotCfg):
    # meta info
    name_prefix: Optional[str] = "cf2x"
    type: Optional[str] = "cf2x"
    prim_path: Optional[str] = "/World/robot/cf2x"

    id: int = 0
    usd_path: Optional[str] = ASSET_PATH + "/Isaac/Robots/Crazyflie/cf2x.usd"

    default_speed: float = 1  # m/s
    takeoff_height: float = 1.0  # 起飞悬停高度，单位米
    land_height: float = 0.0  # 降落高度，地面
    control_mode: str = "velocity"  # "velocity" or "teleport"
