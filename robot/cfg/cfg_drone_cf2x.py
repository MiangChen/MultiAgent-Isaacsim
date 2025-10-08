from .cfg_robot import CfgRobot, ASSET_PATH


class CfgDroneCf2X(CfgRobot):
    # meta info
    type: str = "cf2x"
    id: int = 0
    path_prim_swarm: str = "/World/robot"
    path_prim_robot: str = "/World/robot/cf2x"
    path_usd: str = ASSET_PATH + "/Isaac/Robots/Crazyflie/cf2x.usd"

    default_speed: float = 1  # m/s
    takeoff_height: float = 1.0  # 起飞悬停高度，单位米
    land_height: float = 0.0  # 降落高度，地面
    control_mode: str = "velocity"  # "velocity" or "teleport"
