from .cfg_robot import CfgRobot, ASSET_PATH


class CfgH1(CfgRobot):
    # meta info
    type: str = "h1"
    id: int = 0
    path_prim_swarm: str = "/World/robot"
    path_prim_robot: str = "/World/robot/h1"
    path_usd: str = ASSET_PATH + "/Isaac/Robots/Unitree/H1/h1.usd"
