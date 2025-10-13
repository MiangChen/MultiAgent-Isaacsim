from .cfg_robot import CfgRobot, ASSET_PATH


class CfgG1(CfgRobot):
    # meta info
    type: str = "g1"
    id: int = 0
    path_prim_swarm: str = "/World/robot"
    path_prim_robot: str = "/World/robot/g1"
    path_usd: str = ASSET_PATH + "/Isaac/Robots/Unitree/G1/g1.usd"
