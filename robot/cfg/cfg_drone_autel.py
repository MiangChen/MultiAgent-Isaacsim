from .cfg_robot import CfgRobot, ASSET_PATH


class CfgDroneAutel(CfgRobot):
    # meta info
    type: str = "auteldrone"
    id: int = 0
    path_prim_swarm: str = "/World/robot"
    path_prim_robot: str = "/World/robot/drone"
    path_usd: str = None
