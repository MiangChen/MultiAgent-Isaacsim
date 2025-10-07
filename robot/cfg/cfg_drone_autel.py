from .cfg_robot import CfgRobot, ASSET_PATH


class CfgDroneAutel(CfgRobot):
    # meta info
    type: str = "auteldrone"
    id: int = 0
    prim_path_swarm: str = "/World/robot"
    prim_path_robot: str = "/World/robot/drone"
    usd_path: str = None
