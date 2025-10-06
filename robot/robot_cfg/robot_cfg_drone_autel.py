from .robot_cfg import RobotCfg, ASSET_PATH


class RobotCfgAutelDrone(RobotCfg):
    # meta info
    type: str = "auteldrone"
    id: int = 0
    prim_path_swarm: str = "/World/robot"
    prim_path_robot: str = "/World/robot/drone"
    usd_path: str = None
