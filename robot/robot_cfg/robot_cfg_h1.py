from .robot_cfg import RobotCfg, ASSET_PATH


class RobotCfgH1(RobotCfg):
    # meta info
    type: str = "h1"
    id: int = 0
    prim_path_swarm: str = "/World/robot/h1"
    prim_path_robot: str = "/World/robot/h1"
    usd_path: str = ASSET_PATH + "/Isaac/Robots/Unitree/H1/h1.usd"
