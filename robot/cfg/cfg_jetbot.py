from .cfg_robot import CfgRobot, ASSET_PATH


class CfgJetbot(CfgRobot):
    # meta info
    type: str = "jetbot"
    id: int = 0
    prim_path_swarm: str = "/World/robot/jetbot"
    prim_path_robot: str = "/World/robot/jetbot"
    usd_path: str = ASSET_PATH + "/Isaac/Robots/Jetbot/jetbot.usd"
