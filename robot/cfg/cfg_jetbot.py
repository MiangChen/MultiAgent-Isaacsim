from .cfg_robot import CfgRobot, ASSET_PATH


class CfgJetbot(CfgRobot):
    # meta info
    type: str = "jetbot"
    id: int = 0
    path_prim_swarm: str = "/World/robot"
    path_prim_robot: str = "/World/robot/jetbot"
    path_usd: str = ASSET_PATH + "/Isaac/Robots/Jetbot/jetbot.usd"
