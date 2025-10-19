from .cfg_robot import CfgRobot, ASSET_PATH


class CfgTarget(CfgRobot):
    # meta info
    type: str = "target"
    id: int = 0
    path_prim_swarm: str = "/World/target"
    path_prim_robot: str = "/World/robot/car1"
    path_usd: str = (
        ASSET_PATH + "/Isaac/Robots/Jetbot/jetbot.usd"
    )  # 先用jetbot的模型来当目标
    robot_radius: float = 0.2
    # 起点-中点-终点-起点 的循环运动来躲避追踪
    base_pos: tuple = (0, 0, 0)
    mid_pos: tuple = (0, 0, 0)
    target_pos: tuple = (0, 0, 0)  # FIXME:语义地图做出来后应该在语义地图里选两个点
