# =============================================================================
# Config Target Module - Target Object Configuration
# =============================================================================
#
# This module provides configuration parameters for target objects used
# in robot interaction and tracking scenarios.
#
# =============================================================================

# Local project imports
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
    move_path: list = [(0, 0, 0), (11, 26, 0), (-11, 26, 0)]
