# =============================================================================
# Config Jetbot Module - Jetbot Robot Configuration
# =============================================================================
#
# This module provides configuration parameters specific to the Jetbot
# robot, including physical properties and sensor configurations.
#
# =============================================================================

# Local project imports
from .cfg_robot import CfgRobot, ASSET_PATH


class CfgJetbot(CfgRobot):
    # meta info
    type: str = "jetbot"
    id: int = 0
    path_prim_swarm: str = "/World/robot"
    path_prim_robot: str = "/World/robot/jetbot"
    path_usd: str = ASSET_PATH + "/Isaac/Robots/Jetbot/jetbot.usd"
    detection_radius: float = 1.0
    robot_radius: float = 0.2
