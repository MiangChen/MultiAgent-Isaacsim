# =============================================================================
# Config H1 Module - H1 Humanoid Robot Configuration
# =============================================================================
#
# This module provides configuration parameters specific to the H1
# humanoid robot, including locomotion and sensor settings.
#
# =============================================================================

# Local project imports
from .cfg_robot import CfgRobot, ASSET_PATH


class CfgH1(CfgRobot):
    # meta info
    type: str = "h1"
    id: int = 0
    path_prim_swarm: str = "/World/robot"
    path_prim_robot: str = "/World/robot/h1"
    path_usd: str = ASSET_PATH + "/Isaac/Robots/Unitree/H1/h1.usd"
