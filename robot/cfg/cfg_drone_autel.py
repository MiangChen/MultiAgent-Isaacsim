# =============================================================================
# Config Drone Autel Module - Autel Drone Configuration
# =============================================================================
#
# This module provides configuration parameters specific to the Autel
# drone, including flight parameters and sensor configurations.
#
# =============================================================================

# Local project imports
from .cfg_robot import CfgRobot, ASSET_PATH


class CfgDroneAutel(CfgRobot):
    # meta info
    type: str = "auteldrone"
    id: int = 0
    path_prim_swarm: str = "/World/robot"
    path_prim_robot: str = "/World/robot/drone"
    path_usd: str = None
