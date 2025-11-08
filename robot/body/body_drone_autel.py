# =============================================================================
# Body Drone Autel Module - Autel Drone Body Implementation
# =============================================================================
#
# This module provides the Autel drone body implementation with flight
# dynamics, sensor integration, and control interfaces.
#
# =============================================================================

# Third-party library imports
import carb

# Local project imports
from log.log_manager import LogManager
from physics_engine.isaacsim_utils import (
    Scene,
    Articulation,
    define_prim,
    get_prim_at_path,
)
from robot.body import BodyRobot
from robot.cfg import CfgRobot
from utils import to_torch

logger = LogManager.get_logger(__name__)


class BodyDroneAutel(BodyRobot):
    def __init__(
        self,
        cfg_robot: CfgRobot = None,
    ):
        super().__init__(cfg_robot=cfg_robot)
