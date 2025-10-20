import carb
from physics_engine.isaacsim_utils import (
    Scene,
    Articulation,
    define_prim,
    get_prim_at_path,
)

from robot.body import BodyRobot
from robot.cfg import CfgRobot
from utils import to_torch
from log.log_manager import LogManager

logger = LogManager.get_logger(__name__)


class BodyDroneAutel(BodyRobot):
    def __init__(
        self,
        cfg_robot: CfgRobot = None,
        scene: Scene = None,
    ):
        super().__init__(cfg_robot=cfg_robot, scene=scene)
