from typing import Tuple

import torch

from isaacsim.core.api.scenes import Scene
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.rotations import quat_to_rot_matrix

from log.log_manager import LogManager
from robot.cfg import CfgRobot
from utils import to_torch

logger = LogManager.get_logger(__name__)


class BodyRobot:
    def __init__(
        self,
        cfg_robot: CfgRobot = None,
        scene: Scene = None,
    ):
        self.cfg_robot = cfg_robot
        self.robot_articulation: Articulation = None
        self.scene = scene

    def create_robot_body(self):
        """
        [Abstract Method] Initializes the specific robot entity wrapper.
        Subclasses MUST override this method to create either an Articulation,
        a RigidPrim, or another appropriate wrapper and assign it to self.robot_entity.
        """
        raise NotImplementedError

    def get_world_poses(self) -> Tuple[torch.Tensor, torch.Tensor]:
        raise NotImplementedError
