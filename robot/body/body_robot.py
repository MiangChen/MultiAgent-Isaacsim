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

    def get_world_pose(self) -> Tuple[torch.Tensor, torch.Tensor]:
        pos_IB, q_IB = self.robot_articulation.get_world_pose()
        pos_IB, q_IB = pos_IB[0], q_IB[0]
        pos_IB = to_torch(pos_IB, device=pos_IB.device)
        q_IB = to_torch(q_IB, device=q_IB.device)
        return pos_IB, q_IB

    def get_world_vel(self) -> Tuple[torch.Tensor, torch.Tensor]:
        lin_vel = to_torch(self.robot_articulation.get_linear_velocities())
        ang_vel = to_torch(self.robot_articulation.get_angular_velocities())
        return lin_vel, ang_vel
