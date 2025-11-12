# =============================================================================
# Body Robot Module - Base Robot Body Implementation
# =============================================================================
#
# This module provides the base robot body class with common functionality
# for robot physical representation and control within Isaac Sim.
#
# =============================================================================

# Standard library imports
from typing import Tuple

# Third-party library imports
import torch

# Local project imports
from log.log_manager import LogManager
from physics_engine.isaacsim_utils import Scene, Articulation, quat_to_rot_matrix
from simulation.robot.cfg import CfgRobot
from utils import to_torch

logger = LogManager.get_logger(__name__)


class BodyRobot:
    def __init__(
        self,
        cfg_robot: CfgRobot = None,
    ):
        self.cfg_robot = cfg_robot
        self.robot_articulation: Articulation = None

    def create_robot_body(self):
        """
        [Abstract Method] Initializes the specific robot entity wrapper.
        Subclasses MUST override this method to create either an Articulation,
        a RigidPrim, or another appropriate wrapper and assign it to self.robot_entity.
        """
        raise NotImplementedError

    def get_world_pose(self) -> Tuple[torch.Tensor, torch.Tensor]:
        """
        quat: wxyz
        """
        pos_IB, q_IB = self.robot_articulation.get_world_poses()
        pos_IB, q_IB = pos_IB[0], q_IB[0]
        pos_IB = to_torch(pos_IB)
        q_IB = to_torch(q_IB)
        return pos_IB, q_IB

    def get_world_vel(self) -> Tuple[torch.Tensor, torch.Tensor]:
        """
        vel_linear:  shape [3,]
        vel_angular: shape [3,]
        """
        vel_linear = self.robot_articulation.get_linear_velocities()[0]
        vel_angular = self.robot_articulation.get_angular_velocities()[0]
        vel_linear = to_torch(vel_linear)
        vel_angular = to_torch(vel_angular)
        return vel_linear, vel_angular
