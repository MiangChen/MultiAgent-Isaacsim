from typing import List, Tuple

from rclpy.action import ActionServer
from pxr import Usd, UsdGeom
import numpy as np
import torch
import carb

from isaacsim.core.utils.numpy import rotations
from isaacsim.core.api.scenes import Scene
from isaacsim.core.prims import Articulation
from isaacsim.core.prims import RigidPrim
from isaacsim.core.utils.rotations import quat_to_rot_matrix
import isaacsim.core.utils.prims as prims_utils
from isaacsim.core.utils.prims import define_prim, get_prim_at_path
from isaacsim.core.utils.viewports import (
    create_viewport_for_camera,
    set_camera_view,
    set_intrinsics_matrix,
)
from utils import to_torch

from map.map_grid_map import GridMap
from path_planning.path_planning_astar import AStar
from camera.camera_base import CameraBase
from camera.camera_cfg import CameraCfg
from camera.camera_third_cfg import CameraThirdCfg
from robot.robot_cfg import RobotCfg
from robot.robot_trajectory import Trajectory
from ros.node_robot import NodeRobot
from scene.scene_manager import SceneManager
from log.log_manager import LogManager
from gsi2isaacsim.gsi_msgs_helper import (
    PrimTransform,
    SceneModifications,
    RobotFeedback,
    VelTwistPose,
    RobotSkill,
    PlanExecution,
    SkillExecution,
    SkillFeedback,
    Plan,
    SkillInfo,
    Parameter,
    VelTwistPose,
)

logger = LogManager.get_logger(__name__)


class RobotBody:
    def __init__(
        self,
        cfg_body: RobotCfg = None,
        scene: Scene = None,
    ):
        self.cfg_body = cfg_body
        self.robot_body: Articulation = None
        self.scene = scene

    def create_robot_body(self):
        """
        [Abstract Method] Initializes the specific robot entity wrapper.
        Subclasses MUST override this method to create either an Articulation,
        a RigidPrim, or another appropriate wrapper and assign it to self.robot_entity.
        """
        raise NotImplementedError
