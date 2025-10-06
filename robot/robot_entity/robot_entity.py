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

class RobotEntity():
    def __init__(
        self,
        cfg_body: RobotCfg = None,
        scene: Scene = None,):
        # 代表机器人的实体
        self.cfg_body = cfg_body
        self.robot_entity: Articulation = None

    def create_robot_entity(self):
        """
        初始化机器人关节树
        """
        self.robot_entity = Articulation(
            prim_paths_expr=self.cfg_body.prim_path,
            name=self.cfg_body.name,
            positions=to_torch(self.cfg_body.position).reshape(1, 3),
            orientations=to_torch(self.cfg_body.quat).reshape(1, 4),
        )
