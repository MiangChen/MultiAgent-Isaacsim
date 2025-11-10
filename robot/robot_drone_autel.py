# =============================================================================
# Robot Drone Autel Module - Autel Drone Implementation
# =============================================================================
#
# This module provides the Autel drone robot implementation with advanced
# flight control, multi-camera systems, and autonomous navigation capabilities.
#
# =============================================================================

# Standard library imports
import threading
from dataclasses import dataclass, field
from typing import Dict

# Third-party library imports
import numpy as np

# Local project imports
from physics_engine.pxr_utils import Gf, UsdGeom
from physics_engine.isaacsim_utils import Scene
from robot.cfg import CfgDroneAutel
from scene.scene_manager import SceneManager

from robot.robot import Robot
from robot.body.body_drone_autel import BodyDroneAutel

# ROS2 imports
from rclpy.node import Node

DRONE_COLOR_SCHEMES = {
    "Gray": {"main": [0.6, 0.6, 0.6], "nose": [0.3, 0.3, 0.3]},
    "Red": {"main": [0.8, 0.2, 0.2], "nose": [0.9, 0.4, 0.4]},
    "Blue": {"main": [0.2, 0.4, 0.8], "nose": [0.4, 0.6, 0.9]},
    "Green": {"main": [0.2, 0.8, 0.3], "nose": [0.4, 0.9, 0.5]},
    "Orange": {"main": [0.8, 0.6, 0.2], "nose": [0.9, 0.7, 0.4]},
    "Purple": {"main": [0.7, 0.2, 0.8], "nose": [0.8, 0.4, 0.9]},
    "Cyan": {"main": [0.2, 0.8, 0.8], "nose": [0.4, 0.9, 0.9]},
    "Yellow": {"main": [0.8, 0.8, 0.2], "nose": [0.9, 0.9, 0.4]},
}

DRONE_COLOR_NAMES = list(DRONE_COLOR_SCHEMES.keys())

DRONE_GEOMETRY_TEMPLATE = {
    "MainBody": {
        "shape_type": "cuboid",
        "position": [0.0, 0.0, 0.0],
        "scale": [0.12, 0.06, 0.06],
        "orientation": [1.0, 0.0, 0.0, 0.0],
        "color_type": "main",  # 指定使用哪种颜色
    },
    "Nose": {
        "shape_type": "cuboid",
        "position": [0.08, 0.0, 0.0],
        "scale": [0.03, 0.02, 0.02],
        "orientation": [1.0, 0.0, 0.0, 0.0],
        "color_type": "nose",  # 指定使用哪种颜色
    },
}


class DronePose:
    def __init__(self, pos=Gf.Vec3d(0, 0, 0), quat=Gf.Quatf.GetIdentity()):
        self.pos = pos
        self.quat = quat


class RobotDroneAutel(Robot):
    def __init__(self, cfg_robot: Dict = {}, scene_manager: SceneManager = None):
        # Extract color_scheme_id before passing to CfgDroneAutel
        self.color_scheme_id = cfg_robot.pop('color_scheme_id', 0)
        
        self.cfg_robot = CfgDroneAutel(**cfg_robot)
        super().__init__(scene_manager=scene_manager)
        
        self._body = BodyDroneAutel(cfg_robot=self.cfg_robot)
        
        # ROS2
        self.ros_node = None
        self.pubs = {}
        self.subs = {}
        self.srvs = {}
        
        # LiDAR sensors
        self.lidar_list = None
        
        # Desired pose tracking
        self.des_pose = None
        self.is_pose_dirty = False
        
        # Locks & queues
        self.pending_spawn_requests = []
        self.spawn_lock = threading.Lock()
        self.pending_move_requests = []
        self.move_lock = threading.Lock()
    
    def initialize(self):
        # if self.cfg_robot.disable_gravity:
        self.scene_manager.disable_gravity_for_hierarchy(self.cfg_robot.path_prim_robot)
    
    def setup_lidar_and_ros(self, setup_ros_fn):
        """Setup LiDAR sensors and ROS after creation"""
        from robot.sensor.lidar.lidar_omni import LidarOmni
        from robot.sensor.lidar.cfg_lidar import CfgLidar
        
        prim_path = self.cfg_robot.path_prim_robot
        
        cfg_lfr = CfgLidar(name="lfr", prim_path=prim_path + "/lfr", output_size=(352, 120),
                          quat=(1, 0, 0, 0), config_file_name="autel_perception_120x352")
        cfg_ubd = CfgLidar(name="ubd", prim_path=prim_path + "/ubd", output_size=(352, 120),
                          quat=(0, 0, 0.7071067811865476, 0.7071067811865476),
                          config_file_name="autel_perception_120x352")
        
        self.lidar_list = [LidarOmni(cfg_lidar=cfg_lfr), LidarOmni(cfg_lidar=cfg_ubd)]
        
        node, pubs, subs, srvs = setup_ros_fn(self.namespace, ctx=self)
        self.ros_node = node
        self.pubs = pubs
        self.subs = subs
        self.srvs = srvs
    
    @property
    def drone_prim(self):
        from physics_engine.isaacsim_utils import get_prim_at_path
        return get_prim_at_path(self.cfg_robot.path_prim_robot)
    
    @property
    def prim_path(self):
        return self.cfg_robot.path_prim_robot

        # sensor
        self.lidar = None

    def create_robot_entity(self, prim_path: str = None):
        """
        内部辅助函数，负责创建无人机的程序化视觉模型。
        这是从 asset_creator.py/add_drone_body 移过来的逻辑。
        """
        drone_xform = UsdGeom.Xform.Define(self.scene_manager.stage, prim_path)
        drone_xform.AddTranslateOp().Set(Gf.Vec3d(0, 0, 0))
        drone_xform.AddOrientOp().Set(Gf.Quatf.GetIdentity())
        drone_prim = drone_xform.GetPrim()

        # 根据索引选择颜色方案
        scheme_idx = self.color_scheme_id % len(DRONE_COLOR_NAMES)
        scheme_name = DRONE_COLOR_NAMES[scheme_idx]
        colors = DRONE_COLOR_SCHEMES[scheme_name]

        # 循环遍历几何模板，并创建每个组件
        for name, template in DRONE_GEOMETRY_TEMPLATE.items():
            component_data = template.copy()
            component_data["color"] = colors[template["color_type"]]
            del component_data["color_type"]
            kwargs = {
                "name": name,
                "prim_path": f"{prim_path}/{name}",
                "entity_type": "visual",
                **component_data,
            }
            self.scene_manager.create_shape_unified(**kwargs)
        print("robot drone", drone_prim)
        return drone_prim

    def on_physics_step(self, step_size):
        super().on_physics_step(step_size)
        # self._publish_status_pose()  # 例如，每一步都发布状态

    def step(self, action: np.ndarray):
        # 无人机的动作控制逻辑 (例如，应用推力)
        pass  # raise NotImplementedError
