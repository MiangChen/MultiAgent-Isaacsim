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

from map.map_grid_map import GridMap
from physics_engine.pxr_utils import Gf, UsdGeom
from physics_engine.isaacsim_utils import Scene
from robot.cfg import CfgRobot, CfgDroneAutel
from robot.robot_trajectory import Trajectory
from robot.body.body_drone_autel import BodyDroneAutel
from robot.sensor.camera import CfgCamera, CfgCameraThird
from scene.scene_manager import SceneManager
# from robot.robot import Robot

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


class RobotDroneAutel:
    """
    一个完整的无人机机器人实例，包含了模型创建、ROS接口和传感器逻辑。
    """

    def __init__(
        self,
        cfg_robot: Dict = {},
        # cfg_camera: CfgCamera = None,
        # cfg_camera_third_person: CfgCameraThird = None,
        scene: Scene = None,
        map_grid: GridMap = None,
        scene_manager: SceneManager = None,
        namespace: str = None,
        prim_path: str = None,
        color_scheme_id: int = 0,  # 新增参数，用于选择颜色
    ):
        self.cfg_robot = CfgDroneAutel(**cfg_robot)
        # super().__init__(
        #     cfg_robot=cfg_robot,
        #     cfg_camera=cfg_camera,
        #     cfg_camera_third_person=cfg_camera_third_person,
        #     scene=scene,
        #     map_grid=map_grid,
        #     node=node,
        #     scene_manager=scene_manager
        # )
        self.scene_manager = scene_manager
        self.color_scheme_id = color_scheme_id
        self.drone_prim = self.create_robot_entity(prim_path)

        self.namespace = namespace  # Empty string means root namespace
        self.prim_path = prim_path  # USD prim path, e.g. "/robot1/drone" or "/drone"

        # ROS2
        ros_node: Node = None
        self.pubs: dict = field(default_factory=dict)
        self.subs: dict = field(default_factory=dict)
        self.srvs: dict = field(default_factory=dict)

        # Desired pose tracking
        des_pose: DronePose | None = None
        is_pose_dirty: bool = False

        # Optional custom per-step callable (LiDAR / perception wrapper)
        custom_step_fn: callable = None

        # Pre-computed LUT for depth→point-cloud conversion (used by perception)
        depth2pc_lut: np.ndarray | None = None

        # Locks & queues (spawn/move) — kept separate per UAV to avoid contention
        pending_spawn_requests: list = field(default_factory=list)
        spawn_lock: threading.Lock = field(default_factory=threading.Lock)
        pending_move_requests: list = field(default_factory=list)
        move_lock: threading.Lock = field(default_factory=threading.Lock)

        # sensor
        self.lidar = None

        #
        # self.node.register_feedback_publisher(
        #     robot_class=self.cfg_robot.type,
        #     robot_id=self.cfg_robot.id,
        #     qos=50
        # )
        # self.node.register_motion_publisher(
        #     robot_class=self.cfg_robot.type,
        #     robot_id=self.cfg_robot.id,
        #     qos=50
        # )
        #
        # self.custom_step_fn = None  # 用于Lidar等传感器
        # self._lidar_cfg_path = None

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
