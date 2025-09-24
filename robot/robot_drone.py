from dataclasses import dataclass, field

import numpy as np

import threading
from pxr import Gf, UsdGeom

from rclpy.node import Node


class DronePose:
    def __init__(self, pos=Gf.Vec3d(0, 0, 0), quat=Gf.Quatf.GetIdentity()):
        self.pos = pos
        self.quat = quat


@dataclass
class DroneSimCtx:
    """Container holding all state associated with a single UAV instance."""

    namespace: str  # Empty string means root namespace
    prim_path: str  # USD prim path, e.g. "/robot1/drone" or "/drone"

    # ROS2
    ros_node: Node
    pubs: dict = field(default_factory=dict)
    subs: dict = field(default_factory=dict)
    srvs: dict = field(default_factory=dict)

    # Simulation prim handle for this drone (returned by add_drone_body)
    drone_prim: object | None = None

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
