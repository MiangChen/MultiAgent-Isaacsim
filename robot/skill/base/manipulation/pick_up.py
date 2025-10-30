import json

import numpy as np
import torch

from log.log_manager import LogManager
from physics_engine.isaacsim_utils import RigidPrim
from physics_engine.pxr_utils import UsdPhysics, Gf

from gsi_msgs.gsi_msgs_helper import Parameter

logger = LogManager.get_logger(__name__)


def pick_up_skill(**kwargs):
    robot = kwargs.get("robot")
    robot_hand_prim_path = kwargs.get("robot_hand_prim_path")
    object_prim_path = kwargs.get("object_prim_path")
    distance_threshold = kwargs.get("distance_threshold", 2.0)
    axis = kwargs.get("axis", [0, 0, 1])
    local_pos_hand = kwargs.get("local_pos_hand", [0, 0, 1])
    local_pos_object = kwargs.get("local_pos_object", [0, 0, 0])

    if type(axis) is str:
        axis = json.loads(axis)
    if type(local_pos_hand) is str:
        local_pos_hand = json.loads(local_pos_hand)
    if type(local_pos_object) is str:
        local_pos_object = json.loads(local_pos_object)
    print(type(axis), axis)
    print(type(local_pos_hand), local_pos_hand)
    print(type(local_pos_object), local_pos_object)
    hand_prim = RigidPrim(prim_paths_expr=robot_hand_prim_path)
    object_prim = RigidPrim(prim_paths_expr=object_prim_path)

    hand_pos, _ = hand_prim.get_world_poses()
    object_pos, _ = object_prim.get_world_poses()

    distance = np.linalg.norm(hand_pos - object_pos)

    if distance <= distance_threshold:

        yield robot.form_feedback("processing", "", 40)
        # 停止物体当前的任何运动，清除惯性
        object_prim.set_linear_velocities(torch.zeros(3, dtype=torch.float32))
        object_prim.set_angular_velocities(torch.zeros(3, dtype=torch.float32))

        # 将物体传送到精确的抓取位置
        object_prim.set_world_poses(positions=hand_pos)

        # 禁用碰撞属性
        robot.scene_manager.set_collision_enabled(
            prim_path=object_prim_path, collision_enabled=False
        )

        # 创建物理连接 (关节)
        joint_path = f"/World/grasp_joint_{object_prim.name}"
        joint_prim = robot.scene_manager.stage.GetPrimAtPath(joint_path)

        yield robot.form_feedback("processing", "", 60)

        # 如果关节不存在，就创建一个。这通常只在第一次抓取时发生。
        if not joint_prim.IsValid():
            logger.info(
                f"INFO: Joint '{joint_path}' not found, creating it for the first time."
            )
            robot.scene_manager.create_joint(
                joint_path=joint_path,
                joint_type="fixed",
                body0=robot_hand_prim_path,
                body1=object_prim_path,
                local_pos0=local_pos_hand,
                local_pos1=local_pos_object,
                axis=axis,
            )
            joint_prim = robot.scene_manager.stage.GetPrimAtPath(joint_path)
        joint = UsdPhysics.Joint(joint_prim)
        joint.GetLocalPos0Attr().Set(Gf.Vec3f(local_pos_hand))
        joint.GetLocalPos1Attr().Set(Gf.Vec3f(local_pos_object))
        if axis == [1, 0, 0]:
            axis_str = "X"
        elif axis == [0, 1, 0]:
            axis_str = "Y"
        else:
            axis_str = "Z"
        # joint.GetAxisAttr().Set(axis_str)
        joint.GetJointEnabledAttr().Set(True)  # <-- 关键的状态切换！

        yield robot.form_feedback("success", "Object picked successfully!", 100)
    else:
        yield robot.form_feedback(
            "failed",
            f"Object is too far to pick up ({distance:.2f}m > {distance_threshold}m).",
            0,
        )
