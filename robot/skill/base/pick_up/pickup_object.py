import numpy as np
import torch

from log.log_manager import LogManager
from physics_engine.isaacsim_utils import RigidPrim
from physics_engine.pxr_utils import UsdPhysics

from gsi_msgs.gsi_msgs_helper import Parameter

logger = LogManager.get_logger(__name__)

def pickup_object_skill(**kwargs):
    robot = kwargs.get("robot")
    robot_hand_prim_path = kwargs.get("robot_hand_prim_path")
    object_prim_path = kwargs.get("object_prim_path")
    distance_threshold = kwargs.get("distance_threshold", 2.0)


    hand_prim = RigidPrim(prim_paths_expr=robot_hand_prim_path)
    object_prim = RigidPrim(prim_paths_expr=object_prim_path)

    hand_pos, _ = hand_prim.get_world_poses()
    object_pos, _ = object_prim.get_world_poses()

    distance = np.linalg.norm(hand_pos - object_pos)

    if distance <= distance_threshold:

        yield robot.form_feedback("processing", "", 40)
        # 停止物体当前的任何运动，清除惯性
        object_prim.set_linear_velocities(torch.zeros(3,  dtype=torch.float32))
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
                local_pos0=[0, 0, 1],
                local_pos1=[0, 0, 0],
                axis=[0, 0, 1],
            )
            joint_prim = robot.scene_manager.stage.GetPrimAtPath(joint_path)
        joint = UsdPhysics.Joint(joint_prim)
        joint.GetJointEnabledAttr().Set(True)  # <-- 关键的状态切换！

        yield robot.form_feedback("success", "Object picked successfully!", 100)
    else:
        yield robot.form_feedback(
            "failed",
            f"Object is too far to pick up ({distance:.2f}m > {distance_threshold}m).",
            0,
        )