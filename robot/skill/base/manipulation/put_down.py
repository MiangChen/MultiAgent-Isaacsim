from log.log_manager import LogManager
from physics_engine.isaacsim_utils import RigidPrim
from physics_engine.pxr_utils import UsdPhysics

from gsi_msgs.gsi_msgs_helper import Parameter

logger = LogManager.get_logger(__name__)

def put_down_skill(**kwargs):
    robot = kwargs.get("robot")
    robot_hand_prim_path = kwargs.get("robot_hand_prim_path")
    object_prim_path = kwargs.get("object_prim_path")

    hand_prim = RigidPrim(prim_paths_expr=robot_hand_prim_path)
    object_prim = RigidPrim(prim_paths_expr=object_prim_path)

    # 定位并删除用于抓取的固定关节
    joint_path = f"/World/grasp_joint_{object_prim.name}"
    stage = robot.scene_manager.stage
    joint_prim = stage.GetPrimAtPath(joint_path)

    if joint_prim.IsValid():
        joint = UsdPhysics.Joint(joint_prim)
        joint.GetJointEnabledAttr().Set(False)  # <-- 关键的状态切换！
        robot.scene_manager.set_collision_enabled(
            object_prim_path, collision_enabled=True
        )
        yield robot.form_feedback(progress=50)

        # 模拟惯性
        hand_lin_vel = hand_prim.get_linear_velocities()
        hand_ang_vel = hand_prim.get_angular_velocities()
        object_prim.set_linear_velocities(hand_lin_vel)
        object_prim.set_angular_velocities(hand_ang_vel)
    else:
        logger.info(f"WARNING: Joint '{joint_path}' not found, cannot disable.")
        yield robot.form_feedback("failed", "Joint not found", 0)

    logger.info(f"INFO: Object '{object_prim.name}' dropped and physics restored.")
    yield robot.form_feedback("success", "Object picked successfully!", 100)
