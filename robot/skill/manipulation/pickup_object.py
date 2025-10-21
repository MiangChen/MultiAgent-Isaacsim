def pickup_object_skill(**kwargs):
    robot = kwargs.get("robot")
    robot_hand_prim_path = kwargs.get("robot_hand_prim_path")
    object_prim_path = kwargs.get("object_prim_path")
    distance_threshold = kwargs.get("distance_threshold", 2.0)

    result = {"success": False, "message": "", "data": None}

    from physics_engine.isaacsim_utils import RigidPrim
    from physics_engine.pxr_utils import UsdPhysics
    from gsi_msgs.gsi_msgs_helper import Parameter
    import numpy as np

    hand_prim = RigidPrim(prim_paths_expr=robot_hand_prim_path)
    object_prim = RigidPrim(prim_paths_expr=object_prim_path)

    hand_pos, _ = hand_prim.get_world_poses()
    object_pos, _ = object_prim.get_world_poses()

    distance = np.linalg.norm(hand_pos - object_pos)

    if distance <= distance_threshold:

        robot._publish_feedback(
            params=[
                Parameter(key="status", value="normal"),
                Parameter(key="reason", value=""),
            ],
            progress=30,
        )
        # 停止物体当前的任何运动，清除惯性
        object_prim.set_linear_velocities(np.zeros(3))
        object_prim.set_angular_velocities(np.zeros(3))

        # 将物体传送到精确的抓取位置
        object_prim.set_world_poses(positions=hand_pos)

        # 禁用碰撞属性
        robot.scene_manager.set_collision_enabled(
            prim_path=object_prim_path, collision_enabled=False
        )

        # 创建物理连接 (关节)
        joint_path = f"/World/grasp_joint_{object_prim.name}"
        stage = robot.scene_manager._stage
        joint_prim = stage.GetPrimAtPath(joint_path)

        robot._publish_feedback(
            params=[
                Parameter(key="status", value="normal"),
                Parameter(key="reason", value=""),
            ],
            progress=60,
        )

        # 如果关节不存在，就创建一个。这通常只在第一次抓取时发生。
        if not joint_prim.IsValid():
            from log.log_manager import LogManager

            logger = LogManager.get_logger(__name__)
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
            joint_prim = stage.GetPrimAtPath(joint_path)
        joint = UsdPhysics.Joint(joint_prim)
        joint.GetJointEnabledAttr().Set(True)

        robot._publish_feedback(
            params=[
                Parameter(key="status", value="normal"),
                Parameter(key="reason", value=""),
            ],
            progress=100,
        )

        result["success"] = True
        result["message"] = "Object picked up successfully"
        result["data"] = {"status": "success"}
    else:
        robot._publish_feedback(
            params=[
                Parameter(key="status", value="failed"),
                Parameter(key="reason", value="Object is too far to pick up"),
            ],
            progress=0,
        )
        result["message"] = (
            f"Object is too far to pick up ({distance:.2f}m > {distance_threshold}m)"
        )
        result["data"] = {"status": "skipped"}

    return result
