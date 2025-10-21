def put_down_skill(**kwargs):
    robot = kwargs.get("robot")
    robot_hand_prim_path = kwargs.get("robot_hand_prim_path")
    object_prim_path = kwargs.get("object_prim_path")

    result = {"success": False, "message": "", "data": None}

    from physics_engine.isaacsim_utils import RigidPrim
    from physics_engine.pxr_utils import UsdPhysics
    from gsi_msgs.gsi_msgs_helper import Parameter
    from log.log_manager import LogManager

    logger = LogManager.get_logger(__name__)

    hand_prim = RigidPrim(prim_paths_expr=robot_hand_prim_path)
    object_prim = RigidPrim(prim_paths_expr=object_prim_path)

    # 定位并删除用于抓取的固定关节
    joint_path = f"/World/grasp_joint_{object_prim.name}"
    stage = robot.scene_manager._stage
    joint_prim = stage.GetPrimAtPath(joint_path)

    if joint_prim.IsValid():
        joint = UsdPhysics.Joint(joint_prim)
        joint.GetJointEnabledAttr().Set(False)
        # 碰撞属性
        robot.scene_manager.set_collision_enabled(
            object_prim_path, collision_enabled=True
        )
        # 模拟惯性
        robot._publish_feedback(
            params=[
                Parameter(key="status", value="normal"),
                Parameter(key="reason", value=""),
            ],
            progress=50,
        )
        hand_lin_vel = hand_prim.get_linear_velocities()
        hand_ang_vel = hand_prim.get_angular_velocities()
        object_prim.set_linear_velocities(hand_lin_vel)
        object_prim.set_angular_velocities(hand_ang_vel)

        result["success"] = True
        result["message"] = "Object dropped successfully"
    else:
        robot._publish_feedback(
            params=[
                Parameter(key="status", value="failed"),
                Parameter(key="reason", value="Joint not found"),
            ],
            progress=0,
        )
        logger.info(f"WARNING: Joint '{joint_path}' not found, cannot disable.")
        result["message"] = "Joint not found, cannot drop object"

    logger.info(f"INFO: Object '{object_prim.name}' dropped and physics restored.")
    robot._publish_feedback(
        params=[
            Parameter(key="status", value="normal"),
            Parameter(key="reason", value=""),
        ],
        progress=100,
    )

    result["data"] = {"status": "success"}
    return result
