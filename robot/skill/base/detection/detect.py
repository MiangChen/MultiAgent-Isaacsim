def detect_skill(**kwargs):
    robot = kwargs.get("robot")
    target_prim = kwargs.get("target_prim")

    result = {"success": False, "message": "", "data": None}

    from log.log_manager import LogManager

    logger = LogManager.get_logger(__name__)

    pos, quat = robot.body.get_world_pose()
    detection_result = robot.scene_manager.overlap_hits_target_ancestor(target_prim)
    logger.info(
        f"[Skill] {robot.body.cfg_robot.name} is detecting for {target_prim}. The result is {detection_result}"
    )

    result["success"] = True
    result["message"] = f"Detection completed for {target_prim}"
    result["data"] = detection_result

    return result
