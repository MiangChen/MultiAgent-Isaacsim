def object_detection_skill(**kwargs):
    robot = kwargs.get("robot")

    if robot.skill_state in [None, "INITIALIZING"]:
        _init_object_detection(robot, kwargs)

    if robot.skill_state == "EXECUTING":
        return _handle_executing(robot)
    elif robot.skill_state == "COMPLETED":
        return _handle_completed(robot)
    elif robot.skill_state == "FAILED":
        return _handle_failed(robot)


def _init_object_detection(robot, kwargs):
    """初始化物体检测技能"""
    robot._camera_name = kwargs.get("camera_name", "default")
    robot._target_class = kwargs.get("target_class", "car")
    robot.skill_state = "INITIALIZING"

    if robot.camera_name is None:
        robot.skill_state = "FAILED"
        robot.skill_error = "Semantic camera is not available."
        return robot.form_feedback("failed", robot.skill_error)

    # 检查语义地图是否可用
    # if robot._map_semantic is None:
    #     robot.skill_state = "FAILED"
    #     robot.skill_error = "Map semantic is not available."
    #     return robot.form_feedback("failed", robot.skill_error)

    # 初始化完成，进入执行阶段
    robot.skill_state = "EXECUTING"
    return robot.form_feedback("processing", "Initializing object detection...", 20)


def _handle_executing(robot):
    """执行物体检测"""
    try:
        # 获取当前帧
        camera = robot.camera_dict[robot.camera_name]
        result = camera.get_semantic_detection()
        if result is None:
            robot.skill_state = "FAILED"
            robot.skill_error = "Failed to get current frame from semantic camera"
            return robot.form_feedback("failed", robot.skill_error)

        # robot.skill_feedback =  result
        # # 执行语义检测
        # prim_target, target_pose = robot._map_semantic.get_prim_and_pose_by_semantic(
        #     bounding_box_result, target_semantic_class=robot._target_class
        # )

        # 存储检测结果
        # if prim_target and target_pose:
        #     robot._detection_result = {
        #         "success": True,
        #         "message": f"Detected {robot._target_class}",
        #         "data": {"prim_path": prim_target, "pose": target_pose}
        #     }
        # else:
        #     robot._detection_result = {
        #         "success": False,
        #         "message": f"No {robot._target_class} detected",
        #         "data": None
        #     }

        robot.skill_state = "COMPLETED"
        return robot.form_feedback("processing", f"Detecting {robot._target_class}...{result}", 90)

    except Exception as e:
        robot.skill_state = "FAILED"
        robot.skill_error = f"Object detection failed: {str(e)}"
        return robot.form_feedback("failed", robot.skill_error)


def _handle_completed(robot):
    """处理完成状态"""
    result = getattr(robot, '_detection_result', {"success": False, "message": "Unknown result", "data": None})
    _cleanup_object_detection(robot)

    if result["success"]:
        return robot.form_feedback("finished", result["message"], 100)
    else:
        return robot.form_feedback("finished", result["message"], 100)


def _handle_failed(robot):
    """处理失败状态"""
    error_msg = getattr(robot, 'skill_error', "Unknown error")
    _cleanup_object_detection(robot)
    return robot.form_feedback("failed", error_msg)


def _cleanup_object_detection(robot):
    """清理物体检测技能的状态"""
    attrs_to_remove = ['_semantic_camera', '_map_semantic', '_target_class', '_detection_result']
    for attr in attrs_to_remove:
        if hasattr(robot, attr):
            delattr(robot, attr)

    robot.skill_state = None
    robot.skill_error = None
