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

    if not hasattr(robot, 'camera_dict') or robot._camera_name not in robot.camera_dict:
        robot.skill_state = "FAILED"
        robot.skill_error = "Semantic camera is not available."
        return robot.form_feedback("failed", robot.skill_error)

    # 初始化完成，进入执行阶段
    robot.skill_state = "EXECUTING"
    return robot.form_feedback("processing", "Initializing object detection...", 20)


def _handle_executing(robot):
    """执行物体检测"""
    try:
        camera = robot.camera_dict[robot._camera_name]
        result = camera.get_semantic_detection()
        if result is None:
            robot.skill_state = "EXECUTING"
            robot.skill_error = "Failed to get current frame from semantic camera"
            return robot.form_feedback("processing", robot.skill_error)
        elif type(result) is str:
            robot.skill_state = "EXECUTING"
            return robot.form_feedback("processing", result)
        else:
            robot.skill_state = "COMPLETED"
            robot._detection_result = {"success": True, "message": None, "data": result}
            return robot.form_feedback("processing", f"Detecting {robot._target_class}...", 90)

    except Exception as e:
        robot.skill_state = "FAILED"
        robot.skill_error = f"Object detection failed: {str(e)}"
        return robot.form_feedback("failed", robot.skill_error)


def _handle_completed(robot):
    """处理完成状态"""
    result = getattr(robot, '_detection_result', {"success": False, "message": "no _detection_result", "data": None})
    _cleanup_object_detection(robot)

    return robot.form_feedback("finished", result, 100)


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
