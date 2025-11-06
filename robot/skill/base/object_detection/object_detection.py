from robot.skill.skill_registry import SkillRegistry


@SkillRegistry.register(["jetbot", "g1", "h1", "cf2x"])
def object_detection_skill(**kwargs):
    """物体检测技能 - 使用字典化状态管理"""
    robot = kwargs.get("robot")
    skill_name = "object_detection_skill"

    current_state = robot.skill_states.get(skill_name)

    if current_state in [None, "INITIALIZING"]:
        _init_object_detection(robot, skill_name, kwargs)

    current_state = robot.skill_states.get(skill_name)
    if current_state == "EXECUTING":
        return _handle_executing(robot, skill_name)
    elif current_state == "COMPLETED":
        return _handle_completed(robot, skill_name)
    elif current_state == "FAILED":
        return _handle_failed(robot, skill_name)


def _init_object_detection(robot, skill_name, kwargs):
    """初始化物体检测技能"""
    try:
        # 获取参数
        camera_name = kwargs.get("camera_name", "default")
        target_class = kwargs.get("target_class", "car")

        # 存储到技能私有数据
        robot.set_skill_data(skill_name, "camera_name", camera_name)
        robot.set_skill_data(skill_name, "target_class", target_class)

        robot.skill_states[skill_name] = "INITIALIZING"

        # 检查相机是否可用
        if not hasattr(robot, "camera_dict") or camera_name not in robot.camera_dict:
            robot.skill_states[skill_name] = "FAILED"
            robot.skill_errors[skill_name] = "Semantic camera is not available."
            return

        # 初始化完成，进入执行阶段
        robot.skill_states[skill_name] = "EXECUTING"

    except Exception as e:
        robot.skill_states[skill_name] = "FAILED"
        robot.skill_errors[skill_name] = (
            f"Object detection initialization failed: {str(e)}"
        )


def _handle_executing(robot, skill_name):
    """执行物体检测"""
    try:
        camera_name = robot.get_skill_data(skill_name, "camera_name")
        target_class = robot.get_skill_data(skill_name, "target_class")

        camera = robot.camera_dict[camera_name]
        result = camera.get_semantic_detection()

        if result is None:
            robot.skill_states[skill_name] = "EXECUTING"
            error_msg = "Failed to get current frame from semantic camera"
            robot.set_skill_data(skill_name, "last_error", error_msg)
            return robot.form_feedback("processing", error_msg)
        elif type(result) is str:
            robot.skill_states[skill_name] = "EXECUTING"
            return robot.form_feedback("processing", result)
        else:
            robot.skill_states[skill_name] = "COMPLETED"
            detection_result = {"success": True, "message": None, "data": result}
            robot.set_skill_data(skill_name, "detection_result", detection_result)
            return robot.form_feedback("processing", f"Detecting {target_class}...", 90)

    except Exception as e:
        robot.skill_states[skill_name] = "FAILED"
        robot.skill_errors[skill_name] = f"Object detection failed: {str(e)}"
        return robot.form_feedback("failed", robot.skill_errors[skill_name])


def _handle_completed(robot, skill_name):
    """处理完成状态"""
    detection_result = robot.get_skill_data(
        skill_name,
        "detection_result",
        {"success": False, "message": "no detection result", "data": None},
    )
    return robot.form_feedback("completed", detection_result, 100)


def _handle_failed(robot, skill_name):
    """处理失败状态"""
    error_msg = robot.skill_errors.get(skill_name, "Unknown error")
    return robot.form_feedback("failed", error_msg)
