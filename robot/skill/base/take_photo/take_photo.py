from robot.skill.skill_registry import SkillRegistry


@SkillRegistry.register(["jetbot", "g1", "h1", "cf2x"])
def take_photo(**kwargs):
    """拍照技能 - 使用字典化状态管理"""
    robot = kwargs.get("robot")
    skill_name = "take_photo"

    current_state = robot.skill_states.get(skill_name)

    if current_state in [None, "INITIALIZING"]:
        _init_take_photo(robot, skill_name, kwargs)

    current_state = robot.skill_states.get(skill_name)
    if current_state == "EXECUTING":
        return _handle_executing(robot, skill_name)
    elif current_state == "COMPLETED":
        return _handle_completed(robot, skill_name)
    elif current_state == "FAILED":
        return _handle_failed(robot, skill_name)


def _init_take_photo(robot, skill_name, kwargs):
    """初始化拍照技能"""
    try:
        # 获取参数
        camera_name = kwargs.get("camera_name", "default")
        save_to_file = kwargs.get("save_to_file", "")

        # 存储到技能私有数据
        robot.set_skill_data(skill_name, "camera_name", camera_name)
        robot.set_skill_data(skill_name, "save_to_file", save_to_file)

        # 检查相机是否可用
        if not hasattr(robot, "camera_dict") or camera_name not in robot.camera_dict:
            robot.skill_states[skill_name] = "FAILED"
            robot.skill_errors[skill_name] = f"Camera '{camera_name}' is not available"
        else:
            robot.skill_states[skill_name] = "EXECUTING"

    except Exception as e:
        robot.skill_states[skill_name] = "FAILED"
        robot.skill_errors[skill_name] = f"Take photo initialization failed: {str(e)}"


def _handle_executing(robot, skill_name):
    """执行拍照"""
    try:
        camera_name = robot.get_skill_data(skill_name, "camera_name")
        save_to_file = robot.get_skill_data(skill_name, "save_to_file")

        camera = robot.camera_dict[camera_name]
        rgb = camera.get_rgb()

        if rgb is None:
            robot.skill_states[skill_name] = "FAILED"
            robot.skill_errors[skill_name] = "Failed to capture image"
            return robot.form_feedback("failed", robot.skill_errors[skill_name])

        # 存储照片数据
        photo_data = {
            "rgb_image": rgb,
            "camera_name": camera_name,
        }
        robot.set_skill_data(skill_name, "photo_data", photo_data)

        # robot.data_io.output("photo", photo_data)

        feedback_msg = "Taking photo..."
        if save_to_file:
            save_result = camera.save_rgb_to_file(rgb=rgb, file_path=save_to_file)
            if save_result:
                feedback_msg += f" Saved to {save_to_file}"

        robot.skill_states[skill_name] = "COMPLETED"
        return robot.form_feedback("processing", feedback_msg, 90)

    except Exception as e:
        robot.skill_states[skill_name] = "FAILED"
        robot.skill_errors[skill_name] = f"Take photo execution failed: {str(e)}"
        return robot.form_feedback("failed", robot.skill_errors[skill_name])


def _handle_completed(robot, skill_name):
    """处理完成状态"""
    return robot.form_feedback("completed", "Photo taken successfully", 100)


def _handle_failed(robot, skill_name):
    """处理失败状态"""
    error_msg = robot.skill_errors.get(skill_name, "Unknown error")
    return robot.form_feedback("failed", error_msg)
