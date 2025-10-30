def take_photo_skill(**kwargs):
    robot = kwargs.get("robot")
    
    # 初始化状态机（只在第一次调用时执
    if not hasattr(robot, 'skill_state'):
        _init_take_photo(robot, kwargs)
        
    # 状态机：每个physics step执行一次
    if robot.skill_state == "INITIALIZING":
        return _handle_initializing(robot)
    elif robot.skill_state == "EXECUTING":
        return _handle_executing(robot)
    elif robot.skill_state == "COMPLETED":
        return _handle_completed(robot)
    elif robot.skill_state == "FAILED":
        return _handle_failed(robot)


def _init_take_photo(robot, kwargs):
    """初始化拍照技能"""
    robot._photo_file_path = kwargs.get("file_path")
    robot.skill_state = "EXECUTING"

def _handle_initializing(robot):
    # 检查相机是否可用
    if robot.camera is None:
        robot.skill_state = "FAILED"
        robot.skill_error = "Camera is not available."
        return robot.form_feedback("failed", robot.skill_error)
    return robot.form_feedback("processing", "Camera is ready", 20)

def _handle_executing(robot):
    """执行拍照"""
    try:
        # 获取RGB图像
        rgb = robot.camera.get_rgb()
        
        if rgb is None:
            robot.skill_state = "FAILED"
            robot.skill_error = "Failed to capture image"
            return robot.form_feedback("failed", robot.skill_error)
        
        # 保存图像到文件（如果提供了文件路径）
        if robot._photo_file_path is not None:
            robot.camera.save_rgb_to_file(rgb_tensor_gpu=rgb, file_path=robot._photo_file_path)
        
        # 存储结果
        robot._photo_result = rgb
        robot.skill_state = "COMPLETED"
        
        return robot.form_feedback("processing", "Taking photo...", 90)
        
    except Exception as e:
        robot.skill_state = "FAILED"
        robot.skill_error = f"Photo capture failed: {str(e)}"
        return robot.form_feedback("failed", f"Photo capture failed: {str(e)}")


def _handle_completed(robot):
    """处理完成状态"""
    _cleanup_take_photo(robot)
    return robot.form_feedback("finished", "Photo taken successfully", 100)


def _handle_failed(robot):
    """处理失败状态"""
    error_msg = getattr(robot, 'skill_error', "Unknown error")
    _cleanup_take_photo(robot)
    return robot.form_feedback("failed", error_msg)


def _cleanup_take_photo(robot):
    """清理拍照技能的状态"""
    attrs_to_remove = ['_photo_file_path', '_photo_result']
    for attr in attrs_to_remove:
        if hasattr(robot, attr):
            delattr(robot, attr)
    
    robot.skill_state = None
    robot.skill_error = None