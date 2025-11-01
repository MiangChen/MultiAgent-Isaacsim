def take_photo(**kwargs):
    robot = kwargs.get("robot")
    
    if robot.skill_state in [None, "INITIALIZING"]:
        _init_take_photo(robot, kwargs)
    
    if robot.skill_state == "EXECUTING":
        return _handle_executing(robot)
    elif robot.skill_state == "COMPLETED":
        return _handle_completed(robot)
    elif robot.skill_state == "FAILED":
        return _handle_failed(robot)


def _init_take_photo(robot, kwargs):
    robot._camera_name = kwargs.get("camera_name", "default")
    robot._save_to_file = kwargs.get("save_to_file", "")
    
    if not hasattr(robot, 'camera_dict') or robot._camera_name not in robot.camera_dict:
        robot.skill_state = "FAILED"
        robot.skill_error = f"Camera '{robot._camera_name}' is not available"
    else:
        robot.skill_state = "EXECUTING"

def _handle_executing(robot):
    camera = robot.camera_dict[robot._camera_name]
    
    rgb = camera.get_rgb()
    
    if rgb is None:
        robot.skill_state = "FAILED"
        robot.skill_error = "Failed to capture image"
        return robot.form_feedback("failed", robot.skill_error)
    
    photo_data = {
        'rgb_image': rgb,
        'camera_name': robot._camera_name,
        'camera_info': camera.get_camera_info(),
        'robot_pose': robot.body.get_world_pose(),
    }
    
    robot.data_io.output("photo", photo_data)
    
    if robot._save_to_file:
        camera.save_rgb_to_file(rgb_tensor_gpu=rgb, file_path=robot._save_to_file)
    
    robot.skill_state = "COMPLETED"
    return robot.form_feedback("processing", "Taking photo...", 90)


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
    attrs_to_remove = ['_camera_name', '_save_to_file']
    for attr in attrs_to_remove:
        if hasattr(robot, attr):
            delattr(robot, attr)
    
    robot.skill_state = None
    robot.skill_error = None