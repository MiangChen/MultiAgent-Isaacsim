def object_detection_skill(**kwargs):
    robot = kwargs.get("robot")
    
    # 初始化状态机（只在第一次调用时执行）
    if not hasattr(robot, 'skill_state'):
        _init_object_detection(robot, kwargs)
    
    # 状态机：每个physics step执行一次
    if robot.skill_state == "INITIALIZING":
        return _handle_initializing(robot)
    elif robot.skill_state == "EXECUTING":
        return _handle_executing(robot)
    elif robot.skill_state == "COMPLETED":
        return _handle_completed(robot)
    elif robot.skill_state == "FAILED":
        return _handle_failed(robot)


def _init_object_detection(robot, kwargs):
    """初始化物体检测技能"""
    robot._semantic_camera = kwargs.get("semantic_camera")
    robot._map_semantic = kwargs.get("map_semantic")
    robot._target_class = kwargs.get("target_class", "car")
    robot.skill_state = "INITIALIZING"


def _handle_initializing(robot):
    """初始化阶段：检查必要的组件是否可用"""
    # 检查语义相机是否可用
    if robot._semantic_camera is None:
        robot.skill_state = "FAILED"
        robot.skill_error = "Semantic camera is not available."
        return robot.form_feedback("failed", robot.skill_error)
    
    # 检查语义地图是否可用
    if robot._map_semantic is None:
        robot.skill_state = "FAILED"
        robot.skill_error = "Map semantic is not available."
        return robot.form_feedback("failed", robot.skill_error)
    
    # 初始化完成，进入执行阶段
    robot.skill_state = "EXECUTING"
    return robot.form_feedback("processing", "Initializing object detection...", 20)


def _handle_executing(robot):
    """执行物体检测"""
    try:
        # 获取当前帧
        current_frame = robot._semantic_camera.get_current_frame()
        
        if current_frame is None:
            robot.skill_state = "FAILED"
            robot.skill_error = "Failed to get current frame from semantic camera"
            return robot.form_feedback("failed", robot.skill_error)
        
        # 获取边界框结果
        bounding_box_result = current_frame.get("bounding_box_2d_loose")
        
        if bounding_box_result is None:
            robot.skill_state = "FAILED"
            robot.skill_error = "No bounding box data in current frame"
            return robot.form_feedback("failed", robot.skill_error)
        
        # 执行语义检测
        prim_target, target_pose = robot._map_semantic.get_prim_and_pose_by_semantic(
            bounding_box_result, target_semantic_class=robot._target_class
        )
        
        # 存储检测结果
        if prim_target and target_pose:
            robot._detection_result = {
                "success": True,
                "message": f"Detected {robot._target_class}",
                "data": {"prim_path": prim_target, "pose": target_pose}
            }
        else:
            robot._detection_result = {
                "success": False,
                "message": f"No {robot._target_class} detected",
                "data": None
            }
        
        robot.skill_state = "COMPLETED"
        return robot.form_feedback("processing", f"Detecting {robot._target_class}...", 90)
        
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
