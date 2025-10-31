from log.log_manager import LogManager

logger = LogManager.get_logger(__name__)


def detect_skill(**kwargs):
    robot = kwargs.get("robot")
    
    # 初始化状态机（只在第一次调用时执行）
    if not hasattr(robot, 'skill_state'):
        _init_detect(robot, kwargs)
    
    # 状态机：每个physics step执行一次
    if robot.skill_state == "INITIALIZING":
        return _handle_initializing(robot)
    elif robot.skill_state == "EXECUTING":
        return _handle_executing(robot)
    elif robot.skill_state == "COMPLETED":
        return _handle_completed(robot)
    elif robot.skill_state == "FAILED":
        return _handle_failed(robot)


def _init_detect(robot, kwargs):
    """初始化检测技能"""
    robot._target_prim = kwargs.get("target_prim")
    robot.skill_state = "INITIALIZING"


def _handle_initializing(robot):
    """初始化阶段：检查目标对象和必要组件"""
    try:
        # 检查目标对象
        if not robot._target_prim:
            robot.skill_state = "FAILED"
            robot.skill_error = "Target prim is not provided."
            return robot.form_feedback("failed", robot.skill_error)
        
        # 检查机器人身体组件
        if not hasattr(robot, 'body') or robot.body is None:
            robot.skill_state = "FAILED"
            robot.skill_error = "Robot body is not available."
            return robot.form_feedback("failed", robot.skill_error)
        
        # 检查场景管理器
        if not hasattr(robot, 'scene_manager') or robot.scene_manager is None:
            robot.skill_state = "FAILED"
            robot.skill_error = "Scene manager is not available."
            return robot.form_feedback("failed", robot.skill_error)
        
        # 初始化完成，进入执行阶段
        robot.skill_state = "EXECUTING"
        return robot.form_feedback("processing", "Initializing detection...", 20)
        
    except Exception as e:
        robot.skill_state = "FAILED"
        robot.skill_error = f"Detection initialization failed: {str(e)}"
        return robot.form_feedback("failed", robot.skill_error)


def _handle_executing(robot):
    """执行检测操作"""
    try:
        # 获取机器人当前位置
        pos, quat = robot.body.get_world_pose()
        
        # 执行重叠检测
        detection_result = robot.scene_manager.overlap_hits_target_ancestor(robot._target_prim)
        
        # 记录检测结果
        robot_name = robot.body.cfg_robot.name
        logger.info(
            f"[Skill] {robot_name} is detecting for {robot._target_prim}. The result is {detection_result}"
        )
        
        # 存储检测结果
        robot._detection_result = {
            "success": True,
            "message": f"Detection completed for {robot._target_prim}",
            "data": detection_result
        }
        
        robot.skill_state = "COMPLETED"
        return robot.form_feedback("processing", f"Detecting {robot._target_prim}...", 90)
        
    except Exception as e:
        robot.skill_state = "FAILED"
        robot.skill_error = f"Detection execution failed: {str(e)}"
        return robot.form_feedback("failed", robot.skill_error)


def _handle_completed(robot):
    """处理完成状态"""
    result = getattr(robot, '_detection_result', {"success": False, "message": "Unknown result", "data": None})
    _cleanup_detect(robot)
    return robot.form_feedback("finished", result["message"], 100)


def _handle_failed(robot):
    """处理失败状态"""
    error_msg = getattr(robot, 'skill_error', "Unknown error")
    _cleanup_detect(robot)
    return robot.form_feedback("failed", error_msg)


def _cleanup_detect(robot):
    """清理检测技能的状态"""
    attrs_to_remove = ['_target_prim', '_detection_result']
    for attr in attrs_to_remove:
        if hasattr(robot, attr):
            delattr(robot, attr)
    
    robot.skill_state = None
    robot.skill_error = None
