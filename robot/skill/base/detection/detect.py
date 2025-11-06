from log.log_manager import LogManager

logger = LogManager.get_logger(__name__)


def detect_skill(**kwargs):
    robot = kwargs.get("robot")
    skill_name = "detect_skill"

    current_state = robot.skill_states.get(skill_name)

    # 初始化状态机（只在第一次调用时执行）
    if current_state in [None, "INITIALIZING"]:
        _init_detect(robot, skill_name, kwargs)

    current_state = robot.skill_states.get(skill_name)
    if current_state == "INITIALIZING":
        return _handle_initializing(robot, skill_name)
    elif current_state == "EXECUTING":
        return _handle_executing(robot, skill_name)
    elif current_state == "COMPLETED":
        return _handle_completed(robot, skill_name)
    elif current_state == "FAILED":
        return _handle_failed(robot, skill_name)


def _init_detect(robot, skill_name, kwargs):
    """初始化检测技能"""
    target_prim = kwargs.get("target_prim")

    # 存储参数到技能私有数据
    robot.set_skill_data(skill_name, "target_prim", target_prim)

    robot.skill_states[skill_name] = "INITIALIZING"


def _handle_initializing(robot, skill_name):
    """初始化阶段：检查必要的组件和参数"""
    try:
        # 获取参数
        target_prim = robot.get_skill_data(skill_name, "target_prim")

        # 检查目标对象
        if not target_prim:
            robot.skill_states[skill_name] = "FAILED"
            robot.skill_errors[skill_name] = "Target prim is not provided."
            return robot.form_feedback("failed", robot.skill_errors[skill_name])

        # 检查机器人身体组件
        if not hasattr(robot, "body") or robot.body is None:
            robot.skill_states[skill_name] = "FAILED"
            robot.skill_errors[skill_name] = "Robot body is not available."
            return robot.form_feedback("failed", robot.skill_errors[skill_name])

        # 检查场景管理器
        if not hasattr(robot, "scene_manager") or robot.scene_manager is None:
            robot.skill_states[skill_name] = "FAILED"
            robot.skill_errors[skill_name] = "Scene manager is not available."
            return robot.form_feedback("failed", robot.skill_errors[skill_name])

        # 初始化完成，进入执行阶段
        robot.skill_states[skill_name] = "EXECUTING"
        return robot.form_feedback("processing", "Initializing detection...", 20)

    except Exception as e:
        robot.skill_states[skill_name] = "FAILED"
        robot.skill_errors[skill_name] = f"Detection initialization failed: {str(e)}"
        return robot.form_feedback("failed", robot.skill_errors[skill_name])


def _handle_executing(robot, skill_name):
    """执行检测操作"""
    try:
        # 获取存储的数据
        target_prim = robot.get_skill_data(skill_name, "target_prim")

        # 获取机器人当前位置
        pos, quat = robot.body.get_world_pose()

        # 执行重叠检测
        detection_result = robot.scene_manager.overlap_hits_target_ancestor(target_prim)

        # 记录检测结果
        robot_name = robot.body.cfg_robot.name
        logger.info(
            f"[Skill] {robot_name} is detecting for {target_prim}. The result is {detection_result}"
        )

        # 存储检测结果
        detection_result_data = {
            "success": True,
            "message": f"Detection completed for {target_prim}",
            "data": detection_result,
        }
        robot.set_skill_data(skill_name, "detection_result", detection_result_data)

        robot.skill_states[skill_name] = "COMPLETED"
        return robot.form_feedback("processing", f"Detecting {target_prim}...", 90)

    except Exception as e:
        robot.skill_states[skill_name] = "FAILED"
        robot.skill_errors[skill_name] = f"Detection execution failed: {str(e)}"
        return robot.form_feedback("failed", robot.skill_errors[skill_name])


def _handle_completed(robot, skill_name):
    """处理完成状态"""
    result = robot.get_skill_data(
        skill_name,
        "detection_result",
        {"success": False, "message": "Unknown result", "data": None},
    )
    return robot.form_feedback("completed", result["message"], 100)


def _handle_failed(robot, skill_name):
    """处理失败状态"""
    error_msg = robot.skill_errors.get(skill_name, "Unknown error")
    return robot.form_feedback("failed", error_msg)
