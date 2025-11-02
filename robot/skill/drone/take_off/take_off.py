from robot.skill.base.navigation.navigate_to import navigate_to_skill
from log.log_manager import LogManager

logger = LogManager.get_logger(__name__)


def take_off(**kwargs):
    """无人机起飞技能 - 基于navigate_to实现"""
    robot = kwargs.get("robot")

    if robot.skill_state in [None, "INITIALIZING"]:
        _init_take_off(robot, kwargs)

    if robot.skill_state == "EXECUTING":
        return _handle_executing(robot)
    elif robot.skill_state == "COMPLETED":
        return _handle_completed(robot)
    elif robot.skill_state == "FAILED":
        return _handle_failed(robot)


def _init_take_off(robot, kwargs):
    """初始化起飞技能"""
    try:
        # 获取目标高度参数
        robot._target_altitude = kwargs.get("altitude")
        if robot._target_altitude is None:
            robot.skill_state = "FAILED"
            robot.skill_error = "Altitude parameter is required"
            return

        # 验证高度参数
        try:
            robot._target_altitude = float(robot._target_altitude)
            if robot._target_altitude <= 0:
                robot.skill_state = "FAILED"
                robot.skill_error = "Altitude must be greater than 0"
                return
        except (ValueError, TypeError):
            robot.skill_state = "FAILED"
            robot.skill_error = "Invalid altitude parameter"
            return

        # 获取当前位置
        current_pos, current_quat = robot.pos, robot.quat
        # current_pos = current_pos.cpu().numpy().tolist()
        # current_quat = current_quat.cpu().numpy().tolist()

        # 构造目标位置：保持XY，改变Z
        robot._take_off_goal = [
            current_pos[0],  # 保持当前X
            current_pos[1],  # 保持当前Y
            float(robot._target_altitude)  # 目标高度
        ]

        logger.info(f"Take off: current={current_pos}, target={robot._take_off_goal}")

        # 准备navigate_to的参数
        robot._nav_kwargs = {
            "robot": robot,
            "goal_pos": robot._take_off_goal,
            "goal_quat_wxyz": current_quat  # 保持当前朝向
        }

        robot.skill_state = "EXECUTING"

    except Exception as e:
        robot.skill_state = "FAILED"
        robot.skill_error = f"Take off initialization failed: {str(e)}"
        logger.error(f"Take off initialization error: {e}")


def _handle_executing(robot):
    """执行起飞 - 委托给navigate_to技能"""
    try:
        # 调用navigate_to技能
        robot.skill_state = "INITIALIZING"
        robot.skill_function = navigate_to_skill
        nav_result = navigate_to_skill(**robot._nav_kwargs)
        robot.skill_state = "EXECUTING"
        robot.skill_function = take_off
        # 根据navigate_to的结果更新状态
        if nav_result and nav_result.get("status") == "finished":
            robot.skill_state = "COMPLETED"
            return robot.form_feedback("processing", "Take off completed", 95)
        elif nav_result and nav_result.get("status") == "failed":
            robot.skill_state = "FAILED"
            robot.skill_error = f"Navigation failed: {nav_result.get('message', 'Unknown error')}"
            return robot.form_feedback("failed", robot.skill_error)
        else:
            # 继续执行中，传递navigate_to的进度
            progress = nav_result.get("progress", 50) if nav_result else 50
            message = nav_result.get("message", "Taking off...") if nav_result else "Taking off..."
            return robot.form_feedback("processing", f"Take off: {message}", progress)

    except Exception as e:
        robot.skill_state = "FAILED"
        robot.skill_error = f"Take off execution failed: {str(e)}"
        logger.error(f"Take off execution error: {e}")
        return robot.form_feedback("failed", robot.skill_error)


def _handle_completed(robot):
    """处理完成状态"""
    try:
        final_altitude = robot.pos[2]

        success_msg = f"Take off completed at {final_altitude:.1f}m altitude"
        logger.info(success_msg)

        _cleanup_take_off(robot)
        return robot.form_feedback("finished", success_msg, 100)
    except Exception as e:
        logger.error(f"Take off completion error: {e}")
        _cleanup_take_off(robot)
        return robot.form_feedback("finished", "Take off completed", 100)


def _handle_failed(robot):
    """处理失败状态"""
    error_msg = getattr(robot, 'skill_error', "Unknown error")
    logger.error(f"Take off failed: {error_msg}")
    _cleanup_take_off(robot)
    return robot.form_feedback("failed", error_msg)


def _cleanup_take_off(robot):
    """清理起飞技能状态"""
    attrs_to_remove = ['_target_altitude', '_take_off_goal', '_nav_kwargs']
    for attr in attrs_to_remove:
        if hasattr(robot, attr):
            delattr(robot, attr)

    robot.skill_state = None
    robot.skill_error = None
