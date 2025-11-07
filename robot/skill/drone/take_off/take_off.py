import json
from robot.skill.base.navigation.navigate_to import navigate_to
from robot.skill.skill_registry import SkillRegistry
from log.log_manager import LogManager

logger = LogManager.get_logger(__name__)


@SkillRegistry.register()
def take_off(**kwargs):
    """无人机起飞技能 - 使用字典化状态管理"""
    robot = kwargs.get("robot")
    skill_name = "take_off"  # 直接使用函数名

    current_state = robot.skill_states.get(skill_name)

    if current_state in [None, "INITIALIZING"]:
        _init_take_off(robot, skill_name, kwargs)

    current_state = robot.skill_states.get(skill_name)

    if current_state == "EXECUTING_1":
        return _handle_executing_1(robot, skill_name)
    if current_state == "EXECUTING":
        return _handle_executing(robot, skill_name)
    elif current_state == "COMPLETED":
        return _handle_completed(robot, skill_name)
    elif current_state == "FAILED":
        return _handle_failed(robot, skill_name)


def _init_take_off(robot, skill_name, kwargs):
    """初始化起飞技能"""
    robot.skill_states[skill_name] = "EXECUTING_1"
    try:
        # 获取目标高度参数
        altitude = kwargs.get("altitude")
        if altitude is None:
            robot.skill_states[skill_name] = "FAILED"
            robot.skill_errors[skill_name] = "Altitude parameter is required"
            return

        # 验证高度参数
        try:
            altitude = float(altitude)
            if altitude <= 0:
                robot.skill_states[skill_name] = "FAILED"
                robot.skill_errors[skill_name] = "Altitude must be greater than 0"
                return
        except (ValueError, TypeError):
            robot.skill_states[skill_name] = "FAILED"
            robot.skill_errors[skill_name] = "Invalid altitude parameter"
            return

        # 检查机器人身体组件
        if not hasattr(robot, "body") or robot.body is None:
            robot.skill_states[skill_name] = "FAILED"
            robot.skill_errors[skill_name] = "Robot body is not available"
            return

        # 存储到技能私有数据
        robot.set_skill_data(skill_name, "target_altitude", altitude)

        # 获取当前位置
        current_pos, current_quat = robot.body.get_world_pose()
        current_pos = current_pos.cpu().numpy().tolist()
        current_quat = current_quat.cpu().numpy().tolist()

        # 构造目标位置：保持XY，改变Z
        take_off_goal = [
            current_pos[0],  # 保持当前X
            current_pos[1],  # 保持当前Y
            float(altitude),  # 目标高度
        ]

        logger.info(f"Take off: current={current_pos}, target={take_off_goal}")

        # 存储目标位置和导航参数
        robot.set_skill_data(skill_name, "take_off_goal", take_off_goal)
        robot.set_skill_data(
            skill_name,
            "navigate_to_kwargs",
            {"goal_pos": take_off_goal, "goal_quat_wxyz": current_quat},  # 保持当前朝向
        )

    except Exception as e:
        robot.skill_states[skill_name] = "FAILED"
        robot.skill_errors[skill_name] = f"Take off initialization failed: {str(e)}"
        logger.error(f"Take off initialization error: {e}")


def _handle_executing_1(robot, skill_name):
    try:
        # 检查是否已经启动导航子技能
        navigate_to_skill_started = robot.get_skill_data(skill_name, "navigate_to_skill_started", False)

        if not navigate_to_skill_started:
            # 检查是否已有导航技能在运行
            if "navigate_to" in robot.skill_states:
                return robot.form_feedback(
                    "processing", "navigate_to busy, wait to be available...", 20
                )

            # 启动导航子技能
            navigate_to_kwargs = robot.get_skill_data(skill_name, "navigate_to_kwargs")
            robot.start_skill(navigate_to, **navigate_to_kwargs)
            robot.set_skill_data(skill_name, "navigate_to_skill_started", True)
            robot.set_skill_data(skill_name, "navigate_to_start_time", robot.sim_time)

            return robot.form_feedback(
                "processing", "Starting navigate_to for take off...", 30
            )

        # 检查导航子技能状态
        navigate_to_state = robot.skill_states.get("navigate_to")

        if navigate_to_state == "INITIALIZING":
            return robot.form_feedback("processing", "Navigate_to initializing...", 40)
        elif navigate_to_state == "EXECUTING":
            robot.skill_states[skill_name] = "EXECUTING"
            return robot.form_feedback(
                "processing", "Navigate_to started, taking off...", 50
            )
        elif navigate_to_state == "FAILED":
            navigate_to_error = robot.skill_errors.get(
                "navigate_to", "Unknown navigate_to error"
            )
            robot.skill_states[skill_name] = "FAILED"
            robot.skill_errors[skill_name] = (
                f"Navigate_to initialization failed: {navigate_to_error}"
            )
            return robot.form_feedback("failed", robot.skill_errors[skill_name])
        else:
            # 导航还在初始化中
            elapsed = robot.sim_time - robot.get_skill_data(
                skill_name, "navigate_to_start_time", robot.sim_time
            )
            if elapsed > 30.0:  # 30秒超时
                robot.skill_states[skill_name] = "FAILED"
                robot.skill_errors[skill_name] = "Navigate_to initialization timeout"
                return robot.form_feedback("failed", robot.skill_errors[skill_name])

            return robot.form_feedback(
                "processing", "Waiting for navigate_to initialization...", 35
            )

    except Exception as e:
        robot.skill_states[skill_name] = "FAILED"
        robot.skill_errors[skill_name] = f"Take off initialization failed: {str(e)}"
        logger.error(f"Take off initialization error: {e}")
        return robot.form_feedback("failed", robot.skill_errors[skill_name])


def _handle_executing(robot, skill_name):
    """执行起飞 - 监控导航子技能"""
    try:
        # 检查导航子技能状态
        navigate_to_state = robot.skill_states.get("navigate_to")
        navigate_to_feedback = robot.skill_feedbacks.get("navigate_to", {})

        if navigate_to_state == "COMPLETED":
            robot.skill_states[skill_name] = "COMPLETED"
            return robot.form_feedback("processing", "Take off completed", 95)
        elif navigate_to_state == "FAILED":
            navigate_to_error = robot.skill_errors.get(
                "navigate_to", "Unknown navigate_to error"
            )
            robot.skill_states[skill_name] = "FAILED"
            robot.skill_errors[skill_name] = f"Navigate_to failed: {navigate_to_error}"
            return robot.form_feedback("failed", robot.skill_errors[skill_name])
        elif navigate_to_state == "EXECUTING":
            # 传递导航的进度，但调整消息
            navigate_to_progress = navigate_to_feedback.get("progress", 50)
            navigate_to_message = navigate_to_feedback.get("message", "Moving...")

            # 调整进度范围到50-95
            adjusted_progress = 50 + (navigate_to_progress / 100) * 45

            return robot.form_feedback(
                "processing", f"Taking off: {navigate_to_message}", int(adjusted_progress)
            )
        else:
            # 导航状态异常，检查超时
            navigate_to_start_time = robot.get_skill_data(
                skill_name, "navigate_to_start_time", robot.sim_time
            )
            elapsed = robot.sim_time - navigate_to_start_time

            if elapsed > 120.0:  # 2分钟超时
                robot.skill_states[skill_name] = "FAILED"
                robot.skill_errors[skill_name] = "Take off timeout"
                return robot.form_feedback("failed", robot.skill_errors[skill_name])

            return robot.form_feedback("processing", "Take off in progress...", 60)

    except Exception as e:
        robot.skill_states[skill_name] = "FAILED"
        robot.skill_errors[skill_name] = f"Take off execution failed: {str(e)}"
        logger.error(f"Take off execution error: {e}")
        return robot.form_feedback("failed", robot.skill_errors[skill_name])


def _handle_completed(robot, skill_name):
    """处理完成状态"""

    final_pos, _ = robot.body.get_world_pose()
    final_altitude = final_pos[2].item()

    robot.skill_states[skill_name] = "COMPLETED"
    success_msg = f"Take off completed at {final_altitude:.1f}m altitude"
    logger.info(success_msg)
    return robot.form_feedback("completed", success_msg, 100)


def _handle_failed(robot, skill_name):
    """处理失败状态"""
    error_msg = robot.skill_errors.get(skill_name, "Unknown error")
    logger.error(f"Take off failed: {error_msg}")
    return robot.form_feedback("failed", error_msg)
