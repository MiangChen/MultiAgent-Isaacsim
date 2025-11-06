from log.log_manager import LogManager
from robot.skill.skill_registry import SkillRegistry

logger = LogManager.get_logger(__name__)

@SkillRegistry.register(['jetbot', 'g1', 'h1', 'cf2x'])
def broadcast(**kwargs):
    robot = kwargs.get("robot")
    skill_name = "broadcast_skill"

    current_state = robot.skill_states.get(skill_name)

    # 初始化状态机（只在第一次调用时执行）
    if current_state in [None, "INITIALIZING"]:
        _init_broadcast(robot, skill_name, kwargs)

    current_state = robot.skill_states.get(skill_name)
    if current_state == "INITIALIZING":
        return _handle_initializing(robot, skill_name)
    elif current_state == "EXECUTING":
        return _handle_executing(robot, skill_name)
    elif current_state == "COMPLETED":
        return _handle_completed(robot, skill_name)
    elif current_state == "FAILED":
        return _handle_failed(robot, skill_name)


def _init_broadcast(robot, skill_name, kwargs):
    """初始化广播技能"""
    broadcast_content = kwargs.get("content")

    # 存储参数到技能私有数据
    robot.set_skill_data(skill_name, "broadcast_content", broadcast_content)

    robot.skill_states[skill_name] = "INITIALIZING"


def _handle_initializing(robot, skill_name):
    """初始化阶段：检查广播内容和机器人配置"""
    try:
        # 获取参数
        broadcast_content = robot.get_skill_data(skill_name, "broadcast_content")

        # 检查广播内容
        if not broadcast_content:
            robot.skill_states[skill_name] = "FAILED"
            robot.skill_errors[skill_name] = "Broadcast content is not provided."
            return robot.form_feedback("failed", robot.skill_errors[skill_name])

        # 检查机器人配置
        if not hasattr(robot, "body") or not hasattr(robot.body, "cfg_robot"):
            robot.skill_states[skill_name] = "FAILED"
            robot.skill_errors[skill_name] = "Robot configuration is not available."
            return robot.form_feedback("failed", robot.skill_errors[skill_name])

        # 初始化完成，进入执行阶段
        robot.skill_states[skill_name] = "EXECUTING"
        return robot.form_feedback("processing", "Initializing broadcast...", 20)

    except Exception as e:
        robot.skill_states[skill_name] = "FAILED"
        robot.skill_errors[skill_name] = f"Broadcast initialization failed: {str(e)}"
        return robot.form_feedback("failed", robot.skill_errors[skill_name])


def _handle_executing(robot, skill_name):
    """执行广播操作"""
    try:
        # 获取存储的数据
        broadcast_content = robot.get_skill_data(skill_name, "broadcast_content")

        # 执行广播
        robot_name = robot.body.cfg_robot.namespace
        logger.info(
            f"[Skill] {robot_name} executing broadcasting. The content is {broadcast_content}"
        )

        # 存储广播结果
        robot.set_skill_data(skill_name, "broadcast_result", True)
        robot.skill_states[skill_name] = "COMPLETED"

        return robot.form_feedback("processing", "Broadcasting message...", 90)

    except Exception as e:
        robot.skill_states[skill_name] = "FAILED"
        robot.skill_errors[skill_name] = f"Broadcast execution failed: {str(e)}"
        return robot.form_feedback("failed", robot.skill_errors[skill_name])


def _handle_completed(robot, skill_name):
    """处理完成状态"""
    result = robot.get_skill_data(skill_name, "broadcast_result", False)
    return robot.form_feedback("completed", "Broadcast completed successfully!", 100)


def _handle_failed(robot, skill_name):
    """处理失败状态"""
    error_msg = robot.skill_errors.get(skill_name, "Unknown error")
    return robot.form_feedback("failed", error_msg)
