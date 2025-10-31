from typing import Dict, Any
from log.log_manager import LogManager

logger = LogManager.get_logger(__name__)


def broadcast_skill(**kwargs):
    robot = kwargs.get("robot")
    
    # 初始化状态机（只在第一次调用时执行）
    if not hasattr(robot, 'skill_state'):
        _init_broadcast(robot, kwargs)
    
    # 状态机：每个physics step执行一次
    if robot.skill_state == "INITIALIZING":
        return _handle_initializing(robot)
    elif robot.skill_state == "EXECUTING":
        return _handle_executing(robot)
    elif robot.skill_state == "COMPLETED":
        return _handle_completed(robot)
    elif robot.skill_state == "FAILED":
        return _handle_failed(robot)


def _init_broadcast(robot, kwargs):
    """初始化广播技能"""
    robot._broadcast_content = kwargs.get("content")
    robot.skill_state = "INITIALIZING"


def _handle_initializing(robot):
    """初始化阶段：检查广播内容和机器人配置"""
    try:
        # 检查广播内容
        if not robot._broadcast_content:
            robot.skill_state = "FAILED"
            robot.skill_error = "Broadcast content is not provided."
            return robot.form_feedback("failed", robot.skill_error)
        
        # 检查机器人配置
        if not hasattr(robot, 'body') or not hasattr(robot.body, 'cfg_robot'):
            robot.skill_state = "FAILED"
            robot.skill_error = "Robot configuration is not available."
            return robot.form_feedback("failed", robot.skill_error)
        
        # 初始化完成，进入执行阶段
        robot.skill_state = "EXECUTING"
        return robot.form_feedback("processing", "Initializing broadcast...", 20)
        
    except Exception as e:
        robot.skill_state = "FAILED"
        robot.skill_error = f"Broadcast initialization failed: {str(e)}"
        return robot.form_feedback("failed", robot.skill_error)


def _handle_executing(robot):
    """执行广播操作"""
    try:
        # 执行广播
        robot_name = robot.body.cfg_robot.name
        logger.info(
            f"[Skill] {robot_name} executing broadcasting. The content is {robot._broadcast_content}"
        )
        
        # 存储广播结果
        robot._broadcast_result = True
        robot.skill_state = "COMPLETED"
        
        return robot.form_feedback("processing", "Broadcasting message...", 90)
        
    except Exception as e:
        robot.skill_state = "FAILED"
        robot.skill_error = f"Broadcast execution failed: {str(e)}"
        return robot.form_feedback("failed", robot.skill_error)


def _handle_completed(robot):
    """处理完成状态"""
    result = getattr(robot, '_broadcast_result', False)
    _cleanup_broadcast(robot)
    return robot.form_feedback("finished", "Broadcast completed successfully!", 100)


def _handle_failed(robot):
    """处理失败状态"""
    error_msg = getattr(robot, 'skill_error', "Unknown error")
    _cleanup_broadcast(robot)
    return robot.form_feedback("failed", error_msg)


def _cleanup_broadcast(robot):
    """清理广播技能的状态"""
    attrs_to_remove = ['_broadcast_content', '_broadcast_result']
    for attr in attrs_to_remove:
        if hasattr(robot, attr):
            delattr(robot, attr)
    
    robot.skill_state = None
    robot.skill_error = None
