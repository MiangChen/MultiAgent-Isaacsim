import ast
import json
from .plan_exploration_waypoints import plan_exploration_waypoints_skill
from .ompl_coverage_planner import plan_ompl_coverage_waypoints
from .coverage_config import get_coverage_config, CoverageAlgorithm


def explore_skill(**kwargs):
    robot = kwargs.get("robot")
    skill_name = "explore_skill"

    current_state = robot.skill_states.get(skill_name)

    # 初始化状态机（只在第一次调用时执行）
    if current_state in [None, "INITIALIZING"]:
        _init_explore(robot, skill_name, kwargs)

    current_state = robot.skill_states.get(skill_name)
    if current_state == "EXECUTING":
        return _handle_executing(robot, skill_name)
    elif current_state == "COMPLETED":
        return _handle_completed(robot, skill_name)
    elif current_state == "FAILED":
        return _handle_failed(robot, skill_name)


def _init_explore(robot, skill_name, kwargs):
    """初始化探索技能（与 navigate.py 的风格对齐：使用 skill_states / skill_errors / skill_data）"""

    # 解析 boundary / holes / target_prim（字符串或结构）
    def _parse_list_like(v):
        if isinstance(v, str):
            # 优先 JSON，再退回 literal_eval，容错更好
            try:
                return json.loads(v)
            except Exception:
                try:
                    return ast.literal_eval(v)
                except Exception:
                    return None
        return v

    boundary = _parse_list_like(kwargs.get("boundary"))
    holes = _parse_list_like(kwargs.get("holes")) or []
    target_prim = kwargs.get("target_prim", "/TARGET_PRIM_NOT_SPECIFIED")

    # 基本参数校验
    if not boundary or not isinstance(boundary, (list, tuple)):
        robot.skill_states[skill_name] = "FAILED"
        robot.skill_errors[skill_name] = "Boundary is not provided or invalid."
        return robot.form_feedback("failed", robot.skill_errors[skill_name])

    if not hasattr(robot, 'body') or not hasattr(robot.body, 'cfg_robot'):
        robot.skill_states[skill_name] = "FAILED"
        robot.skill_errors[skill_name] = "Robot configuration is not available."
        return robot.form_feedback("failed", robot.skill_errors[skill_name])

    if not hasattr(robot, 'node_controller_mpc') or not hasattr(robot, 'node_planner_ompl'):
        robot.skill_states[skill_name] = "FAILED"
        robot.skill_errors[skill_name] = "Robot controller or planner is not available."
        return robot.form_feedback("failed", robot.skill_errors[skill_name])

    try:
        # 获取配置参数
        config_name = kwargs.get("config", "ground_robot")  # 默认地面机器人配置
        algorithm_override = kwargs.get("algorithm", None)  # 可选的算法覆盖
        
        # 获取覆盖规划配置
        coverage_config = get_coverage_config(config_name)
        
        # 如果指定了算法覆盖，则使用指定算法
        if algorithm_override:
            if algorithm_override.startswith("ompl_"):
                coverage_config.algorithm = CoverageAlgorithm(algorithm_override)
            elif algorithm_override == "custom":
                coverage_config.algorithm = CoverageAlgorithm.CUSTOM_BOUSTROPHEDON
        
        # 根据算法类型选择规划方法
        if coverage_config.algorithm in [CoverageAlgorithm.OMPL_STC, CoverageAlgorithm.OMPL_BSTAR]:
            # 使用OMPL算法
            ompl_algorithm = coverage_config.algorithm.value.replace("ompl_", "")
            waypoints = plan_ompl_coverage_waypoints(
                robot=robot,
                polygon_coords=boundary,
                holes=holes,
                robot_radius=coverage_config.robot_radius,
                algorithm=ompl_algorithm,
                coverage_resolution=coverage_config.coverage_resolution,
            )
        else:
            # 使用自定义算法（保持向后兼容）
            waypoints = plan_exploration_waypoints_skill(
                robot=robot,
                polygon_coords=boundary,
                holes=holes,
                lane_width=coverage_config.lane_width or coverage_config.coverage_resolution,
                robot_radius=coverage_config.robot_radius,
            )
        if not waypoints:
            robot.skill_states[skill_name] = "FAILED"
            robot.skill_errors[skill_name] = "Failed to plan exploration waypoints."
            return robot.form_feedback("failed", robot.skill_errors[skill_name])

        # 存 skill 私有数据
        robot.set_skill_data(skill_name, "boundary", boundary)
        robot.set_skill_data(skill_name, "holes", holes)
        robot.set_skill_data(skill_name, "target_prim", target_prim)
        robot.set_skill_data(skill_name, "waypoints", waypoints)
        robot.set_skill_data(skill_name, "exploration_start_time", None)

        # 初始化完成，进入执行阶段
        robot.skill_states[skill_name] = "EXECUTING"
        return robot.form_feedback("processing", "Exploration waypoints planned.", 30)

    except Exception as e:
        robot.skill_states[skill_name] = "FAILED"
        robot.skill_errors[skill_name] = f"Exploration initialization failed: {str(e)}"
        return robot.form_feedback("failed", robot.skill_errors[skill_name])


def _handle_executing(robot, skill_name):
    """执行探索操作（对齐 navigate.py 的执行节奏和进度反馈）"""
    try:
        start_time = robot.get_skill_data(skill_name, "exploration_start_time")
        waypoints = robot.get_skill_data(skill_name, "waypoints")
        target_prim = robot.get_skill_data(skill_name, "target_prim", "/TARGET_PRIM_NOT_SPECIFIED")

        # 第一次进入 EXECUTING：设置检测与发布路径
        if start_time is None:
            # 设置机器人检测模式与目标
            if hasattr(robot, 'is_detecting'):
                robot.is_detecting = True
            if hasattr(robot, 'target_prim'):
                robot.target_prim = target_prim

            # 清空移动事件并发布路径
            robot.node_controller_mpc.move_event.clear()
            robot.node_planner_ompl.publisher_path.publish(waypoints)

            # 记录开始时间
            robot.set_skill_data(skill_name, "exploration_start_time", robot.sim_time)
            return robot.form_feedback("processing", "Starting exploration...", 40)

        # 是否完成（控制器在到达末端或完成策略时会 set 该事件）
        if robot.node_controller_mpc.move_event.is_set():
            robot.skill_states[skill_name] = "COMPLETED"
            return robot.form_feedback("processing", "Exploration in progress...", 90)

        elapsed = robot.sim_time - start_time
        timeout_sec = 100.0
        if elapsed > timeout_sec:
            robot.skill_states[skill_name] = "FAILED"
            robot.skill_errors[skill_name] = "Exploration timeout"
            return robot.form_feedback("failed", robot.skill_errors[skill_name])

        # 进度：40 -> 90
        progress = min(40 + (elapsed / timeout_sec) * 70, 90)
        return robot.form_feedback("processing", f"Exploring... ({elapsed:.1f}s)", int(progress))

    except Exception as e:
        robot.skill_states[skill_name] = "FAILED"
        robot.skill_errors[skill_name] = f"Exploration execution failed: {str(e)}"
        return robot.form_feedback("failed", robot.skill_errors[skill_name])


def _handle_completed(robot, skill_name):
    """处理完成状态"""
    _cleanup_explore(robot, skill_name)
    return robot.form_feedback("completed", "Exploration completed.", 100)


def _handle_failed(robot, skill_name):
    """处理失败状态"""
    error_msg = robot.skill_errors.get(skill_name, "Unknown error")
    _cleanup_explore(robot, skill_name)
    return robot.form_feedback("failed", error_msg)


def _cleanup_explore(robot, skill_name):
    """清理探索技能的状态"""
    if hasattr(robot, 'is_detecting'):
        robot.is_detecting = False
    if hasattr(robot, 'target_prim'):
        robot.target_prim = None

    # 重置状态记录
    robot.skill_states[skill_name] = None
    robot.skill_errors[skill_name] = None
