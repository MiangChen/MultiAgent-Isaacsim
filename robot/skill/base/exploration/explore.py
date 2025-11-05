import ast
import json
from .plan_exploration_waypoints import plan_exploration_waypoints_skill
from ..navigation.navigate_to import navigate_to_skill
from log.log_manager import LogManager

logger = LogManager.get_logger(__name__)


def explore_skill(**kwargs):
    robot = kwargs.get("robot")
    skill_name = "explore_skill"

    current_state = robot.skill_states.get(skill_name)

    # 初始化状态机（只在第一次调用时执行）
    if current_state in [None, "INITIALIZING"]:
        _init_explore(robot, skill_name, kwargs)

    current_state = robot.skill_states.get(skill_name)
    if current_state == "EXECUTING1":
        return _handle_executing1(robot, skill_name)
    elif current_state == "EXECUTING2":
        return _handle_executing2(robot, skill_name)
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
        # 获取插值参数
        interpolation_distance = kwargs.get("interpolation_distance", 0.05)  # 默认5cm
        interpolation_method = kwargs.get("interpolation_method", "linear")  # 默认线性插值
        
        # 规划探索路径点
        waypoints = plan_exploration_waypoints_skill(
            robot=robot,
            polygon_coords=boundary,
            holes=holes,
            lane_width=robot.body.cfg_robot.detection_radius,
            robot_radius=robot.body.cfg_robot.robot_radius,
            interpolation_distance=interpolation_distance,
            interpolation_method=interpolation_method,
        )
        if not waypoints or not waypoints.poses:
            robot.skill_states[skill_name] = "FAILED"
            robot.skill_errors[skill_name] = "Failed to plan exploration waypoints."
            return robot.form_feedback("failed", robot.skill_errors[skill_name])

        # 获取第一个waypoint作为起点
        first_waypoint = waypoints.poses[0]
        start_pos = [
            first_waypoint.pose.position.x,
            first_waypoint.pose.position.y,
            first_waypoint.pose.position.z
        ]
        start_quat = [
            first_waypoint.pose.orientation.w,  # wxyz格式
            first_waypoint.pose.orientation.x,
            first_waypoint.pose.orientation.y,
            first_waypoint.pose.orientation.z
        ]

        # 存储导航到起点的参数
        robot.set_skill_data(skill_name, "nav_to_start_kwargs", {
            "goal_pos": start_pos,
            "goal_quat_wxyz": start_quat
        })

        # 存 skill 私有数据
        robot.set_skill_data(skill_name, "boundary", boundary)
        robot.set_skill_data(skill_name, "holes", holes)
        robot.set_skill_data(skill_name, "target_prim", target_prim)
        robot.set_skill_data(skill_name, "waypoints", waypoints)
        robot.set_skill_data(skill_name, "exploration_start_time", None)
        robot.set_skill_data(skill_name, "nav_to_start_completed", False)

        # 初始化完成，进入导航到起点阶段
        robot.skill_states[skill_name] = "EXECUTING1"
        return robot.form_feedback("processing", "Exploration waypoints planned, preparing to navigate to start point.", 20)

    except Exception as e:
        robot.skill_states[skill_name] = "FAILED"
        robot.skill_errors[skill_name] = f"Exploration initialization failed: {str(e)}"
        return robot.form_feedback("failed", robot.skill_errors[skill_name])


def _handle_executing1(robot, skill_name):
    """处理导航到起点状态 - 模仿take_off的导航逻辑"""
    try:
        # 检查是否已经启动导航子技能
        nav_skill_started = robot.get_skill_data(skill_name, "nav_to_start_skill_started", False)
        
        if not nav_skill_started:
            # 检查是否已有导航技能在运行
            if "navigate_to_skill" in robot.skill_states:
                return robot.form_feedback("processing", "Waiting for navigation to be available...", 25)
            
            # 启动导航子技能
            nav_kwargs = robot.get_skill_data(skill_name, "nav_to_start_kwargs")
            robot.start_skill(navigate_to_skill, **nav_kwargs)
            robot.set_skill_data(skill_name, "nav_to_start_skill_started", True)
            robot.set_skill_data(skill_name, "nav_to_start_time", robot.sim_time)
            
            return robot.form_feedback("processing", "Starting navigation to exploration start point...", 30)
        
        # 检查导航子技能状态
        nav_state = robot.skill_states.get("navigate_to_skill")
        nav_feedback = robot.skill_feedbacks.get("navigate_to_skill", {})
        
        if nav_state == "INITIALIZING":
            return robot.form_feedback("processing", "Navigation to start point initializing...", 35)
        elif nav_state == "EXECUTING":
            # 传递导航的进度，但调整消息和进度范围
            nav_progress = nav_feedback.get('progress', 50)
            nav_message = nav_feedback.get('message', 'Moving to start point...')
            
            # 调整进度范围到35-60
            adjusted_progress = 35 + (nav_progress / 100) * 25
            
            return robot.form_feedback("processing", f"Navigating to start: {nav_message}", int(adjusted_progress))
        elif nav_state == "COMPLETED":
            # 导航到起点完成，开始探索
            robot.set_skill_data(skill_name, "nav_to_start_completed", True)
            robot.skill_states[skill_name] = "EXECUTING2"
            return robot.form_feedback("processing", "Reached exploration start point, beginning exploration...", 65)
        elif nav_state == "FAILED":
            nav_error = robot.skill_errors.get("navigate_to_skill", "Unknown navigation error")
            robot.skill_states[skill_name] = "FAILED"
            robot.skill_errors[skill_name] = f"Navigation to start point failed: {nav_error}"
            return robot.form_feedback("failed", robot.skill_errors[skill_name])
        else:
            # 导航还在初始化中，检查超时
            elapsed = robot.sim_time - robot.get_skill_data(skill_name, "nav_to_start_time", robot.sim_time)
            if elapsed > 60.0:  # 60秒超时
                robot.skill_states[skill_name] = "FAILED"
                robot.skill_errors[skill_name] = "Navigation to start point timeout"
                return robot.form_feedback("failed", robot.skill_errors[skill_name])
            
            return robot.form_feedback("processing", "Waiting for navigation to start point...", 32)
            
    except Exception as e:
        robot.skill_states[skill_name] = "FAILED"
        robot.skill_errors[skill_name] = f"Navigation to start point failed: {str(e)}"
        logger.error(f"Navigation to start point error: {e}")
        return robot.form_feedback("failed", robot.skill_errors[skill_name])


def _handle_executing2(robot, skill_name):
    """执行探索操作（对齐 navigate.py 的执行节奏和进度反馈）"""
    try:
        start_time = robot.get_skill_data(skill_name, "exploration_start_time")
        waypoints = robot.get_skill_data(skill_name, "waypoints")
        target_prim = robot.get_skill_data(skill_name, "target_prim", "/TARGET_PRIM_NOT_SPECIFIED")

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
            return robot.form_feedback("processing", "Starting exploration coverage...", 70)

        # 是否完成（控制器在到达末端或完成策略时会 set 该事件）
        if robot.node_controller_mpc.move_event.is_set():
            robot.skill_states[skill_name] = "COMPLETED"
            return robot.form_feedback("processing", "Exploration in progress...", 95)

        elapsed = robot.sim_time - start_time
        timeout_sec = 120.0  # 增加超时时间，因为现在包含了更多路径点
        if elapsed > timeout_sec:
            robot.skill_states[skill_name] = "FAILED"
            robot.skill_errors[skill_name] = "Exploration timeout"
            return robot.form_feedback("failed", robot.skill_errors[skill_name])

        # 进度：70 -> 95 (调整进度范围)
        progress = min(70 + (elapsed / timeout_sec) * 25, 95)
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
