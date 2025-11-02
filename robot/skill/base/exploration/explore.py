from .plan_exploration_waypoints import plan_exploration_waypoints_skill


def explore_skill(**kwargs):
    robot = kwargs.get("robot")

    # 初始化状态机（只在第一次调用时执行）
    if robot.skill_state is None:
        _init_explore(robot, kwargs)

    if robot.skill_state == "EXECUTING":
        return _handle_executing(robot)
    elif robot.skill_state == "COMPLETED":
        return _handle_completed(robot)
    elif robot.skill_state == "FAILED":
        return _handle_failed(robot)


def _init_explore(robot, kwargs):
    """初始化探索技能"""
    robot._boundary = kwargs.get("boundary")
    robot._holes = kwargs.get("holes", [])
    robot._target_prim = kwargs.get("target_prim", "/TARGET_PRIM_NOT_SPECIFIED")
    robot._exploration_start_time = None
    robot.skill_state = "INITIALIZING"

    try:
        # 检查边界参数
        if not robot._boundary:
            robot.skill_state = "FAILED"
            robot.skill_error = "Boundary is not provided."
            return robot.form_feedback("failed", robot.skill_error)

        # 检查必要的机器人组件
        if not hasattr(robot, 'body') or not hasattr(robot.body, 'cfg_robot'):
            robot.skill_state = "FAILED"
            robot.skill_error = "Robot configuration is not available."
            return robot.form_feedback("failed", robot.skill_error)

        # 检查控制器和规划器
        if not hasattr(robot, 'node_controller_mpc') or not hasattr(robot, 'node_planner_ompl'):
            robot.skill_state = "FAILED"
            robot.skill_error = "Robot controller or planner is not available."
            return robot.form_feedback("failed", robot.skill_error)

        # 规划探索路径点
        robot._waypoints = plan_exploration_waypoints_skill(
            robot=robot,
            polygon_coords=robot._boundary,
            holes=robot._holes,
            lane_width=robot.body.cfg_robot.detection_radius,
            robot_radius=robot.body.cfg_robot.robot_radius,
        )

        if not robot._waypoints:
            robot.skill_state = "FAILED"
            robot.skill_error = "Failed to plan exploration waypoints."
            return robot.form_feedback("failed", robot.skill_error)

        # 初始化完成，进入执行阶段
        robot.skill_state = "EXECUTING"
        return robot.form_feedback("processing", "Exploration waypoints planned.", 30)

    except Exception as e:
        robot.skill_state = "FAILED"
        robot.skill_error = f"Exploration initialization failed: {str(e)}"
        return robot.form_feedback("failed", robot.skill_error)


def _handle_executing(robot):
    """执行探索操作"""
    try:
        # 如果是第一次执行，设置探索参数并发布路径
        if robot._exploration_start_time is None:
            robot.is_detecting = True
            robot.target_prim = robot._target_prim
            robot.node_controller_mpc.move_event.clear()
            robot.node_planner_ompl.publisher_path.publish(robot._waypoints)
            robot._exploration_start_time = robot.sim_time
            return robot.form_feedback("processing", "Starting exploration...", 40)

        # 检查探索是否完成
        if robot.node_controller_mpc.move_event.is_set():
            robot.skill_state = "COMPLETED"
            return robot.form_feedback("processing", "Exploration in progress...", 90)

        # 检查超时
        elapsed = robot.sim_time - robot._exploration_start_time
        if elapsed > 100.0:  # 100秒超时
            robot.skill_state = "FAILED"
            robot.skill_error = "Exploration timeout"
            return robot.form_feedback("failed", robot.skill_error)

        # 计算进度
        progress = min(40 + (elapsed / 100.0) * 50, 90)
        return robot.form_feedback("processing", f"Exploring... ({elapsed:.1f}s)", int(progress))

    except Exception as e:
        robot.skill_state = "FAILED"
        robot.skill_error = f"Exploration execution failed: {str(e)}"
        return robot.form_feedback("failed", robot.skill_error)


def _handle_completed(robot):
    """处理完成状态"""
    _cleanup_explore(robot)
    return robot.form_feedback("finished", "Exploration completed.", 100)


def _handle_failed(robot):
    """处理失败状态"""
    error_msg = getattr(robot, 'skill_error', "Unknown error")
    _cleanup_explore(robot)
    return robot.form_feedback("failed", error_msg)


def _cleanup_explore(robot):
    """清理探索技能的状态"""
    attrs_to_remove = ['_boundary', '_holes', '_target_prim', '_waypoints', '_exploration_start_time']
    for attr in attrs_to_remove:
        if hasattr(robot, attr):
            delattr(robot, attr)

    # 重置探索相关的机器人状态
    if hasattr(robot, 'is_detecting'):
        robot.is_detecting = False
    if hasattr(robot, 'target_prim'):
        robot.target_prim = None

    robot.skill_state = None
    robot.skill_error = None
