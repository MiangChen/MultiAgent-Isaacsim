import json

# ROS2 Message
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose


def navigate_to_skill(**kwargs):
    robot = kwargs.get("robot")
    
    # 初始化状态机（只在第一次调用时执行）
    if not hasattr(robot, '_nav_state'):
        _init_navigation(robot, kwargs)
    
    # 状态机：每个physics step执行一次
    if robot._nav_state == "SENDING":
        return _handle_sending(robot)
    elif robot._nav_state == "PLANNING":
        return _handle_planning(robot)
    elif robot._nav_state == "EXECUTING":
        return _handle_executing(robot)
    elif robot._nav_state == "COMPLETED":
        return _handle_completed(robot)
    elif robot._nav_state == "FAILED":
        return _handle_failed(robot)


def _init_navigation(robot, kwargs):
    start_pos = kwargs.get("start_pos", None)
    robot.node_controller_mpc.has_reached_goal = False
    if type(start_pos) is str:
        start_pos = json.loads(start_pos)
    start_quat = kwargs.get("start_quat")
    if type(start_quat) is str:
        start_quat = json.loads(start_quat)
    goal_pos = kwargs.get("goal_pos")
    goal_quat_wxyz = kwargs.get("goal_quat_wxyz", [1.0, 0.0, 0.0, 0.0])
    if isinstance(goal_pos, str):
        goal_pos = json.loads(goal_pos)
    elif isinstance(goal_pos, tuple):
        goal_pos = list(goal_pos)
    if isinstance(goal_quat_wxyz, str):
        goal_quat_wxyz = json.loads(goal_quat_wxyz)
    elif isinstance(goal_quat_wxyz, tuple):
        goal_quat_wxyz = list(goal_quat_wxyz)

    if robot.is_planning:
        robot._nav_state = "FAILED"
        robot._nav_error = "Planning already in progress."
        return

    if not robot.action_client_path_planner.wait_for_server(timeout_sec=2.0):
        robot._nav_state = "FAILED"
        robot._nav_error = "Path planner server is not available."
        return

    robot.is_planning = True
    robot.node_controller_mpc.move_event.clear()
    robot.node.get_logger().info(f"Sending path request to goal: {goal_pos}")

    goal_msg = ComputePathToPose.Goal()

    if start_pos is None:
        start_pos_tensor, _ = robot.body.get_world_pose()
        start_pos = start_pos_tensor.cpu().numpy().tolist()
    if start_quat is None:
        _, start_quat_tensor = robot.body.get_world_pose()
        start_quat = start_quat_tensor.cpu().numpy().tolist()

    goal_msg.start = _create_pose_stamped(robot, start_pos, start_quat)
    goal_msg.goal = _create_pose_stamped(robot, goal_pos, goal_quat_wxyz)

    # 发送规划请求
    robot._nav_send_future = robot.action_client_path_planner.send_goal_async(goal_msg)
    robot._nav_start_time = robot.sim_time
    robot._nav_state = "SENDING"


def _handle_sending(robot):
    if robot._nav_send_future.done():
        goal_handle = robot._nav_send_future.result()
        if goal_handle.accepted:
            robot._nav_result_future = goal_handle.get_result_async()
            robot._nav_goal_handle = goal_handle
            robot._nav_state = "PLANNING"
        else:
            robot._nav_state = "FAILED"
            robot._nav_error = "Request rejected"
    else:
        elapsed = robot.sim_time - robot._nav_start_time
        if elapsed > 3.0:
            robot._nav_state = "FAILED"
            robot._nav_error = "No response from planner"
    
    return robot.form_feedback("processing", "Sending request...", 5)


def _handle_planning(robot):
    if robot._nav_result_future.done():
        result = robot._nav_result_future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED and result.result.path.poses:
            robot.node_controller_mpc.move_event.clear()
            robot._nav_move_start_time = robot.sim_time
            robot._nav_state = "EXECUTING"
        else:
            robot._nav_state = "FAILED"
            robot._nav_error = "Planning failed"
    else:
        elapsed = robot.sim_time - robot._nav_start_time
        if elapsed > 15.0:
            robot._nav_goal_handle.cancel_goal_async()
            robot._nav_state = "FAILED"
            robot._nav_error = "Planning timeout"
    
    return robot.form_feedback("processing", "Planning path...", 30)


def _handle_executing(robot):
    if robot.node_controller_mpc.move_event.is_set():
        robot._nav_state = "COMPLETED"
        return robot.form_feedback("processing", "Executing...", 50)
    else:
        elapsed = robot.sim_time - robot._nav_move_start_time
        if elapsed > 120.0:
            robot._nav_state = "FAILED"
            robot._nav_error = "Navigation timeout"
        else:
            progress = min(50 + (elapsed / 120.0) * 45, 95)
            return robot.form_feedback("processing", f"Moving... ({elapsed:.1f}s)", int(progress))
    
    return robot.form_feedback("processing", "Executing...", 50)


def _handle_completed(robot):
    _cleanup_navigation(robot)
    return robot.form_feedback("finished", "Done", 100)


def _handle_failed(robot):
    error_msg = getattr(robot, '_nav_error', "Unknown error")
    _cleanup_navigation(robot)
    return robot.form_feedback("failed", error_msg)


def _cleanup_navigation(robot):
    robot.is_planning = False
    attrs_to_remove = ['_nav_state', '_nav_start_time', '_nav_send_future', 
                       '_nav_result_future', '_nav_goal_handle', '_nav_move_start_time', '_nav_error']
    for attr in attrs_to_remove:
        if hasattr(robot, attr):
            delattr(robot, attr)


def _create_pose_stamped(robot, pos: list, quat_wxyz: list = [0.0, 0.0, 0.0, 0.0]):
    pose_stamped = PoseStamped()

    sim_time_sec = int(robot.sim_time)
    sim_time_nanosec = int((robot.sim_time - sim_time_sec) * 1e9)
    pose_stamped.header.stamp = Time(sec=sim_time_sec, nanosec=sim_time_nanosec)

    pose_stamped.header.frame_id = "map"

    pose_stamped.pose.position.x = float(pos[0])
    pose_stamped.pose.position.y = float(pos[1])
    pose_stamped.pose.position.z = float(pos[2])

    pose_stamped.pose.orientation.w = float(quat_wxyz[0])
    pose_stamped.pose.orientation.x = float(quat_wxyz[1])
    pose_stamped.pose.orientation.y = float(quat_wxyz[2])
    pose_stamped.pose.orientation.z = float(quat_wxyz[3])
    return pose_stamped
