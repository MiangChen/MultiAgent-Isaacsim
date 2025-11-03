import json
import functools
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import Odometry

def track_callback(robot, msg):
    pos = (
        msg.pose.pose.position.x,
        msg.pose.pose.position.y,
        msg.pose.pose.position.z,
    )
    robot.track_counter += 1
    if robot.track_counter % robot.track_period == 0:
        robot.track_waypoint_list.append(pos)

def track_skill(**kwargs):
    robot = kwargs.get("robot")
    
    # 初始化状态机（只在第一次调用时执行）
    if robot.skill_state is None:
        _init_track(robot, kwargs)
    
    canceled = kwargs.get("cancel", False)
    if canceled:
        return _cancel_track(robot)
        
    # 状态机：每个physics step执行一次
    if robot.skill_state == "IDLE":
        return _handle_idle(robot)
    elif robot.skill_state == "INITIALIZING":
        return _handle_initializing(robot)
    elif robot.skill_state == "EXECUTING":
        return _handle_executing(robot)
    elif robot.skill_state == "COMPLETED":
        return _handle_completed(robot)
    elif robot.skill_state == "FAILED":
        return _handle_failed(robot)


def _init_track(robot, kwargs):
    """初始化跟踪任务"""
    # 初始化跟踪相关的属性
    robot.track_waypoint_list = []
    robot.track_current_waypoint_index = 0
    track_callback_partial = functools.partial(track_callback, robot = robot)
    robot.track_waypoint_sub = robot.node.create_subscription(
        Odometry,
        "/target_0/odom",
        track_callback_partial,
        50,
    )
    robot.skill_state = "IDLE"
    robot.node.get_logger().info("Track skill initialized, waiting for waypoints...")


def _handle_idle(robot):
    """空闲状态：等待waypoint列表中有新的点，然后开始导航"""
    # 检查是否有新的航点可以导航
    if len(robot.track_waypoint_list) > robot.track_current_waypoint_index:
        # 有新的航点，开始导航
        goal_pos = robot.track_waypoint_list[robot.track_current_waypoint_index]
        _start_navigation_to_waypoint(robot, goal_pos)
        return robot.form_feedback("processing", f"Starting navigation to waypoint {robot.track_current_waypoint_index}", 10)
    else:
        # 没有新航点，继续等待
        return robot.form_feedback("processing", "Waiting for target waypoints...", 5)


def _start_navigation_to_waypoint(robot, goal_pos):
    """开始导航到指定航点"""
    robot.node_controller_mpc.has_reached_goal = False
    
    # 将tuple转换为list
    if isinstance(goal_pos, tuple):
        goal_pos = list(goal_pos)
    
    goal_quat_wxyz = [1.0, 0.0, 0.0, 0.0]
    
    # 检查路径规划服务器是否可用
    if not robot.action_client_path_planner.wait_for_server(timeout_sec=2.0):
        robot.skill_state = "FAILED"
        robot.skill_error = "Path planner server is not available."
        return
    
    robot.node_controller_mpc.move_event.clear()
    robot.node.get_logger().info(f"Tracking: Sending path request to waypoint {robot.track_current_waypoint_index}: {goal_pos}")
    
    # 创建路径规划请求
    goal_msg = ComputePathToPose.Goal()
    
    # 获取当前位置作为起点
    start_pos_tensor, start_quat_tensor = robot.body.get_world_pose()
    start_pos = start_pos_tensor.cpu().numpy().tolist()
    start_quat = start_quat_tensor.cpu().numpy().tolist()
    
    goal_msg.start = _create_pose_stamped(robot, start_pos, start_quat)
    goal_msg.goal = _create_pose_stamped(robot, goal_pos, goal_quat_wxyz)
    
    # 发送规划请求
    robot._track_nav_send_future = robot.action_client_path_planner.send_goal_async(goal_msg)
    robot._track_nav_start_time = robot.sim_time
    robot.skill_state = "INITIALIZING"


def _handle_initializing(robot):
    """处理初始化状态：等待路径规划请求被接受并完成规划"""
    # 检查发送请求是否完成
    if not hasattr(robot, '_track_nav_result_future'):
        if robot._track_nav_send_future.done():
            goal_handle = robot._track_nav_send_future.result()
            if goal_handle.accepted:
                robot._track_nav_result_future = goal_handle.get_result_async()
                robot._track_nav_goal_handle = goal_handle
                return robot.form_feedback("processing", "Planning path...", 15)
            else:
                robot.skill_state = "FAILED"
                robot.skill_error = "Request rejected"
                return robot.form_feedback("failed", "Request rejected")
        else:
            elapsed = robot.sim_time - robot._track_nav_start_time
            if elapsed > 3.0:
                robot.skill_state = "FAILED"
                robot.skill_error = "No response from planner"
                return robot.form_feedback("failed", "No response from planner")
            return robot.form_feedback("processing", "Sending request...", 5)
    
    # 检查规划结果
    if robot._track_nav_result_future.done():
        result = robot._track_nav_result_future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED and result.result.path.poses:
            robot.node_controller_mpc.move_event.clear()
            robot._track_nav_move_start_time = robot.sim_time
            robot.skill_state = "EXECUTING"
        else:
            robot.skill_state = "FAILED"
            robot.skill_error = "Planning failed"
    else:
        elapsed = robot.sim_time - robot._track_nav_start_time
        if elapsed > 15.0:
            robot._track_nav_goal_handle.cancel_goal_async()
            robot.skill_state = "FAILED"
            robot.skill_error = "Planning timeout"
    
    return robot.form_feedback("processing", "Planning path...", 30)


def _handle_executing(robot):
    """处理执行状态：机器人正在移动到目标点"""
    if robot.node_controller_mpc.move_event.is_set():
        robot.skill_state = "COMPLETED"
        return robot.form_feedback("processing", "Executing...", 50)
    else:
        elapsed = robot.sim_time - robot._track_nav_move_start_time
        if elapsed > 120.0:
            robot.skill_state = "FAILED"
            robot.skill_error = "Navigation timeout"
        else:
            progress = min(50 + (elapsed / 120.0) * 45, 95)
            waypoint_info = f"to waypoint {robot.track_current_waypoint_index}"
            return robot.form_feedback("processing", f"Moving {waypoint_info}... ({elapsed:.1f}s)", int(progress))
    
    return robot.form_feedback("processing", "Executing...", 50)


def _handle_completed(robot):
    """处理完成状态：到达航点后准备下一个"""
    robot.node.get_logger().info(f"Reached waypoint {robot.track_current_waypoint_index}")
    robot.track_current_waypoint_index += 1
    _cleanup_tracking_navigation(robot)
    robot.skill_state = "IDLE"  # 回到IDLE状态，检查是否有下一个航点
    return robot.form_feedback("processing", f"Reached waypoint, checking for next...", 50)


def _handle_failed(robot):
    """处理失败状态：清理资源并返回错误信息"""
    error_msg = getattr(robot, 'skill_error', "Unknown error")
    _cleanup_tracking_navigation(robot)
    return robot.form_feedback("failed", error_msg)


def _cancel_track(robot):
    """取消跟踪任务：清理所有资源并停止跟踪"""
    robot.node.get_logger().info("Canceling track skill...")
    
    # 取消正在进行的导航（如果有）
    if hasattr(robot, '_track_nav_goal_handle'):
        robot._track_nav_goal_handle.cancel_goal_async()
    
    # 清理导航相关的临时属性
    _cleanup_tracking_navigation(robot)
    
    # 停止跟踪订阅
    robot.is_tracking = False
    robot.track_waypoint_list = []
    robot.track_current_waypoint_index = 0
    if hasattr(robot, 'track_waypoint_sub') and robot.track_waypoint_sub is not None:
        robot.node.destroy_subscription(robot.track_waypoint_sub)
        robot.track_waypoint_sub = None
    
    return robot.form_feedback("finished", "Tracking canceled")


def _cleanup_tracking_navigation(robot):
    """清理当前导航相关的临时属性"""
    attrs_to_remove = [
        '_track_nav_start_time', 
        '_track_nav_send_future', 
        '_track_nav_result_future', 
        '_track_nav_goal_handle', 
        '_track_nav_move_start_time'
    ]
    for attr in attrs_to_remove:
        if hasattr(robot, attr):
            delattr(robot, attr)
    
    robot.skill_state = None
    robot.skill_error = None


def _create_pose_stamped(robot, pos: list, quat_wxyz: list = [0.0, 0.0, 0.0, 0.0]):
    """创建PoseStamped消息"""
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