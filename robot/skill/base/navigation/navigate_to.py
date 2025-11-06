import json

from log.log_manager import LogManager
from robot.skill.skill_registry import SkillRegistry

logger = LogManager().get_logger(__name__)

# ROS2 Message
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose


@SkillRegistry.register()
def navigate_to_skill(**kwargs):
    robot = kwargs.get("robot")
    skill_name = "navigate_to_skill"  # 直接使用函数名

    current_state = robot.skill_states.get(skill_name)

    # 初始化状态机（只在第一次调用时执行）
    if current_state in [None, "INITIALIZING"]:
        _init_navigation(robot, skill_name, kwargs)

    current_state = robot.skill_states.get(skill_name)
    if current_state == "EXECUTING":
        return _handle_executing(robot, skill_name)
    elif current_state == "COMPLETED":
        return _handle_completed(robot, skill_name)
    elif current_state == "FAILED":
        return _handle_failed(robot, skill_name)


def _init_navigation(robot, skill_name, kwargs):
    robot.node_controller_mpc.has_reached_goal = False
    robot.skill_states[skill_name] = "EXECUTING"
    start_pos = kwargs.get("start_pos", None)
    start_quat = kwargs.get("start_quat")
    if type(start_pos) is str:
        start_pos = json.loads(start_pos)
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

    if not robot.action_client_path_planner.wait_for_server(timeout_sec=2.0):
        robot.skill_states[skill_name] = "FAILED"
        robot.skill_errors[skill_name] = "Path planner server is not available."
        logger.info(robot.skill_errors[skill_name])
        return

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

    # 存储导航参数到技能私有数据
    robot.set_skill_data(skill_name, "goal_pos", goal_pos)
    robot.set_skill_data(skill_name, "goal_quat_wxyz", goal_quat_wxyz)
    robot.set_skill_data(skill_name, "start_pos", start_pos)
    robot.set_skill_data(skill_name, "start_quat", start_quat)

    # 发送规划请求
    nav_send_future = robot.action_client_path_planner.send_goal_async(goal_msg)
    nav_start_time = robot.sim_time

    # 存储导航状态到技能私有数据
    robot.set_skill_data(skill_name, "nav_send_future", nav_send_future)
    robot.set_skill_data(skill_name, "nav_start_time", nav_start_time)

    """处理初始化状态 - 检查路径规划结果"""
    nav_send_future = robot.get_skill_data(skill_name, "nav_send_future")
    nav_start_time = robot.get_skill_data(skill_name, "nav_start_time")

    nav_result_future = robot.get_skill_data(skill_name, "nav_result_future")

    if nav_result_future is None:
        if nav_send_future.done():
            goal_handle = nav_send_future.result()
            if goal_handle.accepted:
                nav_result_future = goal_handle.get_result_async()
                robot.set_skill_data(skill_name, "nav_result_future", nav_result_future)
                robot.set_skill_data(skill_name, "nav_goal_handle", goal_handle)
                return robot.form_feedback("processing", "Planning path...", 15)
            else:
                robot.skill_states[skill_name] = "FAILED"
                robot.skill_errors[skill_name] = "Request rejected"
                return robot.form_feedback("failed", "Request rejected")
        else:
            elapsed = robot.sim_time - nav_start_time
            if elapsed > 3.0:
                robot.skill_states[skill_name] = "FAILED"
                robot.skill_errors[skill_name] = "No response from planner"
                return robot.form_feedback("failed", "No response from planner")
            return robot.form_feedback("processing", "Sending request...", 5)

    # 检查规划结果
    if nav_result_future.done():
        result = nav_result_future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED and result.result.path.poses:
            robot.node_controller_mpc.move_event.clear()
            robot.set_skill_data(skill_name, "nav_move_start_time", robot.sim_time)
            robot.skill_states[skill_name] = "EXECUTING"
        else:
            robot.skill_states[skill_name] = "FAILED"
            robot.skill_errors[skill_name] = "Planning failed"
    else:
        elapsed = robot.sim_time - nav_start_time
        if elapsed > 15.0:
            nav_goal_handle = robot.get_skill_data(skill_name, "nav_goal_handle")
            if nav_goal_handle:
                nav_goal_handle.cancel_goal_async()
            robot.skill_states[skill_name] = "FAILED"
            robot.skill_errors[skill_name] = "Planning timeout"

    return robot.form_feedback("processing", "Planning path...", 30)


def _handle_executing(robot, skill_name):
    """处理执行状态"""
    if robot.node_controller_mpc.move_event.is_set():
        robot.skill_states[skill_name] = "COMPLETED"
        return robot.form_feedback("processing", "Executing...", 50)
    else:
        nav_move_start_time = robot.get_skill_data(skill_name, "nav_move_start_time", 0)
        elapsed = robot.sim_time - nav_move_start_time
        if elapsed > 120.0:
            robot.skill_states[skill_name] = "FAILED"
            robot.skill_errors[skill_name] = "Navigation timeout"
        else:
            progress = min(50 + (elapsed / 120.0) * 45, 95)
            return robot.form_feedback(
                "processing", f"Moving... ({elapsed:.1f}s)", int(progress)
            )

    return robot.form_feedback("processing", "Executing...", 50)


def _handle_completed(robot, skill_name):
    """处理完成状态"""
    return robot.form_feedback("completed", "Navigation completed", 100)


def _handle_failed(robot, skill_name):
    """处理失败状态"""
    error_msg = robot.skill_errors.get(skill_name, "Unknown error")
    return robot.form_feedback("failed", error_msg)


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
