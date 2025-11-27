"""
Nav 3D Skill - 3D Path Planning for Aerial Robots (Drones)

Architecture:
    1. Path Planning: ROS action with 3D grid map (via service)
    2. Velocity Computation: MPC controller (application layer)
    3. Execution: Control objects (simulation layer)

Flow:
    nav_3d -> Request map via service -> 3D path planning -> MPC computes velocity ->
    RobotControl object -> robot.apply_control() -> Isaac Sim
"""

# Standard library imports
import json
from application import SkillManager

# ROS2 message imports
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose


@SkillManager.register()
def nav_3d(**kwargs):
    """
    Navigate to target position using 3D path planning.
    Suitable for aerial robots (drones) that can move in full 3D space.
    """
    skill_manager = kwargs.get("skill_manager")
    skill_name = "nav_3d"

    current_state = skill_manager.get_skill_state(skill_name)

    if current_state in [None, "INITIALIZING"]:
        _init_nav_3d(skill_manager, skill_name, kwargs)

    current_state = skill_manager.get_skill_state(skill_name)

    if current_state == "EXECUTING":
        return _handle_executing(skill_manager, skill_name)
    elif current_state == "COMPLETED":
        return _handle_completed(skill_manager, skill_name)
    elif current_state == "FAILED":
        return _handle_failed(skill_manager, skill_name)


def _init_nav_3d(skill_manager, skill_name, kwargs):
    """Initialize 3D navigation"""

    skill_ros = skill_manager.skill_ros_interface
    skill_ros.get_node_controller_mpc().has_reached_goal = False
    skill_manager.set_skill_state(skill_name, "EXECUTING")

    # Parse parameters
    start_pos = kwargs.get("start_pos", None)
    start_quat = kwargs.get("start_quat")
    goal_pos = kwargs.get("goal_pos")
    goal_quat_wxyz = kwargs.get("goal_quat_wxyz", [1.0, 0.0, 0.0, 0.0])

    if isinstance(start_pos, str):
        start_pos = json.loads(start_pos)
    if isinstance(start_quat, str):
        start_quat = json.loads(start_quat)
    if isinstance(goal_pos, str):
        goal_pos = json.loads(goal_pos)
    elif isinstance(goal_pos, tuple):
        goal_pos = list(goal_pos)
    if isinstance(goal_quat_wxyz, str):
        goal_quat_wxyz = json.loads(goal_quat_wxyz)
    elif isinstance(goal_quat_wxyz, tuple):
        goal_quat_wxyz = list(goal_quat_wxyz)

    # Check ROS action server
    action_client = skill_ros.get_action_client_path_planner_3d()
    if not action_client.wait_for_server(timeout_sec=2.0):
        skill_manager.set_skill_state(skill_name, "FAILED")
        skill_manager.skill_errors[skill_name] = "3D Path planner server not available"
        return

    skill_ros.get_node_controller_mpc().move_event.clear()

    # Create goal message
    goal_msg = ComputePathToPose.Goal()

    # Get current pose from odom topic (not from robot instance)
    if start_pos is None or start_quat is None:
        current_pos, current_quat = skill_ros.get_current_pose()
        if start_pos is None:
            start_pos = current_pos
        if start_quat is None:
            start_quat = current_quat

    goal_msg.start = _create_pose_stamped(skill_manager, start_pos, start_quat)
    goal_msg.goal = _create_pose_stamped(skill_manager, goal_pos, goal_quat_wxyz)

    # Store skill data
    skill_manager.set_skill_data(skill_name, "goal_pos", goal_pos)
    skill_manager.set_skill_data(skill_name, "goal_quat_wxyz", goal_quat_wxyz)
    skill_manager.set_skill_data(skill_name, "start_pos", start_pos)
    skill_manager.set_skill_data(skill_name, "start_quat", start_quat)

    # Send planning request (ROS action)
    send_future = action_client.send_goal_async(goal_msg)
    start_time = skill_manager.sim_time

    skill_manager.set_skill_data(skill_name, "send_future", send_future)
    skill_manager.set_skill_data(skill_name, "start_time", start_time)

    # Check planning result
    result_future = skill_manager.get_skill_data(skill_name, "result_future")

    if result_future is None:
        if send_future.done():
            goal_handle = send_future.result()
            if goal_handle.accepted:
                result_future = goal_handle.get_result_async()
                skill_manager.set_skill_data(skill_name, "result_future", result_future)
                skill_manager.set_skill_data(skill_name, "goal_handle", goal_handle)
                return skill_manager.form_feedback("processing", "Planning 3D path...", 15)
            else:
                skill_manager.set_skill_state(skill_name, "FAILED")
                skill_manager.skill_errors[skill_name] = "Request rejected"
                return skill_manager.form_feedback("failed", "Request rejected")
        else:
            elapsed = skill_manager.sim_time - start_time
            if elapsed > 3.0:
                skill_manager.set_skill_state(skill_name, "FAILED")
                skill_manager.skill_errors[skill_name] = "No response from planner"
                return skill_manager.form_feedback("failed", "No response")
            return skill_manager.form_feedback("processing", "Sending request...", 5)

    # Check planning result
    if result_future.done():
        result = result_future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED and result.result.path.poses:
            skill_ros.get_node_controller_mpc().move_event.clear()
            skill_manager.set_skill_data(skill_name, "move_start_time", skill_manager.sim_time)
            skill_manager.set_skill_state(skill_name, "EXECUTING")
        else:
            skill_manager.set_skill_state(skill_name, "FAILED")
            skill_manager.skill_errors[skill_name] = "3D Planning failed"
    else:
        elapsed = skill_manager.sim_time - start_time
        if elapsed > 15.0:
            goal_handle = skill_manager.get_skill_data(skill_name, "goal_handle")
            if goal_handle:
                goal_handle.cancel_goal_async()
            skill_manager.set_skill_state(skill_name, "FAILED")
            skill_manager.skill_errors[skill_name] = "Planning timeout"

    return skill_manager.form_feedback("processing", "Planning 3D path...", 30)


def _handle_executing(skill_manager, skill_name):
    """Handle executing state"""
    skill_ros = skill_manager.skill_ros_interface
    if skill_ros.get_node_controller_mpc().move_event.is_set():
        skill_manager.set_skill_state(skill_name, "COMPLETED")
        return skill_manager.form_feedback("completed", "Reached goal", 100)

    move_start_time = skill_manager.get_skill_data(skill_name, "move_start_time", 0)
    elapsed = skill_manager.sim_time - move_start_time
    if elapsed > 120.0:
        skill_manager.set_skill_state(skill_name, "FAILED")
        skill_manager.skill_errors[skill_name] = "Navigate timeout"
        return skill_manager.form_feedback("failed", "Timeout")

    progress = min(50 + (elapsed / 120.0) * 45, 95)
    return skill_manager.form_feedback(
        "processing", f"Moving in 3D... ({elapsed:.1f}s)", int(progress)
    )


def _handle_completed(skill_manager, skill_name):
    """Handle completed state"""
    return skill_manager.form_feedback("completed", "3D Navigate completed", 100)


def _handle_failed(skill_manager, skill_name):
    """Handle failed state"""
    error_msg = skill_manager.skill_errors.get(skill_name, "Unknown error")
    return skill_manager.form_feedback("failed", error_msg)


def _create_pose_stamped(skill_manager, pos: list, quat_wxyz: list):
    """Create ROS PoseStamped message"""
    pose_stamped = PoseStamped()

    sim_time_sec = int(skill_manager.sim_time)
    sim_time_nanosec = int((skill_manager.sim_time - sim_time_sec) * 1e9)
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
