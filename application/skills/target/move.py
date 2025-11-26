"""
Move Skill - Application layer (for Target robots)

Moves through a predefined path of waypoints in a loop.
This skill REQUIRES ROS.
"""

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose

from application import SkillManager
from log.log_manager import LogManager

logger = LogManager.get_logger(__name__)


@SkillManager.register()
def move(**kwargs):
    """
    Move through a predefined path

    Args:
        path: List of waypoints [(x, y, z), ...]
        cancel: Cancel movement if True

    Returns:
        Feedback dict with movement status
    """
    robot = kwargs.get("robot")
    skill_manager = kwargs.get("skill_manager")
    skill_name = "move"

    current_state = skill_manager.get_skill_state(skill_name)

    # Check for cancellation
    canceled = kwargs.get("cancel", False)
    if canceled:
        return _cancel_move(robot, skill_manager, skill_name)

    # State machine
    if current_state in [None, "IDLE"]:
        return _init_move(robot, skill_manager, skill_name, kwargs)
    elif current_state == "IDLE":
        return _handle_idle(robot, skill_manager, skill_name)
    elif current_state == "INITIALIZING":
        return _handle_initializing(robot, skill_manager, skill_name)
    elif current_state == "EXECUTING":
        return _handle_executing(robot, skill_manager, skill_name)
    elif current_state == "COMPLETED":
        return _handle_completed(robot, skill_manager, skill_name)
    elif current_state == "FAILED":
        return _handle_failed(skill_manager, skill_name)


def _init_move(robot, skill_manager, skill_name, kwargs):
    """Initialize move task"""
    # Check if ROS is available
    if not robot.has_ros():
        skill_manager.set_skill_state(skill_name, "FAILED")
        skill_manager.skill_errors[skill_name] = (
            "ROS not available - movement requires ROS"
        )
        return skill_manager.form_feedback("failed", "ROS not available")

    # Get path from kwargs
    path = kwargs.get("path", [])
    if not path:
        skill_manager.set_skill_state(skill_name, "FAILED")
        skill_manager.skill_errors[skill_name] = "Path is not provided"
        return skill_manager.form_feedback("failed", "Path is not provided")

    # Initialize movement data
    skill_manager.set_skill_data(skill_name, "path", path)
    skill_manager.set_skill_data(skill_name, "path_index", 0)

    skill_manager.set_skill_state(skill_name, "IDLE")
    logger.info("Target initialized and ready to move...")
    return skill_manager.form_feedback("processing", "Movement initialized", 5)


def _handle_idle(robot, skill_manager, skill_name):
    """Idle state: start navigation to next waypoint"""
    path = skill_manager.get_skill_data(skill_name, "path", [])
    path_index = skill_manager.get_skill_data(skill_name, "path_index", 0)

    # Loop through path
    path_index %= len(path)
    goal_pos = path[path_index]

    _start_navigation_to_waypoint(robot, skill_manager, skill_name, goal_pos)

    skill_manager.set_skill_data(skill_name, "path_index", path_index)

    return skill_manager.form_feedback(
        "processing", f"Target starting moving to {goal_pos}", 10
    )


def _start_navigation_to_waypoint(robot, skill_manager, skill_name, goal_pos):
    """Start navigation to specified waypoint"""
    mpc = robot.ros_manager.get_node_controller_mpc()
    mpc.has_reached_goal = False

    # Convert tuple to list
    if isinstance(goal_pos, tuple):
        goal_pos = list(goal_pos)

    goal_quat_wxyz = [1.0, 0.0, 0.0, 0.0]

    # Check if path planner server is available
    action_client = robot.ros_manager.get_action_client_path_planner()
    if not action_client.wait_for_server(timeout_sec=2.0):
        skill_manager.set_skill_state(skill_name, "FAILED")
        skill_manager.skill_errors[skill_name] = "Path planner server is not available"
        return

    mpc.move_event.clear()

    # Create path planning request
    goal_msg = ComputePathToPose.Goal()

    # Get current position as start
    start_pos_tensor, start_quat_tensor = robot.get_world_pose()
    start_pos = start_pos_tensor.cpu().numpy().tolist()
    start_quat = start_quat_tensor.cpu().numpy().tolist()

    goal_msg.start = _create_pose_stamped(skill_manager, start_pos, start_quat)
    goal_msg.goal = _create_pose_stamped(skill_manager, goal_pos, goal_quat_wxyz)

    # Send planning request
    move_nav_send_future = action_client.send_goal_async(goal_msg)
    move_nav_start_time = skill_manager.sim_time

    skill_manager.set_skill_data(
        skill_name, "move_nav_send_future", move_nav_send_future
    )
    skill_manager.set_skill_data(skill_name, "move_nav_start_time", move_nav_start_time)
    skill_manager.set_skill_state(skill_name, "INITIALIZING")


def _handle_initializing(robot, skill_manager, skill_name):
    """Handle initializing state: wait for path planning to complete"""
    move_nav_send_future = skill_manager.get_skill_data(
        skill_name, "move_nav_send_future"
    )
    move_nav_start_time = skill_manager.get_skill_data(
        skill_name, "move_nav_start_time"
    )
    move_nav_result_future = skill_manager.get_skill_data(
        skill_name, "move_nav_result_future"
    )

    # Check if send request is done
    if move_nav_result_future is None:
        if move_nav_send_future.done():
            goal_handle = move_nav_send_future.result()
            if goal_handle.accepted:
                move_nav_result_future = goal_handle.get_result_async()
                skill_manager.set_skill_data(
                    skill_name, "move_nav_result_future", move_nav_result_future
                )
                skill_manager.set_skill_data(
                    skill_name, "move_nav_goal_handle", goal_handle
                )
                return skill_manager.form_feedback("processing", "Planning path...", 15)
            else:
                skill_manager.set_skill_state(skill_name, "FAILED")
                skill_manager.skill_errors[skill_name] = "Request rejected"
                return skill_manager.form_feedback("failed", "Request rejected")
        else:
            elapsed = skill_manager.sim_time - move_nav_start_time
            if elapsed > 3.0:
                skill_manager.set_skill_state(skill_name, "FAILED")
                skill_manager.skill_errors[skill_name] = "No response from planner"
                return skill_manager.form_feedback("failed", "No response from planner")
            return skill_manager.form_feedback("processing", "Sending request...", 5)

    # Check planning result
    if move_nav_result_future.done():
        result = move_nav_result_future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED and result.result.path.poses:
            robot.ros_manager.get_node_controller_mpc().move_event.clear()
            skill_manager.set_skill_data(
                skill_name, "move_nav_start_time", skill_manager.sim_time
            )
            skill_manager.set_skill_state(skill_name, "EXECUTING")
        else:
            skill_manager.set_skill_state(skill_name, "FAILED")
            skill_manager.skill_errors[skill_name] = "Planning failed"
    else:
        elapsed = skill_manager.sim_time - move_nav_start_time
        if elapsed > 15.0:
            move_nav_goal_handle = skill_manager.get_skill_data(
                skill_name, "move_nav_goal_handle"
            )
            if move_nav_goal_handle:
                move_nav_goal_handle.cancel_goal_async()
            skill_manager.set_skill_state(skill_name, "FAILED")
            skill_manager.skill_errors[skill_name] = "Planning timeout"

    return skill_manager.form_feedback("processing", "Planning path...", 30)


def _handle_executing(robot, skill_manager, skill_name):
    """Handle executing state: robot is moving to target"""
    if robot.ros_manager.get_node_controller_mpc().move_event.is_set():
        skill_manager.set_skill_state(skill_name, "COMPLETED")
        return skill_manager.form_feedback("processing", "Executing...", 50)
    else:
        move_nav_start_time = skill_manager.get_skill_data(
            skill_name, "move_nav_start_time", skill_manager.sim_time
        )
        path_index = skill_manager.get_skill_data(skill_name, "path_index", 0)

        elapsed = skill_manager.sim_time - move_nav_start_time
        if elapsed > 120.0:
            skill_manager.set_skill_state(skill_name, "FAILED")
            skill_manager.skill_errors[skill_name] = "Navigation timeout"
        else:
            progress = min(50 + (elapsed / 120.0) * 45, 95)
            waypoint_info = f"to waypoint {path_index}"
            return skill_manager.form_feedback(
                "processing",
                f"Moving {waypoint_info}... ({elapsed:.1f}s)",
                int(progress),
            )

    return skill_manager.form_feedback("processing", "Executing...", 50)


def _handle_completed(robot, skill_manager, skill_name):
    """Handle completed state: reached waypoint, prepare for next"""
    path_index = skill_manager.get_skill_data(skill_name, "path_index", 0)
    logger.info(f"Reached waypoint {path_index}")

    # Update waypoint index
    skill_manager.set_skill_data(skill_name, "path_index", path_index + 1)
    _cleanup_move_navigation(skill_manager, skill_name)
    skill_manager.set_skill_state(skill_name, "IDLE")  # Back to IDLE for next waypoint
    return skill_manager.form_feedback(
        "processing", f"Reached waypoint, checking for next...", 50
    )


def _handle_failed(skill_manager, skill_name):
    """Handle failed state"""
    error_msg = skill_manager.skill_errors.get(skill_name, "Unknown error")
    _cleanup_move_navigation(skill_manager, skill_name)
    return skill_manager.form_feedback("failed", error_msg)


def _cancel_move(robot, skill_manager, skill_name):
    """Cancel movement"""
    logger.info("Cancel Moving...")

    # Cancel ongoing navigation if any
    move_nav_goal_handle = skill_manager.get_skill_data(
        skill_name, "move_nav_goal_handle"
    )
    if move_nav_goal_handle:
        move_nav_goal_handle.cancel_goal_async()

    # Cleanup navigation data
    _cleanup_move_navigation(skill_manager, skill_name)

    # Clear skill state
    skill_manager.set_skill_state(skill_name, None)
    skill_manager.skill_errors[skill_name] = None

    return skill_manager.form_feedback("completed", "Target stop moving.")


def _cleanup_move_navigation(skill_manager, skill_name):
    """Cleanup current navigation temporary data"""
    nav_keys_to_remove = [
        "move_nav_start_time",
        "move_nav_send_future",
        "move_nav_result_future",
        "move_nav_goal_handle",
    ]
    for key in nav_keys_to_remove:
        skill_manager.set_skill_data(skill_name, key, None)


def _create_pose_stamped(skill_manager, pos: list, quat_wxyz: list):
    """Create PoseStamped message"""
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
