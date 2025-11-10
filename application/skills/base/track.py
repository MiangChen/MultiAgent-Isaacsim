"""
Track Skill - Application layer

Tracks a moving target by subscribing to its odometry and following waypoints.
This skill REQUIRES ROS.
"""

import functools
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import Odometry

from application import SkillManager
from log.log_manager import LogManager

logger = LogManager.get_logger(__name__)


def track_callback(robot, skill_manager, msg):
    """Callback for target odometry subscription"""
    pos = (
        msg.pose.pose.position.x,
        msg.pose.pose.position.y,
        msg.pose.pose.position.z,
    )
    skill_name = "track"
    track_counter = skill_manager.get_skill_data(skill_name, "track_counter", 0)
    track_period = skill_manager.get_skill_data(skill_name, "track_period", 10)
    track_waypoint_list = skill_manager.get_skill_data(
        skill_name, "track_waypoint_list", []
    )

    track_counter += 1
    skill_manager.set_skill_data(skill_name, "track_counter", track_counter)

    if track_counter % track_period == 0:
        track_waypoint_list.append(pos)
        skill_manager.set_skill_data(
            skill_name, "track_waypoint_list", track_waypoint_list
        )


@SkillManager.register()
def track(**kwargs):
    """
    Track a moving target

    Args:
        track_period: Period for sampling waypoints (default: 10)
        target_topic: Topic to subscribe for target odometry (default: "/target_0/odom")
        cancel: Cancel tracking if True

    Returns:
        Feedback dict with tracking status
    """
    robot = kwargs.get("robot")
    skill_manager = kwargs.get("skill_manager")
    skill_name = "track"

    current_state = skill_manager.get_skill_state(skill_name)

    # Check for cancellation
    canceled = kwargs.get("cancel", False)
    if canceled:
        return _cancel_track(robot, skill_manager, skill_name)

    # State machine
    if current_state in [None, "IDLE"]:
        return _init_track(robot, skill_manager, skill_name, kwargs)
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


def _init_track(robot, skill_manager, skill_name, kwargs):
    """Initialize tracking task"""
    # Check if ROS is available
    if not robot.has_ros():
        skill_manager.set_skill_state(skill_name, "FAILED")
        skill_manager.skill_errors[skill_name] = (
            "ROS not available - tracking requires ROS"
        )
        return skill_manager.form_feedback("failed", "ROS not available")

    # Initialize tracking data
    skill_manager.set_skill_data(skill_name, "track_waypoint_list", [])
    skill_manager.set_skill_data(skill_name, "track_current_waypoint_index", 0)
    skill_manager.set_skill_data(skill_name, "track_counter", 0)
    skill_manager.set_skill_data(
        skill_name, "track_period", kwargs.get("track_period", 10)
    )

    # Create subscription to target odometry
    target_topic = kwargs.get("target_topic", "/target_0/odom")
    track_callback_partial = functools.partial(
        track_callback, robot=robot, skill_manager=skill_manager
    )

    node = robot.ros_manager.get_node()
    track_waypoint_sub = node.create_subscription(
        Odometry,
        target_topic,
        track_callback_partial,
        50,
    )
    skill_manager.set_skill_data(skill_name, "track_waypoint_sub", track_waypoint_sub)

    skill_manager.set_skill_state(skill_name, "IDLE")
    logger.info("Track skill initialized, waiting for waypoints...")
    return skill_manager.form_feedback("processing", "Tracking initialized", 5)


def _handle_idle(robot, skill_manager, skill_name):
    """Idle state: wait for new waypoints and start navigation"""
    track_waypoint_list = skill_manager.get_skill_data(
        skill_name, "track_waypoint_list", []
    )
    track_current_waypoint_index = skill_manager.get_skill_data(
        skill_name, "track_current_waypoint_index", 0
    )

    # Check if there are new waypoints to navigate to
    if len(track_waypoint_list) > track_current_waypoint_index:
        # Start navigation to next waypoint
        goal_pos = track_waypoint_list[track_current_waypoint_index]
        _start_navigation_to_waypoint(robot, skill_manager, skill_name, goal_pos)
        return skill_manager.form_feedback(
            "processing",
            f"Starting navigation to waypoint {track_current_waypoint_index}",
            10,
        )
    else:
        # No new waypoints, continue waiting
        return skill_manager.form_feedback(
            "processing", "Waiting for target waypoints...", 5
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
    track_current_waypoint_index = skill_manager.get_skill_data(
        skill_name, "track_current_waypoint_index", 0
    )
    logger.info(
        f"Tracking: Sending path request to waypoint {track_current_waypoint_index}: {goal_pos}"
    )

    # Create path planning request
    goal_msg = ComputePathToPose.Goal()

    # Get current position as start
    start_pos_tensor, start_quat_tensor = robot.get_world_pose()
    start_pos = start_pos_tensor.cpu().numpy().tolist()
    start_quat = start_quat_tensor.cpu().numpy().tolist()

    goal_msg.start = _create_pose_stamped(robot, start_pos, start_quat)
    goal_msg.goal = _create_pose_stamped(robot, goal_pos, goal_quat_wxyz)

    # Send planning request
    track_nav_send_future = action_client.send_goal_async(goal_msg)
    track_nav_start_time = robot.sim_time

    skill_manager.set_skill_data(
        skill_name, "track_nav_send_future", track_nav_send_future
    )
    skill_manager.set_skill_data(
        skill_name, "track_nav_start_time", track_nav_start_time
    )
    skill_manager.set_skill_state(skill_name, "INITIALIZING")


def _handle_initializing(robot, skill_manager, skill_name):
    """Handle initializing state: wait for path planning to complete"""
    track_nav_send_future = skill_manager.get_skill_data(
        skill_name, "track_nav_send_future"
    )
    track_nav_start_time = skill_manager.get_skill_data(
        skill_name, "track_nav_start_time"
    )
    track_nav_result_future = skill_manager.get_skill_data(
        skill_name, "track_nav_result_future"
    )

    # Check if send request is done
    if track_nav_result_future is None:
        if track_nav_send_future.done():
            goal_handle = track_nav_send_future.result()
            if goal_handle.accepted:
                track_nav_result_future = goal_handle.get_result_async()
                skill_manager.set_skill_data(
                    skill_name, "track_nav_result_future", track_nav_result_future
                )
                skill_manager.set_skill_data(
                    skill_name, "track_nav_goal_handle", goal_handle
                )
                return skill_manager.form_feedback("processing", "Planning path...", 15)
            else:
                skill_manager.set_skill_state(skill_name, "FAILED")
                skill_manager.skill_errors[skill_name] = "Request rejected"
                return skill_manager.form_feedback("failed", "Request rejected")
        else:
            elapsed = robot.sim_time - track_nav_start_time
            if elapsed > 3.0:
                skill_manager.set_skill_state(skill_name, "FAILED")
                skill_manager.skill_errors[skill_name] = "No response from planner"
                return skill_manager.form_feedback("failed", "No response from planner")
            return skill_manager.form_feedback("processing", "Sending request...", 5)

    # Check planning result
    if track_nav_result_future.done():
        result = track_nav_result_future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED and result.result.path.poses:
            robot.ros_manager.get_node_controller_mpc().move_event.clear()
            skill_manager.set_skill_data(
                skill_name, "track_nav_move_start_time", robot.sim_time
            )
            skill_manager.set_skill_state(skill_name, "EXECUTING")
        else:
            skill_manager.set_skill_state(skill_name, "FAILED")
            skill_manager.skill_errors[skill_name] = "Planning failed"
    else:
        elapsed = robot.sim_time - track_nav_start_time
        if elapsed > 15.0:
            track_nav_goal_handle = skill_manager.get_skill_data(
                skill_name, "track_nav_goal_handle"
            )
            if track_nav_goal_handle:
                track_nav_goal_handle.cancel_goal_async()
            skill_manager.set_skill_state(skill_name, "FAILED")
            skill_manager.skill_errors[skill_name] = "Planning timeout"

    return skill_manager.form_feedback("processing", "Planning path...", 30)


def _handle_executing(robot, skill_manager, skill_name):
    """Handle executing state: robot is moving to target"""
    if robot.ros_manager.get_node_controller_mpc().move_event.is_set():
        skill_manager.set_skill_state(skill_name, "COMPLETED")
        return skill_manager.form_feedback("processing", "Executing...", 50)
    else:
        track_nav_move_start_time = skill_manager.get_skill_data(
            skill_name, "track_nav_move_start_time", robot.sim_time
        )
        track_current_waypoint_index = skill_manager.get_skill_data(
            skill_name, "track_current_waypoint_index", 0
        )

        elapsed = robot.sim_time - track_nav_move_start_time
        if elapsed > 120.0:
            skill_manager.set_skill_state(skill_name, "FAILED")
            skill_manager.skill_errors[skill_name] = "Navigation timeout"
        else:
            progress = min(50 + (elapsed / 120.0) * 45, 95)
            waypoint_info = f"to waypoint {track_current_waypoint_index}"
            return skill_manager.form_feedback(
                "processing",
                f"Moving {waypoint_info}... ({elapsed:.1f}s)",
                int(progress),
            )

    return skill_manager.form_feedback("processing", "Executing...", 50)


def _handle_completed(robot, skill_manager, skill_name):
    """Handle completed state: reached waypoint, prepare for next"""
    track_current_waypoint_index = skill_manager.get_skill_data(
        skill_name, "track_current_waypoint_index", 0
    )
    logger.info(f"Reached waypoint {track_current_waypoint_index}")

    # Update waypoint index
    skill_manager.set_skill_data(
        skill_name, "track_current_waypoint_index", track_current_waypoint_index + 1
    )
    _cleanup_track_navigation(skill_manager, skill_name)
    skill_manager.set_skill_state(
        skill_name, "IDLE"
    )  # Back to IDLE to check for next waypoint
    return skill_manager.form_feedback(
        "processing", f"Reached waypoint, checking for next...", 50
    )


def _handle_failed(skill_manager, skill_name):
    """Handle failed state"""
    error_msg = skill_manager.skill_errors.get(skill_name, "Unknown error")
    _cleanup_track_navigation(skill_manager, skill_name)
    return skill_manager.form_feedback("failed", error_msg)


def _cancel_track(robot, skill_manager, skill_name):
    """Cancel tracking task"""
    logger.info("Canceling track skill...")

    # Cancel ongoing navigation if any
    track_nav_goal_handle = skill_manager.get_skill_data(
        skill_name, "track_nav_goal_handle"
    )
    if track_nav_goal_handle:
        track_nav_goal_handle.cancel_goal_async()

    # Cleanup navigation data
    _cleanup_track_navigation(skill_manager, skill_name)

    # Stop tracking subscription
    if robot.has_ros():
        track_waypoint_sub = skill_manager.get_skill_data(
            skill_name, "track_waypoint_sub"
        )
        if track_waypoint_sub is not None:
            robot.ros_manager.get_node().destroy_subscription(track_waypoint_sub)

    # Clear skill state
    skill_manager.set_skill_state(skill_name, None)
    skill_manager.skill_errors[skill_name] = None

    return skill_manager.form_feedback("completed", "Tracking canceled")


def _cleanup_track_navigation(skill_manager, skill_name):
    """Cleanup current navigation temporary data"""
    nav_keys_to_remove = [
        "track_nav_start_time",
        "track_nav_send_future",
        "track_nav_result_future",
        "track_nav_goal_handle",
        "track_nav_move_start_time",
    ]
    for key in nav_keys_to_remove:
        skill_manager.set_skill_data(skill_name, key, None)


def _create_pose_stamped(robot, pos: list, quat_wxyz: list):
    """Create PoseStamped message"""
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
