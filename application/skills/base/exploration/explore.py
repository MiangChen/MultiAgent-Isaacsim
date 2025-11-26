"""Explore Skill - Application layer

Flow: Plan waypoints -> Navigate to start -> Follow path -> Complete
"""

# Standard library imports
import json
from application import SkillManager

# ROS2 message imports
from action_msgs.msg import GoalStatus


@SkillManager.register()
def explore(**kwargs):
    robot = kwargs.get("robot")
    skill_manager = kwargs.get("skill_manager")
    skill_name = "explore"

    current_state = skill_manager.get_skill_state(skill_name)

    if current_state in [None, "INITIALIZING"]:
        return _init_explore(robot, skill_manager, skill_name, kwargs)
    elif current_state == "NAVIGATING_TO_START":
        return _handle_navigating_to_start(robot, skill_manager, skill_name)
    elif current_state == "EXECUTING":
        return _handle_executing(robot, skill_manager, skill_name)
    elif current_state == "COMPLETED":
        return _handle_completed(robot, skill_manager, skill_name)
    elif current_state == "FAILED":
        return _handle_failed(robot, skill_manager, skill_name)


def _init_explore(robot, skill_manager, skill_name, kwargs):
    # Check if ROS is available
    if not robot.has_ros():
        skill_manager.set_skill_state(skill_name, "FAILED")
        skill_manager.skill_errors[skill_name] = (
            "ROS not available - exploration requires ROS"
        )
        return skill_manager.form_feedback("failed", "ROS not available")

    # Parse parameters
    boundary = kwargs.get("boundary")
    holes = kwargs.get("holes", [])
    target_prim = kwargs.get("target_prim", "/TARGET_PRIM_NOT_SPECIFIED")
    interpolation_distance = kwargs.get("interpolation_distance", 0.05)
    interpolation_method = kwargs.get("interpolation_method", "linear")

    # Convert string to list
    if isinstance(boundary, str):
        try:
            boundary = json.loads(boundary)
        except:
            pass
    if isinstance(holes, str):
        try:
            holes = json.loads(holes)
        except:
            holes = []

    # Validate
    if not boundary or not isinstance(boundary, (list, tuple)):
        skill_manager.set_skill_state(skill_name, "FAILED")
        skill_manager.skill_errors[skill_name] = "Invalid boundary"
        return skill_manager.form_feedback("failed", "Invalid boundary")

    try:
        # Get current robot z position
        current_position, _ = robot.get_world_pose()
        current_z = current_position[2] if len(current_position) > 2 else 0.0

        # Plan exploration waypoints
        from application.skills.base.exploration.plan_exploration_waypoints import (
            plan_exploration_waypoints,
        )

        waypoints = plan_exploration_waypoints(
            robot=robot,
            polygon_coords=boundary,
            holes=holes,
            lane_width=robot.get_detection_radius(),
            robot_radius=robot.get_robot_radius(),
            interpolation_distance=interpolation_distance,
            interpolation_method=interpolation_method,
            z_out=current_z,  # Use current robot z height
        )

        if not waypoints or not waypoints.poses:
            skill_manager.set_skill_state(skill_name, "FAILED")
            skill_manager.skill_errors[skill_name] = "Failed to plan waypoints"
            return skill_manager.form_feedback("failed", "Failed to plan waypoints")

        # Get start point
        first_waypoint = waypoints.poses[0]
        start_pos = [
            first_waypoint.pose.position.x,
            first_waypoint.pose.position.y,
            first_waypoint.pose.position.z,
        ]
        start_quat = [
            first_waypoint.pose.orientation.w,
            first_waypoint.pose.orientation.x,
            first_waypoint.pose.orientation.y,
            first_waypoint.pose.orientation.z,
        ]

        # Store data
        skill_manager.set_skill_data(skill_name, "boundary", boundary)
        skill_manager.set_skill_data(skill_name, "holes", holes)
        skill_manager.set_skill_data(skill_name, "target_prim", target_prim)
        skill_manager.set_skill_data(skill_name, "waypoints", waypoints)
        skill_manager.set_skill_data(skill_name, "start_pos", start_pos)
        skill_manager.set_skill_data(skill_name, "start_quat", start_quat)

        skill_manager.set_skill_state(skill_name, "NAVIGATING_TO_START")
        return skill_manager.form_feedback("processing", "Waypoints planned", 20)

    except Exception as e:
        skill_manager.set_skill_state(skill_name, "FAILED")
        skill_manager.skill_errors[skill_name] = f"Init failed: {str(e)}"
        return skill_manager.form_feedback("failed", f"Init failed: {str(e)}")


def _handle_navigating_to_start(robot, skill_manager, skill_name):
    # Check if navigate_to is available
    if skill_manager.get_skill_state("navigate_to") not in [
        None,
        "COMPLETED",
        "FAILED",
    ]:
        return skill_manager.form_feedback("processing", "Waiting for navigate_to", 25)

    # Start navigate_to if not started
    if not skill_manager.get_skill_data(skill_name, "navigate_started", False):
        start_pos = skill_manager.get_skill_data(skill_name, "start_pos")
        start_quat = skill_manager.get_skill_data(skill_name, "start_quat")

        # Execute navigate_to skill
        from application.skills.base.navigation import navigate_to

        skill_manager.register_skill("navigate_to", navigate_to)
        skill_manager.execute_skill(
            "navigate_to", goal_pos=start_pos, goal_quat_wxyz=start_quat
        )

        skill_manager.set_skill_data(skill_name, "navigate_started", True)
        skill_manager.set_skill_data(skill_name, "navigate_start_time", robot.sim_time)
        return skill_manager.form_feedback("processing", "Navigating to start", 30)

    # Check navigate_to status
    nav_state = skill_manager.get_skill_state("navigate_to")

    if nav_state == "COMPLETED":
        skill_manager.set_skill_state(skill_name, "EXECUTING")
        return skill_manager.form_feedback("processing", "Reached start point", 60)
    elif nav_state == "FAILED":
        error = skill_manager.skill_errors.get("navigate_to", "Unknown error")
        skill_manager.set_skill_state(skill_name, "FAILED")
        skill_manager.skill_errors[skill_name] = f"Navigate to start failed: {error}"
        return skill_manager.form_feedback("failed", f"Navigate failed: {error}")
    else:
        # Check timeout
        elapsed = robot.sim_time - skill_manager.get_skill_data(
            skill_name, "navigate_start_time", robot.sim_time
        )
        if elapsed > 60.0:
            skill_manager.set_skill_state(skill_name, "FAILED")
            skill_manager.skill_errors[skill_name] = "Navigate to start timeout"
            return skill_manager.form_feedback("failed", "Navigate timeout")

        return skill_manager.form_feedback("processing", "Navigating to start", 40)


def _handle_executing(robot, skill_manager, skill_name):
    start_time = skill_manager.get_skill_data(skill_name, "exploration_start_time")

    if start_time is None:
        # Set detection mode
        if hasattr(robot, "is_detecting"):
            robot.is_detecting = True
        if hasattr(robot, "target_prim"):
            target_prim = skill_manager.get_skill_data(skill_name, "target_prim")
            robot.target_prim = target_prim

        # Publish waypoints path
        waypoints = skill_manager.get_skill_data(skill_name, "waypoints")
        robot.ros_manager.get_node_controller_mpc().move_event.clear()
        robot.ros_manager.get_node_planner_ompl().publisher_path.publish(waypoints)

        skill_manager.set_skill_data(
            skill_name, "exploration_start_time", robot.sim_time
        )
        return skill_manager.form_feedback("processing", "Starting exploration", 65)

    elapsed = robot.sim_time - start_time

    # Check completion
    if robot.ros_manager.get_node_controller_mpc().move_event.is_set():
        _cleanup_explore(robot, skill_manager, skill_name)
        skill_manager.set_skill_state(skill_name, "COMPLETED")
        return skill_manager.form_feedback("completed", "Exploration completed", 100)

    # Check timeout
    if elapsed > 120.0:
        _cleanup_explore(robot, skill_manager, skill_name)
        skill_manager.set_skill_state(skill_name, "FAILED")
        skill_manager.skill_errors[skill_name] = "Exploration timeout"
        return skill_manager.form_feedback("failed", "Timeout")

    progress = min(65 + (elapsed / 120.0) * 30, 95)
    return skill_manager.form_feedback(
        "processing", f"Exploring... ({elapsed:.1f}s)", int(progress)
    )


def _handle_completed(robot, skill_manager, skill_name):
    return skill_manager.form_feedback("completed", "Exploration completed", 100)


def _handle_failed(robot, skill_manager, skill_name):
    error_msg = skill_manager.skill_errors.get(skill_name, "Unknown error")
    return skill_manager.form_feedback("failed", error_msg)


def _cleanup_explore(robot, skill_manager, skill_name):
    if hasattr(robot, "is_detecting"):
        robot.is_detecting = False
    if hasattr(robot, "target_prim"):
        robot.target_prim = None
