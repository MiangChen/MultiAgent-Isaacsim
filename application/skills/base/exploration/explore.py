"""Explore Skill - Application layer

Flow: Plan waypoints -> Navigate to start -> Follow path -> Complete
"""

# Standard library imports
import json
from application import SkillManager


@SkillManager.register()
def explore(**kwargs):
    skill_manager = kwargs.get("skill_manager")
    skill_name = "explore"

    current_state = skill_manager.get_skill_state(skill_name)

    if current_state in [None, "INITIALIZING"]:
        return _init_explore(skill_manager, skill_name, kwargs)
    elif current_state == "NAVIGATING_TO_START":
        return _handle_navigating_to_start(skill_manager, skill_name)
    elif current_state == "EXECUTING":
        return _handle_executing(skill_manager, skill_name)
    elif current_state == "COMPLETED":
        return _handle_completed(skill_manager, skill_name)
    elif current_state == "FAILED":
        return _handle_failed(skill_manager, skill_name)


def _init_explore(skill_manager, skill_name, kwargs):
    skill_ros = skill_manager.skill_ros_interface

    # Parse parameters
    boundary = kwargs.get("boundary")
    holes = kwargs.get("holes", [])
    interpolation_distance = kwargs.get("interpolation_distance", 0.05)
    interpolation_method = kwargs.get("interpolation_method", "linear")
    lane_width = kwargs.get("lane_width", 2.0)
    robot_radius = kwargs.get("robot_radius", 0.5)

    # Convert string to list
    if isinstance(boundary, str):
        try:
            boundary = json.loads(boundary)
        except Exception as e:
            skill_manager.set_skill_state(skill_name, "FAILED")
            skill_manager.skill_errors[skill_name] = f"Failed to parse boundary: {e}"
            return skill_manager.form_feedback("failed", f"Invalid boundary: {e}")
    if isinstance(holes, str):
        try:
            holes = json.loads(holes)
        except:
            holes = []

    if not boundary or not isinstance(boundary, (list, tuple)):
        skill_manager.set_skill_state(skill_name, "FAILED")
        skill_manager.skill_errors[skill_name] = "Invalid boundary format"
        return skill_manager.form_feedback("failed", "Invalid boundary format")

    try:
        # Get current position from odom
        current_pos, _ = skill_ros.get_current_pose()
        current_z = current_pos[2] if len(current_pos) > 2 else 0.0

        # Plan exploration waypoints
        from application.skills.base.exploration.plan_exploration_waypoints import (
            plan_exploration_waypoints,
        )

        waypoints = plan_exploration_waypoints(
            polygon_coords=boundary,
            holes=holes,
            lane_width=lane_width,
            robot_radius=robot_radius,
            interpolation_distance=interpolation_distance,
            interpolation_method=interpolation_method,
            z_out=current_z,
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
        skill_manager.set_skill_data(skill_name, "waypoints", waypoints)
        skill_manager.set_skill_data(skill_name, "start_pos", start_pos)
        skill_manager.set_skill_data(skill_name, "start_quat", start_quat)

        skill_manager.set_skill_state(skill_name, "NAVIGATING_TO_START")
        return skill_manager.form_feedback("processing", "Waypoints planned", 20)

    except Exception as e:
        skill_manager.set_skill_state(skill_name, "FAILED")
        skill_manager.skill_errors[skill_name] = f"Init failed: {str(e)}"
        return skill_manager.form_feedback("failed", f"Init failed: {str(e)}")


def _handle_navigating_to_start(skill_manager, skill_name):
    # Start nav_2d if not started
    if not skill_manager.get_skill_data(skill_name, "navigate_started", False):
        start_pos = skill_manager.get_skill_data(skill_name, "start_pos")
        start_quat = skill_manager.get_skill_data(skill_name, "start_quat")

        # Reset nav_2d state before starting
        skill_manager.reset_skill("nav_2d")
        skill_manager.set_skill_data(skill_name, "navigate_started", True)
        skill_manager.set_skill_data(skill_name, "navigate_start_time", skill_manager.sim_time)

    # Keep calling nav_2d to update its state machine
    start_pos = skill_manager.get_skill_data(skill_name, "start_pos")
    start_quat = skill_manager.get_skill_data(skill_name, "start_quat")
    skill_manager.execute_skill("nav_2d", goal_pos=start_pos, goal_quat_wxyz=start_quat)

    # Check nav_2d status
    nav_state = skill_manager.get_skill_state("nav_2d")

    if nav_state == "COMPLETED":
        skill_manager.set_skill_state(skill_name, "EXECUTING")
        return skill_manager.form_feedback("processing", "Reached start point", 60)
    elif nav_state == "FAILED":
        error = skill_manager.skill_errors.get("nav_2d", "Unknown error")
        skill_manager.set_skill_state(skill_name, "FAILED")
        skill_manager.skill_errors[skill_name] = f"Navigate to start failed: {error}"
        return skill_manager.form_feedback("failed", f"Navigate failed: {error}")
    else:
        elapsed = skill_manager.sim_time - skill_manager.get_skill_data(
            skill_name, "navigate_start_time", skill_manager.sim_time
        )
        if elapsed > 60.0:
            skill_manager.set_skill_state(skill_name, "FAILED")
            skill_manager.skill_errors[skill_name] = "Navigate to start timeout"
            return skill_manager.form_feedback("failed", "Navigate timeout")

        return skill_manager.form_feedback("processing", "Navigating to start", 40)


def _handle_executing(skill_manager, skill_name):
    start_time = skill_manager.get_skill_data(skill_name, "exploration_start_time")

    if start_time is None:
        # Publish waypoints path
        waypoints = skill_manager.get_skill_data(skill_name, "waypoints")
        skill_ros = skill_manager.skill_ros_interface
        skill_ros.get_node_controller_mpc().move_event.clear()
        skill_ros.get_node_planner_ompl_2d().publisher_path.publish(waypoints)

        skill_manager.set_skill_data(skill_name, "exploration_start_time", skill_manager.sim_time)
        return skill_manager.form_feedback("processing", "Starting exploration", 65)

    elapsed = skill_manager.sim_time - start_time

    # Check completion
    skill_ros = skill_manager.skill_ros_interface
    if skill_ros.get_node_controller_mpc().move_event.is_set():
        skill_manager.set_skill_state(skill_name, "COMPLETED")
        return skill_manager.form_feedback("completed", "Exploration completed", 100)

    # Check timeout
    if elapsed > 120.0:
        skill_manager.set_skill_state(skill_name, "FAILED")
        skill_manager.skill_errors[skill_name] = "Exploration timeout"
        return skill_manager.form_feedback("failed", "Timeout")

    progress = min(65 + (elapsed / 120.0) * 30, 95)
    return skill_manager.form_feedback("processing", f"Exploring... ({elapsed:.1f}s)", int(progress))


def _handle_completed(skill_manager, skill_name):
    return skill_manager.form_feedback("completed", "Exploration completed", 100)


def _handle_failed(skill_manager, skill_name):
    error_msg = skill_manager.skill_errors.get(skill_name, "Unknown error")
    return skill_manager.form_feedback("failed", error_msg)
