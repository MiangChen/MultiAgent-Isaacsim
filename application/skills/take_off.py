"""Take Off Skill - Application layer

Flow: Get current position -> Navigate to altitude -> Complete
"""

from application.skill_registry import SkillRegistry


@SkillRegistry.register()
def take_off(**kwargs):
    robot = kwargs.get("robot")
    skill_manager = kwargs.get("skill_manager")
    skill_name = "take_off"
    
    current_state = skill_manager.get_skill_state(skill_name)
    
    if current_state in [None, "INITIALIZING"]:
        return _init_take_off(robot, skill_manager, skill_name, kwargs)
    elif current_state == "NAVIGATING_TO_ALTITUDE":
        return _handle_navigating_to_altitude(robot, skill_manager, skill_name)
    elif current_state == "COMPLETED":
        return _handle_completed(robot, skill_manager, skill_name)
    elif current_state == "FAILED":
        return _handle_failed(robot, skill_manager, skill_name)


def _init_take_off(robot, skill_manager, skill_name, kwargs):
    # Get altitude parameter
    altitude = kwargs.get("altitude")
    if altitude is None:
        skill_manager.set_skill_state(skill_name, "FAILED")
        skill_manager.skill_errors[skill_name] = "Altitude parameter required"
        return skill_manager.form_feedback("failed", "Altitude required")
    
    # Validate altitude
    try:
        altitude = float(altitude)
        if altitude <= 0:
            skill_manager.set_skill_state(skill_name, "FAILED")
            skill_manager.skill_errors[skill_name] = "Altitude must be > 0"
            return skill_manager.form_feedback("failed", "Invalid altitude")
    except (ValueError, TypeError):
        skill_manager.set_skill_state(skill_name, "FAILED")
        skill_manager.skill_errors[skill_name] = "Invalid altitude parameter"
        return skill_manager.form_feedback("failed", "Invalid altitude")
    
    # Check robot body
    if not hasattr(robot, "body") or robot.body is None:
        skill_manager.set_skill_state(skill_name, "FAILED")
        skill_manager.skill_errors[skill_name] = "Robot body not available"
        return skill_manager.form_feedback("failed", "Robot body not available")
    
    try:
        # Get current position and orientation
        current_pos, current_quat = robot.body.get_world_pose()
        current_pos = current_pos.cpu().numpy().tolist()
        current_quat = current_quat.cpu().numpy().tolist()
        
        # Target position: keep XY, change Z to target altitude
        take_off_goal = [
            current_pos[0],  # Keep current X
            current_pos[1],  # Keep current Y
            float(altitude), # Target altitude
        ]
        
        # Store data
        skill_manager.set_skill_data(skill_name, "target_altitude", altitude)
        skill_manager.set_skill_data(skill_name, "take_off_goal", take_off_goal)
        skill_manager.set_skill_data(skill_name, "goal_quat", current_quat)
        
        skill_manager.set_skill_state(skill_name, "NAVIGATING_TO_ALTITUDE")
        return skill_manager.form_feedback("processing", f"Target altitude: {altitude}m", 20)
        
    except Exception as e:
        skill_manager.set_skill_state(skill_name, "FAILED")
        skill_manager.skill_errors[skill_name] = f"Init failed: {str(e)}"
        return skill_manager.form_feedback("failed", f"Init failed: {str(e)}")


def _handle_navigating_to_altitude(robot, skill_manager, skill_name):
    # Check if navigate_to is available
    if skill_manager.get_skill_state("navigate_to") not in [None, "COMPLETED", "FAILED"]:
        return skill_manager.form_feedback("processing", "Waiting for navigate_to", 25)
    
    # Start navigate_to if not started
    if not skill_manager.get_skill_data(skill_name, "navigate_started", False):
        take_off_goal = skill_manager.get_skill_data(skill_name, "take_off_goal")
        goal_quat = skill_manager.get_skill_data(skill_name, "goal_quat")
        
        # Execute navigate_to skill
        from application.skills.navigate_to import navigate_to
        skill_manager.register_skill("navigate_to", navigate_to)
        skill_manager.execute_skill("navigate_to", goal_pos=take_off_goal, goal_quat_wxyz=goal_quat)
        
        skill_manager.set_skill_data(skill_name, "navigate_started", True)
        skill_manager.set_skill_data(skill_name, "navigate_start_time", robot.sim_time)
        return skill_manager.form_feedback("processing", "Navigating to altitude", 30)
    
    # Check navigate_to status
    nav_state = skill_manager.get_skill_state("navigate_to")
    
    if nav_state == "COMPLETED":
        # Get final altitude
        final_pos, _ = robot.body.get_world_pose()
        final_altitude = final_pos[2].item()
        
        skill_manager.set_skill_state(skill_name, "COMPLETED")
        return skill_manager.form_feedback(
            "completed",
            f"Take off completed at {final_altitude:.1f}m",
            100
        )
    
    elif nav_state == "FAILED":
        error = skill_manager.skill_errors.get("navigate_to", "Unknown error")
        skill_manager.set_skill_state(skill_name, "FAILED")
        skill_manager.skill_errors[skill_name] = f"Navigate failed: {error}"
        return skill_manager.form_feedback("failed", f"Navigate failed: {error}")
    
    else:
        # Check timeout
        elapsed = robot.sim_time - skill_manager.get_skill_data(skill_name, "navigate_start_time", robot.sim_time)
        if elapsed > 120.0:
            skill_manager.set_skill_state(skill_name, "FAILED")
            skill_manager.skill_errors[skill_name] = "Take off timeout"
            return skill_manager.form_feedback("failed", "Timeout")
        
        return skill_manager.form_feedback("processing", "Taking off...", 50)


def _handle_completed(robot, skill_manager, skill_name):
    final_pos, _ = robot.body.get_world_pose()
    final_altitude = final_pos[2].item()
    return skill_manager.form_feedback(
        "completed",
        f"Take off completed at {final_altitude:.1f}m",
        100
    )


def _handle_failed(robot, skill_manager, skill_name):
    error_msg = skill_manager.skill_errors.get(skill_name, "Unknown error")
    return skill_manager.form_feedback("failed", error_msg)
