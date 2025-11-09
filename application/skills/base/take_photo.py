"""Take Photo Skill - Application layer

Flow: Check camera -> Capture image -> Save (optional) -> Complete
"""

from application.skill_registry import SkillRegistry


@SkillRegistry.register()
def take_photo(**kwargs):
    robot = kwargs.get("robot")
    skill_manager = kwargs.get("skill_manager")
    skill_name = "take_photo"
    
    current_state = skill_manager.get_skill_state(skill_name)
    
    if current_state in [None, "INITIALIZING"]:
        return _init_take_photo(robot, skill_manager, skill_name, kwargs)
    elif current_state == "EXECUTING":
        return _handle_executing(robot, skill_manager, skill_name)
    elif current_state == "COMPLETED":
        return _handle_completed(robot, skill_manager, skill_name)
    elif current_state == "FAILED":
        return _handle_failed(robot, skill_manager, skill_name)


def _init_take_photo(robot, skill_manager, skill_name, kwargs):
    # Get parameters
    camera_name = kwargs.get("camera_name", "default")
    save_to_file = kwargs.get("save_to_file", "")
    
    # Check camera availability
    if not hasattr(robot, "camera_dict") or camera_name not in robot.camera_dict:
        skill_manager.set_skill_state(skill_name, "FAILED")
        skill_manager.skill_errors[skill_name] = f"Camera '{camera_name}' not available"
        return skill_manager.form_feedback("failed", f"Camera '{camera_name}' not found")
    
    try:
        # Store data
        skill_manager.set_skill_data(skill_name, "camera_name", camera_name)
        skill_manager.set_skill_data(skill_name, "save_to_file", save_to_file)
        
        skill_manager.set_skill_state(skill_name, "EXECUTING")
        return skill_manager.form_feedback("processing", "Preparing camera", 20)
        
    except Exception as e:
        skill_manager.set_skill_state(skill_name, "FAILED")
        skill_manager.skill_errors[skill_name] = f"Init failed: {str(e)}"
        return skill_manager.form_feedback("failed", f"Init failed: {str(e)}")


def _handle_executing(robot, skill_manager, skill_name):
    try:
        camera_name = skill_manager.get_skill_data(skill_name, "camera_name")
        save_to_file = skill_manager.get_skill_data(skill_name, "save_to_file")
        
        # Get camera
        camera = robot.camera_dict[camera_name]
        
        # Capture image
        rgb = camera.get_rgb()
        
        if rgb is None:
            skill_manager.set_skill_state(skill_name, "FAILED")
            skill_manager.skill_errors[skill_name] = "Failed to capture image"
            return skill_manager.form_feedback("failed", "Capture failed")
        
        # Store photo data
        photo_data = {
            "rgb_image": rgb,
            "camera_name": camera_name,
        }
        skill_manager.set_skill_data(skill_name, "photo_data", photo_data)
        
        # Save to file (optional)
        feedback_msg = "Photo captured"
        if save_to_file:
            save_result = camera.save_rgb_to_file(rgb=rgb, file_path=save_to_file)
            if save_result:
                feedback_msg = f"Saved to {save_to_file}"
        
        skill_manager.set_skill_state(skill_name, "COMPLETED")
        return skill_manager.form_feedback("completed", feedback_msg, 100)
        
    except Exception as e:
        skill_manager.set_skill_state(skill_name, "FAILED")
        skill_manager.skill_errors[skill_name] = f"Execution failed: {str(e)}"
        return skill_manager.form_feedback("failed", f"Execution failed: {str(e)}")


def _handle_completed(robot, skill_manager, skill_name):
    return skill_manager.form_feedback("completed", "Photo taken", 100)


def _handle_failed(robot, skill_manager, skill_name):
    error_msg = skill_manager.skill_errors.get(skill_name, "Unknown error")
    return skill_manager.form_feedback("failed", error_msg)
