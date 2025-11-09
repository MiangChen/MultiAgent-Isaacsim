"""Object Detection Skill - Application layer

Flow: Check camera -> Get semantic detection -> Complete
"""

from application import SkillManager


@SkillManager.register()
def object_detection(**kwargs):
    robot = kwargs.get("robot")
    skill_manager = kwargs.get("skill_manager")
    skill_name = "object_detection"
    
    current_state = skill_manager.get_skill_state(skill_name)
    
    if current_state in [None, "INITIALIZING"]:
        return _init_object_detection(robot, skill_manager, skill_name, kwargs)
    elif current_state == "EXECUTING":
        return _handle_executing(robot, skill_manager, skill_name)
    elif current_state == "COMPLETED":
        return _handle_completed(robot, skill_manager, skill_name)
    elif current_state == "FAILED":
        return _handle_failed(robot, skill_manager, skill_name)


def _init_object_detection(robot, skill_manager, skill_name, kwargs):
    # Get parameters
    camera_name = kwargs.get("camera_name", "default")
    target_class = kwargs.get("target_class", "car")
    
    # Check camera availability
    if not hasattr(robot, "camera_dict") or camera_name not in robot.camera_dict:
        skill_manager.set_skill_state(skill_name, "FAILED")
        skill_manager.skill_errors[skill_name] = f"Camera '{camera_name}' not available"
        return skill_manager.form_feedback("failed", f"Camera '{camera_name}' not found")
    
    try:
        # Store data
        skill_manager.set_skill_data(skill_name, "camera_name", camera_name)
        skill_manager.set_skill_data(skill_name, "target_class", target_class)
        
        skill_manager.set_skill_state(skill_name, "EXECUTING")
        return skill_manager.form_feedback("processing", "Initializing detection", 20)
        
    except Exception as e:
        skill_manager.set_skill_state(skill_name, "FAILED")
        skill_manager.skill_errors[skill_name] = f"Init failed: {str(e)}"
        return skill_manager.form_feedback("failed", f"Init failed: {str(e)}")


def _handle_executing(robot, skill_manager, skill_name):
    try:
        camera_name = skill_manager.get_skill_data(skill_name, "camera_name")
        target_class = skill_manager.get_skill_data(skill_name, "target_class")
        
        # Get camera
        camera = robot.camera_dict[camera_name]
        
        # Get semantic detection
        result = camera.get_semantic_detection()
        
        if result is None:
            error_msg = "Failed to get current frame from semantic camera"
            skill_manager.set_skill_data(skill_name, "last_error", error_msg)
            return skill_manager.form_feedback("processing", error_msg, 50)
        
        elif isinstance(result, str):
            # Still processing
            return skill_manager.form_feedback("processing", result, 70)
        
        else:
            # Detection completed
            detection_result = {
                "success": True,
                "message": f"Detected {target_class}",
                "data": result
            }
            skill_manager.set_skill_data(skill_name, "detection_result", detection_result)
            
            skill_manager.set_skill_state(skill_name, "COMPLETED")
            return skill_manager.form_feedback("completed", f"Detected {target_class}", 100)
        
    except Exception as e:
        skill_manager.set_skill_state(skill_name, "FAILED")
        skill_manager.skill_errors[skill_name] = f"Execution failed: {str(e)}"
        return skill_manager.form_feedback("failed", f"Execution failed: {str(e)}")


def _handle_completed(robot, skill_manager, skill_name):
    detection_result = skill_manager.get_skill_data(
        skill_name,
        "detection_result",
        {"success": False, "message": "No detection result", "data": None}
    )
    return skill_manager.form_feedback("completed", detection_result["message"], 100)


def _handle_failed(robot, skill_manager, skill_name):
    error_msg = skill_manager.skill_errors.get(skill_name, "Unknown error")
    return skill_manager.form_feedback("failed", error_msg)
