"""Detect Skill - Application layer

Flow: Check components -> Perform overlap detection -> Complete
"""

from application.skill_registry import SkillRegistry


@SkillRegistry.register()
def detect(**kwargs):
    robot = kwargs.get("robot")
    skill_manager = kwargs.get("skill_manager")
    skill_name = "detect"
    
    current_state = skill_manager.get_skill_state(skill_name)
    
    if current_state in [None, "INITIALIZING"]:
        return _init_detect(robot, skill_manager, skill_name, kwargs)
    elif current_state == "EXECUTING":
        return _handle_executing(robot, skill_manager, skill_name)
    elif current_state == "COMPLETED":
        return _handle_completed(robot, skill_manager, skill_name)
    elif current_state == "FAILED":
        return _handle_failed(robot, skill_manager, skill_name)


def _init_detect(robot, skill_manager, skill_name, kwargs):
    # Get parameters
    target_prim = kwargs.get("target_prim")
    
    # Validate parameters
    if not target_prim:
        skill_manager.set_skill_state(skill_name, "FAILED")
        skill_manager.skill_errors[skill_name] = "Target prim required"
        return skill_manager.form_feedback("failed", "Target prim required")
    
    # Check robot body
    if not hasattr(robot, "body") or robot.body is None:
        skill_manager.set_skill_state(skill_name, "FAILED")
        skill_manager.skill_errors[skill_name] = "Robot body not available"
        return skill_manager.form_feedback("failed", "Robot body not available")
    
    # Check scene manager
    if not hasattr(robot, "scene_manager") or robot.scene_manager is None:
        skill_manager.set_skill_state(skill_name, "FAILED")
        skill_manager.skill_errors[skill_name] = "Scene manager not available"
        return skill_manager.form_feedback("failed", "Scene manager not available")
    
    try:
        # Store data
        skill_manager.set_skill_data(skill_name, "target_prim", target_prim)
        
        skill_manager.set_skill_state(skill_name, "EXECUTING")
        return skill_manager.form_feedback("processing", "Initializing detection", 20)
        
    except Exception as e:
        skill_manager.set_skill_state(skill_name, "FAILED")
        skill_manager.skill_errors[skill_name] = f"Init failed: {str(e)}"
        return skill_manager.form_feedback("failed", f"Init failed: {str(e)}")


def _handle_executing(robot, skill_manager, skill_name):
    try:
        target_prim = skill_manager.get_skill_data(skill_name, "target_prim")
        
        # Get robot position
        pos, quat = robot.body.get_world_pose()
        
        # Perform overlap detection
        detection_result = robot.scene_manager.overlap_hits_target_ancestor(target_prim)
        
        # Store result
        detection_result_data = {
            "success": True,
            "message": f"Detection completed for {target_prim}",
            "data": detection_result,
        }
        skill_manager.set_skill_data(skill_name, "detection_result", detection_result_data)
        
        skill_manager.set_skill_state(skill_name, "COMPLETED")
        return skill_manager.form_feedback("completed", f"Detected {target_prim}", 100)
        
    except Exception as e:
        skill_manager.set_skill_state(skill_name, "FAILED")
        skill_manager.skill_errors[skill_name] = f"Execution failed: {str(e)}"
        return skill_manager.form_feedback("failed", f"Execution failed: {str(e)}")


def _handle_completed(robot, skill_manager, skill_name):
    result = skill_manager.get_skill_data(
        skill_name,
        "detection_result",
        {"success": False, "message": "Unknown result", "data": None}
    )
    return skill_manager.form_feedback("completed", result["message"], 100)


def _handle_failed(robot, skill_manager, skill_name):
    error_msg = skill_manager.skill_errors.get(skill_name, "Unknown error")
    return skill_manager.form_feedback("failed", error_msg)
