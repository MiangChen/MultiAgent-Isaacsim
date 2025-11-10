from application import SkillManager
from dependency_injector.wiring import inject, Provide


@SkillManager.register()
@inject
def detect(
    world=Provide["world_configured"],
    **kwargs
):
    robot = kwargs.get("robot")
    skill_manager = kwargs.get("skill_manager")
    skill_name = "detect"
    state = skill_manager.get_skill_state(skill_name)
    
    if state in [None, "INITIALIZING"]:
        target_prim = kwargs.get("target_prim")
        if not target_prim:
            skill_manager.set_skill_state(skill_name, "FAILED")
            skill_manager.skill_errors[skill_name] = "Target prim required"
            return skill_manager.form_feedback("failed", "Target prim required")
        
        skill_manager.set_skill_data(skill_name, "target_prim", target_prim)
        skill_manager.set_skill_state(skill_name, "EXECUTING")
        return skill_manager.form_feedback("processing", "Detecting", 20)
    
    elif state == "EXECUTING":
        target_prim = skill_manager.get_skill_data(skill_name, "target_prim")
        
        # Perform detection
        detection_result = world.overlap_test(target_prim) if world else None
        skill_manager.set_skill_data(skill_name, "detection_result", detection_result)
        
        skill_manager.set_skill_state(skill_name, "COMPLETED")
        return skill_manager.form_feedback("completed", f"Detected: {detection_result}", 100)
    
    elif state == "COMPLETED":
        result = skill_manager.get_skill_data(skill_name, "detection_result", False)
        return skill_manager.form_feedback("completed", f"Result: {result}", 100)
    
    elif state == "FAILED":
        error = skill_manager.skill_errors.get(skill_name, "Unknown")
        return skill_manager.form_feedback("failed", error)
