"""Put Down Skill - Application layer

Flow: Check joint -> Disable joint -> Restore physics -> Complete
"""

from physics_engine.isaacsim_utils import RigidPrim
from physics_engine.pxr_utils import UsdPhysics
from application.skill_registry import SkillRegistry


@SkillRegistry.register()
def put_down(**kwargs):
    robot = kwargs.get("robot")
    skill_manager = kwargs.get("skill_manager")
    skill_name = "put_down"
    
    current_state = skill_manager.get_skill_state(skill_name)
    
    if current_state in [None, "INITIALIZING"]:
        return _init_put_down(robot, skill_manager, skill_name, kwargs)
    elif current_state == "EXECUTING":
        return _handle_executing(robot, skill_manager, skill_name)
    elif current_state == "COMPLETED":
        return _handle_completed(robot, skill_manager, skill_name)
    elif current_state == "FAILED":
        return _handle_failed(robot, skill_manager, skill_name)


def _init_put_down(robot, skill_manager, skill_name, kwargs):
    # Get parameters
    robot_hand_prim_path = kwargs.get("robot_hand_prim_path")
    object_prim_path = kwargs.get("object_prim_path")
    
    # Validate parameters
    if not robot_hand_prim_path:
        skill_manager.set_skill_state(skill_name, "FAILED")
        skill_manager.skill_errors[skill_name] = "Robot hand prim path required"
        return skill_manager.form_feedback("failed", "Hand path required")
    
    if not object_prim_path:
        skill_manager.set_skill_state(skill_name, "FAILED")
        skill_manager.skill_errors[skill_name] = "Object prim path required"
        return skill_manager.form_feedback("failed", "Object path required")
    
    # Check scene manager
    if not hasattr(robot, "scene_manager") or robot.scene_manager is None:
        skill_manager.set_skill_state(skill_name, "FAILED")
        skill_manager.skill_errors[skill_name] = "Scene manager not available"
        return skill_manager.form_feedback("failed", "Scene manager not available")
    
    try:
        # Initialize rigid prims
        hand_prim = RigidPrim(prim_paths_expr=robot_hand_prim_path)
        object_prim = RigidPrim(prim_paths_expr=object_prim_path)
        
        # Build joint path
        joint_path = f"/World/grasp_joint_{object_prim.name}"
        
        # Store data
        skill_manager.set_skill_data(skill_name, "hand_prim", hand_prim)
        skill_manager.set_skill_data(skill_name, "object_prim", object_prim)
        skill_manager.set_skill_data(skill_name, "object_prim_path", object_prim_path)
        skill_manager.set_skill_data(skill_name, "joint_path", joint_path)
        
        skill_manager.set_skill_state(skill_name, "EXECUTING")
        return skill_manager.form_feedback("processing", "Preparing to put down", 20)
        
    except Exception as e:
        skill_manager.set_skill_state(skill_name, "FAILED")
        skill_manager.skill_errors[skill_name] = f"Init failed: {str(e)}"
        return skill_manager.form_feedback("failed", f"Init failed: {str(e)}")


def _handle_executing(robot, skill_manager, skill_name):
    try:
        # Get stored data
        hand_prim = skill_manager.get_skill_data(skill_name, "hand_prim")
        object_prim = skill_manager.get_skill_data(skill_name, "object_prim")
        object_prim_path = skill_manager.get_skill_data(skill_name, "object_prim_path")
        joint_path = skill_manager.get_skill_data(skill_name, "joint_path")
        
        # Get joint
        stage = robot.scene_manager.stage
        joint_prim = stage.GetPrimAtPath(joint_path)
        
        if not joint_prim.IsValid():
            skill_manager.set_skill_state(skill_name, "FAILED")
            skill_manager.skill_errors[skill_name] = "Grasp joint not found"
            return skill_manager.form_feedback("failed", "Joint not found")
        
        # Disable joint
        joint = UsdPhysics.Joint(joint_prim)
        joint.GetJointEnabledAttr().Set(False)
        
        # Enable collision
        robot.scene_manager.set_collision_enabled(
            object_prim_path, collision_enabled=True
        )
        
        # Transfer momentum
        hand_lin_vel = hand_prim.get_linear_velocities()
        hand_ang_vel = hand_prim.get_angular_velocities()
        object_prim.set_linear_velocities(hand_lin_vel)
        object_prim.set_angular_velocities(hand_ang_vel)
        
        skill_manager.set_skill_state(skill_name, "COMPLETED")
        return skill_manager.form_feedback("completed", "Object put down", 100)
        
    except Exception as e:
        skill_manager.set_skill_state(skill_name, "FAILED")
        skill_manager.skill_errors[skill_name] = f"Execution failed: {str(e)}"
        return skill_manager.form_feedback("failed", f"Execution failed: {str(e)}")


def _handle_completed(robot, skill_manager, skill_name):
    return skill_manager.form_feedback("completed", "Object put down", 100)


def _handle_failed(robot, skill_manager, skill_name):
    error_msg = skill_manager.skill_errors.get(skill_name, "Unknown error")
    return skill_manager.form_feedback("failed", error_msg)
