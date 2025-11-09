"""Pick Up Skill - Application layer

Flow: Check distance -> Create joint -> Complete
"""

import json
import numpy as np
import torch
from physics_engine.isaacsim_utils import RigidPrim
from physics_engine.pxr_utils import UsdPhysics, Gf
from application import SkillManager


@SkillManager.register()
def pick_up(**kwargs):
    robot = kwargs.get("robot")
    skill_manager = kwargs.get("skill_manager")
    skill_name = "pick_up"
    
    current_state = skill_manager.get_skill_state(skill_name)
    
    if current_state in [None, "INITIALIZING"]:
        return _init_pick_up(robot, skill_manager, skill_name, kwargs)
    elif current_state == "EXECUTING":
        return _handle_executing(robot, skill_manager, skill_name)
    elif current_state == "COMPLETED":
        return _handle_completed(robot, skill_manager, skill_name)
    elif current_state == "FAILED":
        return _handle_failed(robot, skill_manager, skill_name)


def _init_pick_up(robot, skill_manager, skill_name, kwargs):
    # Get parameters
    robot_hand_prim_path = kwargs.get("robot_hand_prim_path")
    object_prim_path = kwargs.get("object_prim_path")
    distance_threshold = kwargs.get("distance_threshold", 2.0)
    axis = kwargs.get("axis", [0, 0, 1])
    local_pos_hand = kwargs.get("local_pos_hand", [0, 0, 1])
    local_pos_object = kwargs.get("local_pos_object", [0, 0, 0])
    
    # Parse string parameters
    if isinstance(axis, str):
        axis = json.loads(axis)
    if isinstance(local_pos_hand, str):
        local_pos_hand = json.loads(local_pos_hand)
    if isinstance(local_pos_object, str):
        local_pos_object = json.loads(local_pos_object)
    
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
        
        # Calculate distance
        hand_pos, _ = hand_prim.get_world_poses()
        object_pos, _ = object_prim.get_world_poses()
        distance = np.linalg.norm(hand_pos - object_pos)
        
        # Check distance threshold
        if distance > distance_threshold:
            skill_manager.set_skill_state(skill_name, "FAILED")
            skill_manager.skill_errors[skill_name] = (
                f"Object too far ({distance:.2f}m > {distance_threshold}m)"
            )
            return skill_manager.form_feedback("failed", f"Too far: {distance:.2f}m")
        
        # Store data
        skill_manager.set_skill_data(skill_name, "hand_prim", hand_prim)
        skill_manager.set_skill_data(skill_name, "object_prim", object_prim)
        skill_manager.set_skill_data(skill_name, "robot_hand_prim_path", robot_hand_prim_path)
        skill_manager.set_skill_data(skill_name, "object_prim_path", object_prim_path)
        skill_manager.set_skill_data(skill_name, "axis", axis)
        skill_manager.set_skill_data(skill_name, "local_pos_hand", local_pos_hand)
        skill_manager.set_skill_data(skill_name, "local_pos_object", local_pos_object)
        skill_manager.set_skill_data(skill_name, "distance", distance)
        
        skill_manager.set_skill_state(skill_name, "EXECUTING")
        return skill_manager.form_feedback("processing", f"Distance OK: {distance:.2f}m", 20)
        
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
        robot_hand_prim_path = skill_manager.get_skill_data(skill_name, "robot_hand_prim_path")
        local_pos_hand = skill_manager.get_skill_data(skill_name, "local_pos_hand")
        local_pos_object = skill_manager.get_skill_data(skill_name, "local_pos_object")
        axis = skill_manager.get_skill_data(skill_name, "axis")
        
        # Stop object motion
        object_prim.set_linear_velocities(torch.zeros(3, dtype=torch.float32))
        object_prim.set_angular_velocities(torch.zeros(3, dtype=torch.float32))
        
        # Move object to hand position
        hand_pos, _ = hand_prim.get_world_poses()
        object_prim.set_world_poses(positions=hand_pos)
        
        # Disable collision
        robot.scene_manager.set_collision_enabled(
            prim_path=object_prim_path, collision_enabled=False
        )
        
        # Create or update joint
        joint_path = f"/World/grasp_joint_{object_prim.name}"
        skill_manager.set_skill_data(skill_name, "joint_path", joint_path)
        joint_prim = robot.scene_manager.stage.GetPrimAtPath(joint_path)
        
        if not joint_prim.IsValid():
            robot.scene_manager.create_joint(
                joint_path=joint_path,
                joint_type="fixed",
                body0=robot_hand_prim_path,
                body1=object_prim_path,
                local_pos0=local_pos_hand,
                local_pos1=local_pos_object,
                axis=axis,
            )
            joint_prim = robot.scene_manager.stage.GetPrimAtPath(joint_path)
        
        # Enable joint
        joint = UsdPhysics.Joint(joint_prim)
        joint.GetLocalPos0Attr().Set(Gf.Vec3f(local_pos_hand))
        joint.GetLocalPos1Attr().Set(Gf.Vec3f(local_pos_object))
        joint.GetJointEnabledAttr().Set(True)
        
        skill_manager.set_skill_state(skill_name, "COMPLETED")
        return skill_manager.form_feedback("completed", "Object picked up", 100)
        
    except Exception as e:
        skill_manager.set_skill_state(skill_name, "FAILED")
        skill_manager.skill_errors[skill_name] = f"Execution failed: {str(e)}"
        return skill_manager.form_feedback("failed", f"Execution failed: {str(e)}")


def _handle_completed(robot, skill_manager, skill_name):
    return skill_manager.form_feedback("completed", "Object picked up", 100)


def _handle_failed(robot, skill_manager, skill_name):
    error_msg = skill_manager.skill_errors.get(skill_name, "Unknown error")
    return skill_manager.form_feedback("failed", error_msg)
