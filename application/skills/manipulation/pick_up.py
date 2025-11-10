"""Pick Up Skill - Application layer

Flow: Check distance -> Attach object -> Complete

Architecture:
- Application layer creates Control objects (pure data)
- Robot layer executes them in on_physics_step
- Application layer queries results asynchronously
"""

import json
from application import SkillManager


@SkillManager.register()
def pick_up(
        **kwargs
):
    robot = kwargs.get("robot")
    skill_manager = kwargs.get("skill_manager")
    skill_name = "pick_up"

    current_state = skill_manager.get_skill_state(skill_name)

    if current_state in [None, "INITIALIZING"]:
        return _init_pick_up(robot, skill_manager, skill_name, kwargs)
    elif current_state == "CHECKING":
        return _handle_checking(robot, skill_manager, skill_name)
    elif current_state == "ATTACHING":
        return _handle_attaching(robot, skill_manager, skill_name)
    elif current_state == "COMPLETED":
        return _handle_completed(robot, skill_manager, skill_name)
    elif current_state == "FAILED":
        return _handle_failed(robot, skill_manager, skill_name)


def _init_pick_up(robot, skill_manager, skill_name, kwargs):
    """
    Initialize pick_up skill - Create Control object (pure data, no Isaac Sim API)
    
    Architecture:
    - Application layer: Create GraspControl object
    - Robot layer: Execute in on_physics_step
    - Application layer: Query result asynchronously
    """
    from simulation.control import GraspControl, ControlAction

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

    # Create GraspControl object (pure data, no Isaac Sim API call)
    grasp_control = GraspControl(
        hand_prim_path=robot_hand_prim_path,
        object_prim_path=object_prim_path,
        action=ControlAction.CHECK_DISTANCE,  # Use Enum instead of string
        distance_threshold=distance_threshold,
        local_pos_hand=local_pos_hand,
        local_pos_object=local_pos_object,
        axis=axis,
    )

    # Apply control (Robot will execute in on_physics_step)
    robot.apply_manipulation_control(grasp_control)

    # Save control for later use
    skill_manager.set_skill_data(skill_name, "grasp_control", grasp_control)
    skill_manager.set_skill_state(skill_name, "CHECKING")
    return skill_manager.form_feedback("processing", "Checking distance", 20)


def _handle_checking(robot, skill_manager, skill_name):
    """
    Handle CHECKING state - Query result from Robot layer
    
    No Isaac Sim API calls here, only query results from Robot.
    """
    # Query result from Robot layer (async)
    result = robot.get_manipulation_result()

    if result is None:
        # Still processing, wait for next cycle
        return skill_manager.form_feedback("processing", "Checking distance", 30)

    if result['success']:
        # Distance check passed, proceed to attach
        from simulation.control import ControlAction
        grasp_control = skill_manager.get_skill_data(skill_name, "grasp_control")
        grasp_control.action = ControlAction.ATTACH  # Use Enum instead of string

        # Apply attach control
        robot.apply_manipulation_control(grasp_control)

        skill_manager.set_skill_state(skill_name, "ATTACHING")
        return skill_manager.form_feedback("processing", "Attaching object", 60)
    else:
        # Distance check failed
        skill_manager.set_skill_state(skill_name, "FAILED")
        skill_manager.skill_errors[skill_name] = result['message']
        return skill_manager.form_feedback("failed", result['message'])


def _handle_attaching(robot, skill_manager, skill_name):
    """
    Handle ATTACHING state - Query attach result from Robot layer
    
    No Isaac Sim API calls here, only query results from Robot.
    """
    # Query result from Robot layer (async)
    result = robot.get_manipulation_result()

    if result is None:
        # Still processing, wait for next cycle
        return skill_manager.form_feedback("processing", "Attaching object", 70)

    if result['success']:
        # Attach succeeded
        joint_path = result['data'].get('joint_path')
        skill_manager.set_skill_data(skill_name, "joint_path", joint_path)

        skill_manager.set_skill_state(skill_name, "COMPLETED")
        return skill_manager.form_feedback("completed", "Object picked up", 100)
    else:
        # Attach failed
        skill_manager.set_skill_state(skill_name, "FAILED")
        skill_manager.skill_errors[skill_name] = result['message']
        return skill_manager.form_feedback("failed", result['message'])


def _handle_completed(robot, skill_manager, skill_name):
    return skill_manager.form_feedback("completed", "Object picked up", 100)


def _handle_failed(robot, skill_manager, skill_name):
    error_msg = skill_manager.skill_errors.get(skill_name, "Unknown error")
    return skill_manager.form_feedback("failed", error_msg)
