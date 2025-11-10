"""Put Down Skill - Application layer

Flow: Release object -> Complete

Architecture:
- Application layer creates Control objects (pure data)
- Robot layer executes them in on_physics_step
- Application layer queries results asynchronously
"""

from application import SkillManager
from dependency_injector.wiring import inject, Provide


@SkillManager.register()
@inject
def put_down(world=Provide["world_configured"], **kwargs):
    robot = kwargs.get("robot")
    skill_manager = kwargs.get("skill_manager")
    skill_name = "put_down"

    current_state = skill_manager.get_skill_state(skill_name)

    if current_state in [None, "INITIALIZING"]:
        return _init_put_down(robot, skill_manager, skill_name, world, kwargs)
    elif current_state == "RELEASING":
        return _handle_releasing(robot, skill_manager, skill_name)
    elif current_state == "COMPLETED":
        return _handle_completed(robot, skill_manager, skill_name)
    elif current_state == "FAILED":
        return _handle_failed(robot, skill_manager, skill_name)


def _init_put_down(robot, skill_manager, skill_name, world, kwargs):
    """
    Initialize put_down skill - Create Control object (pure data, no Isaac Sim API)

    Architecture:
    - Application layer: Create ReleaseControl object
    - Robot layer: Execute in on_physics_step
    - Application layer: Query result asynchronously
    """
    from simulation.control import ReleaseControl, ControlAction

    # Get parameters
    object_prim_path = kwargs.get("object_prim_path")
    joint_path = kwargs.get("joint_path")

    # Validate parameters
    if not object_prim_path:
        skill_manager.set_skill_state(skill_name, "FAILED")
        skill_manager.skill_errors[skill_name] = "Object prim path required"
        return skill_manager.form_feedback("failed", "Object path required")

    # If joint_path not provided, try to get from pick_up skill
    if not joint_path:
        joint_path = skill_manager.get_skill_data("pick_up", "joint_path")
        if not joint_path:
            skill_manager.set_skill_state(skill_name, "FAILED")
            skill_manager.skill_errors[skill_name] = (
                "Joint path not found (object not grasped?)"
            )
            return skill_manager.form_feedback("failed", "Joint path not found")

    # Create ReleaseControl object (pure data, no Isaac Sim API call)
    release_control = ReleaseControl(
        object_prim_path=object_prim_path,
        joint_path=joint_path,
        action=ControlAction.RELEASE,  # Use Enum (already default, but explicit)
    )

    # Apply control (Robot will execute in on_physics_step)
    robot.apply_manipulation_control(release_control)

    skill_manager.set_skill_state(skill_name, "RELEASING")
    return skill_manager.form_feedback("processing", "Releasing object", 50)


def _handle_releasing(robot, skill_manager, skill_name):
    """
    Handle RELEASING state - Query result from Robot layer

    No Isaac Sim API calls here, only query results from Robot.
    """
    # Query result from Robot layer (async)
    result = robot.get_manipulation_result()

    if result is None:
        # Still processing, wait for next cycle
        return skill_manager.form_feedback("processing", "Releasing object", 70)

    if result["success"]:
        # Release succeeded
        skill_manager.set_skill_state(skill_name, "COMPLETED")
        return skill_manager.form_feedback("completed", "Object put down", 100)
    else:
        # Release failed
        skill_manager.set_skill_state(skill_name, "FAILED")
        skill_manager.skill_errors[skill_name] = result["message"]
        return skill_manager.form_feedback("failed", result["message"])


def _handle_completed(robot, skill_manager, skill_name):
    return skill_manager.form_feedback("completed", "Object put down", 100)


def _handle_failed(robot, skill_manager, skill_name):
    error_msg = skill_manager.skill_errors.get(skill_name, "Unknown error")
    return skill_manager.form_feedback("failed", error_msg)
