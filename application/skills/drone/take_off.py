from application import SkillManager
from simulation.control import RobotControl


@SkillManager.register()
def take_off(**kwargs):
    robot = kwargs.get("robot")
    skill_manager = kwargs.get("skill_manager")
    skill_name = "take_off"
    state = skill_manager.get_skill_state(skill_name)

    if state in [None, "INITIALIZING"]:
        altitude = kwargs.get("altitude", 1.0)
        if altitude <= 0:
            skill_manager.set_skill_state(skill_name, "FAILED")
            skill_manager.skill_errors[skill_name] = "Invalid altitude"
            return skill_manager.form_feedback("failed", "Invalid altitude")

        skill_manager.set_skill_data(skill_name, "target_altitude", altitude)
        skill_manager.set_skill_data(skill_name, "start_time", robot.sim_time)
        skill_manager.set_skill_state(skill_name, "EXECUTING")
        return skill_manager.form_feedback("processing", "Taking off", 10)

    elif state == "EXECUTING":
        target_alt = skill_manager.get_skill_data(skill_name, "target_altitude")
        start_time = skill_manager.get_skill_data(skill_name, "start_time")

        pos = robot.position
        current_alt = pos[2].item()

        # Reached target
        if current_alt >= target_alt - 0.1:
            control = RobotControl()
            control.linear_velocity = [0.0, 0.0, 0.0]
            robot.apply_control(control)
            skill_manager.set_skill_state(skill_name, "COMPLETED")
            return skill_manager.form_feedback("completed", "Reached altitude", 100)

        # Timeout
        if robot.sim_time - start_time > 30.0:
            skill_manager.set_skill_state(skill_name, "FAILED")
            skill_manager.skill_errors[skill_name] = "Timeout"
            return skill_manager.form_feedback("failed", "Timeout")

        # Apply upward velocity
        control = RobotControl()
        control.linear_velocity = [0.0, 0.0, 0.5]
        robot.apply_control(control)

        progress = min(int((current_alt / target_alt) * 100), 95)
        return skill_manager.form_feedback(
            "processing", f"{current_alt:.1f}m", progress
        )

    elif state == "COMPLETED":
        return skill_manager.form_feedback("completed", "Completed", 100)

    elif state == "FAILED":
        error = skill_manager.skill_errors.get(skill_name, "Unknown")
        return skill_manager.form_feedback("failed", error)
