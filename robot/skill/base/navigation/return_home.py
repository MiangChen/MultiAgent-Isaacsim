from utils.form_feedback import form_feedback

def return_home_skill(**kwargs):
    robot = kwargs.get("robot")

    result = {"success": False, "message": "", "data": None}

    robot.navigate_to(list(robot.body.cfg_robot.base))

    result["success"] = True
    return form_feedback("")
    return result
