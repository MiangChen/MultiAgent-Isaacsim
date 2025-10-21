def return_home_skill(**kwargs):
    robot = kwargs.get("robot")

    result = {"success": False, "message": "", "data": None}

    robot.navigate_to(robot.body.cfg_robot.base)

    result["success"] = True
    result["message"] = "Returning home"
    return result
