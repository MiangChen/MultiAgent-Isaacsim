def explore_skill(**kwargs):
    robot = kwargs.get("robot")
    boundary = kwargs.get("boundary")
    holes = kwargs.get("holes")
    target_prim = kwargs.get("target_prim", "/TARGET_PRIM_NOT_SPECIFIED")

    result = {"success": False, "message": "", "data": None}

    waypoints = robot.plan_exploration_waypoints(
        boundary,
        holes,
        lane_width=robot.body.cfg_robot.detection_radius,
        robot_radius=robot.body.cfg_robot.robot_radius,
    )
    robot.is_detecting = True
    robot.target_prim = target_prim
    robot.move_along_path(waypoints, flag_reset=True)

    result["success"] = True
    result["message"] = "Exploration started"
    result["data"] = {"waypoints": waypoints, "target_prim": target_prim}

    return result
