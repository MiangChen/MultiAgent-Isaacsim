def explore_skill(**kwargs):
    robot = kwargs.get("robot")
    boundary = kwargs.get("boundary")
    holes = kwargs.get("holes")
    target_prim = kwargs.get("target_prim", "/TARGET_PRIM_NOT_SPECIFIED")

    waypoints = robot.plan_exploration_waypoints(
        boundary,
        holes,
        lane_width=robot.body.cfg_robot.detection_radius,
        robot_radius=robot.body.cfg_robot.robot_radius,
    )
    yield robot.form_feedback("processing", "Exploration waypoints planned.", 30)

    robot.is_detecting = True
    robot.target_prim = target_prim
    robot.node_controller_mpc.move_event.clear()
    robot.node_planner_ompl.publisher_path.publish(waypoints)

    result = robot.node_controller_mpc.move_event.wait(timeout=100)

    if result:
        return robot.form_feedback("finished", "Exploration completed.", 100)
    else:
        return robot.form_feedback("failed", "Exploration failed. / Time exceeded", 0)

