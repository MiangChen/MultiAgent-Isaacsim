def stop_tracking(robot):
    robot.is_tracking = False
    robot.track_waypoint_list = []
    robot.track_waypoint_index = 0
    robot.node.destroy_subscription(robot.track_waypoint_sub)
    robot.track_waypoint_sub = None
    return robot.form_feedback(status="finished")
