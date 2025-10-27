from nav_msgs.msg import Odometry, Path

def start_tracking_skill(robot, target_prim: str = None):
    robot.is_tracking = True
    robot.track_waypoint_sub = robot.node.create_subscription(
        Odometry,
        "/target_0/odom",
        robot.track_callback,
        50,
    )
    return robot.form_feedback(status="normal")