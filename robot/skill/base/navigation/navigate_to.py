import json
from functools import partial

# ROS2
from nav2_msgs.action import ComputePathToPose


def navigate_to_skill(**kwargs):

    robot = kwargs.get("robot")
    goal_pos = kwargs.get("goal_pos")
    goal_quat_wxyz = kwargs.get("goal_quat_wxyz", [1.0, 0.0, 0.0, 0.0])

    if isinstance(goal_pos, str):
        goal_pos = json.loads(goal_pos)
    elif isinstance(goal_pos, tuple):
        goal_pos = list(goal_pos)
    if isinstance(goal_quat_wxyz, str):
        goal_quat_wxyz = json.loads(goal_quat_wxyz)
    elif isinstance(goal_quat_wxyz, tuple):
        goal_quat_wxyz = list(goal_quat_wxyz)

    if robot.is_planning:
        robot.node.get_logger().warn("Planning already in progress.")
        return robot.form_feedback("failed", "Planning already in progress.")

    if not robot.action_client_path_planner.wait_for_server(timeout_sec=2.0):
        robot.node.get_logger().error("Path planner server is not available.")
        return robot.form_feedback("failed", "Path planner server is not available.")

    robot.is_planning = True
    robot.node_controller_mpc.move_event.clear()
    robot.node.get_logger().info(f"Sending path request to goal: {goal_pos}")

    goal_msg = ComputePathToPose.Goal()

    start_pos_tensor, start_quat_tensor = robot.body.get_world_pose()
    # start_pos_tensor[2] = 1  # 这里有问题, ai写的烂代码, 不理解物理规律. 本身是要让机器人在地面上规划, 应该是0的, 但是写成了1
    start_pos = start_pos_tensor.cpu().numpy().tolist()
    start_quat = start_quat_tensor.cpu().numpy().tolist()

    goal_msg.start = _create_pose_stamped(robot, start_pos, start_quat)
    goal_msg.goal = _create_pose_stamped(robot, goal_pos, goal_quat_wxyz)

    send_goal_future = robot.action_client_path_planner.send_goal_async(goal_msg)
    send_goal_future.add_done_callback(partial(_goal_response_callback, robot))

    yield robot.form_feedback("processing", "Path Planning processing", 30)

    while True:
        if robot.node_controller_mpc.move_event.is_set():
            return robot.form_feedback("finished", "Navigation completed.", 100)
        else:
            yield robot.form_feedback("processing", "Navigation in progress...", 50)

            # 短暂休眠，避免过度占用CPU
            import time
            time.sleep(0.1)


def _goal_response_callback(robot, future):
    goal_handle = future.result()
    if not goal_handle.accepted:
        robot.node.get_logger().error("Goal was rejected by the planner server.")
    else:
        robot.node.get_logger().info("Goal was accepted by the planner server.")

    robot.is_planning = False


def _create_pose_stamped(robot, pos: list, quat_wxyz: list = [0.0, 0.0, 0.0, 0.0]):
    from geometry_msgs.msg import PoseStamped

    pose_stamped = PoseStamped()
    pose_stamped.header.stamp = robot.node.get_clock().now().to_msg()
    pose_stamped.header.frame_id = "map"

    pose_stamped.pose.position.x = float(pos[0])
    pose_stamped.pose.position.y = float(pos[1])
    pose_stamped.pose.position.z = float(pos[2])

    pose_stamped.pose.orientation.w = float(quat_wxyz[0])
    pose_stamped.pose.orientation.x = float(quat_wxyz[1])
    pose_stamped.pose.orientation.y = float(quat_wxyz[2])
    pose_stamped.pose.orientation.z = float(quat_wxyz[3])
    return pose_stamped
