def navigate_to_skill(**kwargs):
    robot = kwargs.get("robot")
    goal_pos = kwargs.get("goal_pos")
    goal_quat_wxyz = kwargs.get("goal_quat_wxyz", [1.0, 0.0, 0.0, 0.0])

    result = {"success": False, "message": "", "data": None}

    if robot.is_planning:
        result["message"] = "Planning already in progress"
        return result

    if not robot.action_client_path_planner.wait_for_server(timeout_sec=2.0):
        result["message"] = "Path planner server is not available"
        return result

    robot.is_planning = True
    robot.node.get_logger().info(f"Sending path request to goal: {goal_pos}")

    from nav2_msgs.action import ComputePathToPose

    goal_msg = ComputePathToPose.Goal()

    start_pos_tensor, start_quat_tensor = robot.body.get_world_pose()
    start_pos_tensor[2] = 1
    start_pos = start_pos_tensor.cpu().numpy().tolist()
    start_quat = start_quat_tensor.cpu().numpy().tolist()

    goal_msg.start = _create_pose_stamped(robot, start_pos, start_quat)
    goal_msg.goal = _create_pose_stamped(robot, goal_pos, goal_quat_wxyz)

    send_goal_future = robot.action_client_path_planner.send_goal_async(goal_msg)
    send_goal_future.add_done_callback(
        lambda future: _goal_response_callback(robot, future)
    )

    result["success"] = True
    result["message"] = "Navigation started"
    return result


def _goal_response_callback(robot, future):
    goal_handle = future.result()
    if not goal_handle.accepted:
        robot.node.get_logger().error("Goal was rejected by the planner server.")
    else:
        robot.node.get_logger().info("Goal was accepted by the planner server.")

    robot.is_planning = False


def _create_pose_stamped(robot, pos: list, quat_wxyz: list):
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
