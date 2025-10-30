import json

# ROS2 Message
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose


def navigate_to_skill(**kwargs):
    robot = kwargs.get("robot")
    start_pos = kwargs.get("start_pos", None)
    robot.node_controller_mpc.has_reached_goal = False
    if type(start_pos) is str:
        start_pos = json.loads(start_pos)
    start_quat = kwargs.get("start_quat")
    if type(start_quat) is str:
        start_quat = json.loads(start_quat)
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

    if start_pos is None:
        start_pos_tensor, _ = robot.body.get_world_pose()
        # start_pos_tensor[2] = 1  # 这里有问题, ai写的烂代码, 不理解物理规律. 本身是要让机器人在地面上规划, 应该是0的, 但是写成了1
        start_pos = start_pos_tensor.cpu().numpy().tolist()
    if start_quat is None:
        _, start_quat_tensor = robot.body.get_world_pose()
        start_quat = start_quat_tensor.cpu().numpy().tolist()

    goal_msg.start = _create_pose_stamped(robot, start_pos, start_quat)
    goal_msg.goal = _create_pose_stamped(robot, goal_pos, goal_quat_wxyz)

    # 发送规划请求
    send_future = robot.action_client_path_planner.send_goal_async(goal_msg)

    # 等待响应 - 使用仿真时钟
    robot._nav_send_start_time = robot.sim_time

    if send_future.done():
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            robot.is_planning = False
            delattr(robot, '_nav_send_start_time')
            return robot.form_feedback("failed", "Request rejected")

        # 开始等待规划结果
        robot._nav_result_future = goal_handle.get_result_async()
        robot._nav_goal_handle = goal_handle
        robot._nav_plan_start_time = robot.sim_time
        delattr(robot, '_nav_send_start_time')
    else:
        # 检查超时
        elapsed_time = robot.sim_time - robot._nav_send_start_time

        if elapsed_time < 3.0:  # 3秒超时
            yield robot.form_feedback("processing", "Sending request...", 5)
        else:
            robot.is_planning = False
            delattr(robot, '_nav_send_start_time')
            return robot.form_feedback("failed", "No response from planner")

    # 等待规划结果 - 使用仿真时钟
    if robot._nav_result_future.done():
        # 检查结果
        result = robot._nav_result_future.result()
        robot.is_planning = False
        delattr(robot, '_nav_result_future')
        delattr(robot, '_nav_goal_handle')
        delattr(robot, '_nav_plan_start_time')

        if result.status != GoalStatus.STATUS_SUCCEEDED or not result.result.path.poses:
            return robot.form_feedback("failed", "Planning failed")

        # 执行导航
        yield robot.form_feedback("processing", "Executing...", 50)
        robot.node_controller_mpc.move_event.clear()
        robot._nav_move_start_time = robot.sim_time
    else:
        # 检查规划超时
        elapsed_time = robot.sim_time - robot._nav_plan_start_time

        if elapsed_time < 15.0:  # 15秒超时
            yield robot.form_feedback("processing", "Planning path...", 30)
        else:
            robot._nav_goal_handle.cancel_goal_async()
            robot.is_planning = False
            delattr(robot, '_nav_result_future')
            delattr(robot, '_nav_goal_handle')
            delattr(robot, '_nav_plan_start_time')
            return robot.form_feedback("failed", "Planning timeout")

    # 移动执行 - 使用仿真时钟
    if robot.node_controller_mpc.move_event.is_set():
        delattr(robot, '_nav_move_start_time')
        return robot.form_feedback("finished", "Done", 100)
    else:
        # 检查移动超时
        elapsed_time = robot.sim_time - robot._nav_move_start_time

        if elapsed_time < 120.0:
            # 计算动态进度
            progress = min(80 + (elapsed_time / 120.0) * 15, 95)
            yield robot.form_feedback("processing", f"Moving... ({elapsed_time:.1f}s)", int(progress))
        else:
            delattr(robot, '_nav_move_start_time')
            return robot.form_feedback("failed", "Navigation timeout")


def _create_pose_stamped(robot, pos: list, quat_wxyz: list = [0.0, 0.0, 0.0, 0.0]):
    pose_stamped = PoseStamped()

    sim_time_sec = int(robot.sim_time)
    sim_time_nanosec = int((robot.sim_time - sim_time_sec) * 1e9)
    pose_stamped.header.stamp = Time(sec=sim_time_sec, nanosec=sim_time_nanosec)

    pose_stamped.header.frame_id = "map"

    pose_stamped.pose.position.x = float(pos[0])
    pose_stamped.pose.position.y = float(pos[1])
    pose_stamped.pose.position.z = float(pos[2])

    pose_stamped.pose.orientation.w = float(quat_wxyz[0])
    pose_stamped.pose.orientation.x = float(quat_wxyz[1])
    pose_stamped.pose.orientation.y = float(quat_wxyz[2])
    pose_stamped.pose.orientation.z = float(quat_wxyz[3])
    return pose_stamped
