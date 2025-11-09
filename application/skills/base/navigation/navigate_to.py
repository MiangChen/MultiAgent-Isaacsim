"""
Navigate To Skill - Application layer

Architecture:
    1. Path Planning: ROS action (gridmap via ROS)
    2. Velocity Computation: MPC controller (application layer)
    3. Execution: Control objects (simulation layer)

Flow:
    navigate_to -> ROS path planning -> MPC computes velocity -> 
    RobotControl object -> robot.apply_control() -> Isaac Sim
"""

import json
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from application import SkillManager


@SkillManager.register()
def navigate_to(**kwargs):
    """
    Navigate to target position
    
    Application layer skill:
    - ROS: Path planning (gridmap subscription)
    - MPC: Velocity computation
    - Control: Execution via robot.apply_control()
    """
    robot = kwargs.get("robot")
    skill_manager = kwargs.get("skill_manager")
    skill_name = "navigate_to"
    
    current_state = skill_manager.get_skill_state(skill_name)
    
    # State machine
    if current_state in [None, "INITIALIZING"]:
        _init_navigate_to(robot, skill_manager, skill_name, kwargs)
    
    current_state = skill_manager.get_skill_state(skill_name)
    
    if current_state == "EXECUTING":
        return _handle_executing(robot, skill_manager, skill_name)
    elif current_state == "COMPLETED":
        return _handle_completed(skill_manager, skill_name)
    elif current_state == "FAILED":
        return _handle_failed(skill_manager, skill_name)


def _init_navigate_to(robot, skill_manager, skill_name, kwargs):
    """Initialize navigation"""
    # Check if ROS is available
    if not robot.has_ros():
        skill_manager.set_skill_state(skill_name, "FAILED")
        skill_manager.skill_errors[skill_name] = "ROS not available - navigation requires ROS"
        return
    
    robot.ros_manager.get_node_controller_mpc().has_reached_goal = False
    skill_manager.set_skill_state(skill_name, "EXECUTING")
    
    # Parse parameters
    start_pos = kwargs.get("start_pos", None)
    start_quat = kwargs.get("start_quat")
    goal_pos = kwargs.get("goal_pos")
    goal_quat_wxyz = kwargs.get("goal_quat_wxyz", [1.0, 0.0, 0.0, 0.0])
    
    if isinstance(start_pos, str):
        start_pos = json.loads(start_pos)
    if isinstance(start_quat, str):
        start_quat = json.loads(start_quat)
    if isinstance(goal_pos, str):
        goal_pos = json.loads(goal_pos)
    elif isinstance(goal_pos, tuple):
        goal_pos = list(goal_pos)
    if isinstance(goal_quat_wxyz, str):
        goal_quat_wxyz = json.loads(goal_quat_wxyz)
    elif isinstance(goal_quat_wxyz, tuple):
        goal_quat_wxyz = list(goal_quat_wxyz)
    
    # Check ROS action server
    action_client = robot.ros_manager.get_action_client_path_planner()
    if not action_client.wait_for_server(timeout_sec=2.0):
        skill_manager.set_skill_state(skill_name, "FAILED")
        skill_manager.skill_errors[skill_name] = "Path planner server not available"
        return
    
    robot.ros_manager.get_node_controller_mpc().move_event.clear()
    
    # Create goal message
    goal_msg = ComputePathToPose.Goal()
    
    if start_pos is None:
        start_pos_tensor, _ = robot.body.get_world_pose()
        start_pos = start_pos_tensor.cpu().numpy().tolist()
    if start_quat is None:
        _, start_quat_tensor = robot.body.get_world_pose()
        start_quat = start_quat_tensor.cpu().numpy().tolist()
    
    goal_msg.start = _create_pose_stamped(robot, start_pos, start_quat)
    goal_msg.goal = _create_pose_stamped(robot, goal_pos, goal_quat_wxyz)
    
    # Store skill data
    skill_manager.set_skill_data(skill_name, "goal_pos", goal_pos)
    skill_manager.set_skill_data(skill_name, "goal_quat_wxyz", goal_quat_wxyz)
    skill_manager.set_skill_data(skill_name, "start_pos", start_pos)
    skill_manager.set_skill_data(skill_name, "start_quat", start_quat)
    
    # Send planning request (ROS action)
    action_client = robot.ros_manager.get_action_client_path_planner()
    send_future = action_client.send_goal_async(goal_msg)
    start_time = robot.sim_time
    
    skill_manager.set_skill_data(skill_name, "send_future", send_future)
    skill_manager.set_skill_data(skill_name, "start_time", start_time)
    
    # Check planning result
    result_future = skill_manager.get_skill_data(skill_name, "result_future")
    
    if result_future is None:
        if send_future.done():
            goal_handle = send_future.result()
            if goal_handle.accepted:
                result_future = goal_handle.get_result_async()
                skill_manager.set_skill_data(skill_name, "result_future", result_future)
                skill_manager.set_skill_data(skill_name, "goal_handle", goal_handle)
                return skill_manager.form_feedback("processing", "Planning path...", 15)
            else:
                skill_manager.set_skill_state(skill_name, "FAILED")
                skill_manager.skill_errors[skill_name] = "Request rejected"
                return skill_manager.form_feedback("failed", "Request rejected")
        else:
            elapsed = robot.sim_time - start_time
            if elapsed > 3.0:
                skill_manager.set_skill_state(skill_name, "FAILED")
                skill_manager.skill_errors[skill_name] = "No response from planner"
                return skill_manager.form_feedback("failed", "No response")
            return skill_manager.form_feedback("processing", "Sending request...", 5)
    
    # Check planning result
    if result_future.done():
        result = result_future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED and result.result.path.poses:
            robot.ros_manager.get_node_controller_mpc().move_event.clear()
            skill_manager.set_skill_data(skill_name, "move_start_time", robot.sim_time)
            skill_manager.set_skill_state(skill_name, "EXECUTING")
        else:
            skill_manager.set_skill_state(skill_name, "FAILED")
            skill_manager.skill_errors[skill_name] = "Planning failed"
    else:
        elapsed = robot.sim_time - start_time
        if elapsed > 15.0:
            goal_handle = skill_manager.get_skill_data(skill_name, "goal_handle")
            if goal_handle:
                goal_handle.cancel_goal_async()
            skill_manager.set_skill_state(skill_name, "FAILED")
            skill_manager.skill_errors[skill_name] = "Planning timeout"
    
    return skill_manager.form_feedback("processing", "Planning path...", 30)


def _handle_executing(robot, skill_manager, skill_name):
    """
    Handle executing state
    
    Gets velocity from MPC controller and applies via Control object.
    This decouples the skill from simulation layer.
    """
    # Check if reached goal
    if robot.ros_manager.get_node_controller_mpc().move_event.is_set():
        # Stop robot using Control object
        from simulation.control import RobotControl
        control = RobotControl()
        control.linear_velocity = [0.0, 0.0, 0.0]
        control.angular_velocity = [0.0, 0.0, 0.0]
        robot.apply_control(control)
        
        skill_manager.set_skill_state(skill_name, "COMPLETED")
        return skill_manager.form_feedback("completed", "Reached goal", 100)
    
    # Get velocity command from MPC controller
    # MPC controller computes velocity based on path
    linear_vel = robot.vel_linear
    angular_vel = robot.vel_angular
    
    # Apply velocity using Control object (Application -> Simulation layer)
    from simulation.control import RobotControl
    control = RobotControl()
    control.linear_velocity = linear_vel.tolist() if hasattr(linear_vel, 'tolist') else linear_vel
    control.angular_velocity = angular_vel.tolist() if hasattr(angular_vel, 'tolist') else angular_vel
    robot.apply_control(control)
    
    # Check timeout
    move_start_time = skill_manager.get_skill_data(skill_name, "move_start_time", 0)
    elapsed = robot.sim_time - move_start_time
    if elapsed > 120.0:
        skill_manager.set_skill_state(skill_name, "FAILED")
        skill_manager.skill_errors[skill_name] = "Navigate timeout"
        return skill_manager.form_feedback("failed", "Timeout")
    
    progress = min(50 + (elapsed / 120.0) * 45, 95)
    return skill_manager.form_feedback("processing", f"Moving... ({elapsed:.1f}s)", int(progress))


def _handle_completed(skill_manager, skill_name):
    """Handle completed state"""
    return skill_manager.form_feedback("completed", "Navigate completed", 100)


def _handle_failed(skill_manager, skill_name):
    """Handle failed state"""
    error_msg = skill_manager.skill_errors.get(skill_name, "Unknown error")
    return skill_manager.form_feedback("failed", error_msg)


def _create_pose_stamped(robot, pos: list, quat_wxyz: list):
    """Create ROS PoseStamped message"""
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
