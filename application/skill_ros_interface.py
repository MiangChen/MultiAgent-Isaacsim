"""
Skill ROS Interface - Application Layer

Provides ROS Action Server interface for SkillManager.
Each robot has its own SkillROSInterface instance.

Architecture:
    - Application Layer component (not ROS Layer)
    - Wraps SkillManager with ROS Action Server
    - Independent ROS node and executor
    - Runs in separate thread
    - Supports concurrent execution for multiple robots
    - Subscribes to sim_clock for skill timing
    - Manages navigation nodes (planner, trajectory, MPC)

Usage:
    # In main.py (Application Layer)
    skill_manager = SkillManager(robot, auto_register=True)
    skill_ros = SkillROSInterface(
        robot=robot,
        skill_manager=skill_manager,
        namespace="jetbot_0"
    )
    skill_ros.start()
    
    # ROS command
    ros2 action send_goal /jetbot_0/skill_execution plan_msgs/action/SkillExecution \
        "{skill: navigate_to, params: [{key: goal_pos, value: '[3, 3, 0]'}]}" --feedback
"""

import threading
import time
import json
from typing import Optional, Tuple, List

# ROS2 imports
import rclpy
from rclpy.action import ActionServer, ActionClient, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from nav2_msgs.action import ComputePathToPose

# Local imports
from log.log_manager import LogManager
from ros_msg.gsi_msgs_helper import SkillExecution
from application.skills.base.navigation import (
    NodePlannerOmpl2D,
    NodePlannerOmpl3D,
    NodeTrajectoryGenerator,
    NodeMpcController,
)

logger = LogManager.get_logger(__name__)


class SkillROSInterface:
    """
    Skill ROS Interface - Provides ROS Action Server for SkillManager
    
    Responsibilities:
        1. Create ROS Action Server at /{namespace}/skill_execution
        2. Receive and parse ROS action requests
        3. Call SkillManager to execute skills
        4. Publish real-time feedback to ROS
        5. Return final result
    
    Architecture:
        - One instance per robot
        - Independent ROS node (not reusing NodeRobot)
        - Independent thread (not blocking simulation)
        - Supports concurrent execution (multiple robots)
    """

    def __init__(self, robot, skill_manager, namespace: str):
        """
        Initialize Skill ROS Interface
        
        Args:
            robot: Robot instance (for accessing state)
            skill_manager: SkillManager instance (for executing skills)
            namespace: Robot namespace (e.g., "jetbot_0")
        """
        self.robot = robot
        self.skill_manager = skill_manager
        self.namespace = namespace

        # ROS components
        self._node: Optional[Node] = None
        self._action_server: Optional[ActionServer] = None
        self._executor: Optional[MultiThreadedExecutor] = None
        self._ros_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()

        # Navigation nodes (Application Layer)
        self.node_planner_ompl_2d: Optional[NodePlannerOmpl2D] = None
        self.node_planner_ompl_3d: Optional[NodePlannerOmpl3D] = None
        self.node_trajectory_generator: Optional[NodeTrajectoryGenerator] = None
        self.node_controller_mpc: Optional[NodeMpcController] = None

        # Action clients for path planning
        self.action_client_path_planner_2d: Optional[ActionClient] = None
        self.action_client_path_planner_3d: Optional[ActionClient] = None

        # Initialize ROS node and action server
        self._init_ros_components()
        self._init_navigation_nodes()
        self._init_action_clients()

        # Set reference in skill_manager for skills to access navigation nodes
        if self.skill_manager is not None:
            self.skill_manager.skill_ros_interface = self

        logger.info(f"SkillROSInterface initialized for {namespace}")

    def _init_ros_components(self):
        """Initialize ROS node, action server, and sim_clock subscriber"""
        # Create independent ROS node
        self._node = Node(
            node_name=f"skill_interface_{self.namespace}",
            namespace=self.namespace
        )

        # Create action server with reentrant callback group for concurrency
        callback_group = ReentrantCallbackGroup()
        self._action_server = ActionServer(
            self._node,
            SkillExecution,
            'skill_execution',
            execute_callback=self._action_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=callback_group
        )

        # Subscribe to simulation clock (for skill timing)
        self._sim_clock_sub = self._node.create_subscription(
            Clock,
            '/isaacsim_simulation_clock',
            self._callback_sim_clock,
            10
        )

        # Subscribe to odometry (for robot pose)
        self._current_position: List[float] = [0.0, 0.0, 0.0]
        self._current_quat_wxyz: List[float] = [1.0, 0.0, 0.0, 0.0]
        self._odom_received = False

        self._odom_sub = self._node.create_subscription(
            Odometry,
            'odom',
            self._callback_odom,
            10
        )

        logger.info(
            f"Action server created: /{self.namespace}/skill_execution"
        )

    def _init_navigation_nodes(self):
        """Initialize navigation nodes (Application Layer)"""
        self.node_planner_ompl_2d = NodePlannerOmpl2D(namespace=self.namespace)
        self.node_planner_ompl_3d = NodePlannerOmpl3D(namespace=self.namespace)
        self.node_trajectory_generator = NodeTrajectoryGenerator(
            namespace=self.namespace
        )
        self.node_controller_mpc = NodeMpcController(namespace=self.namespace)

        logger.info(f"Navigation nodes initialized for {self.namespace}")

    def _init_action_clients(self):
        """Initialize action clients for path planning"""
        # 2D planner (service-based map, z=0 layer only)
        self.action_client_path_planner_2d = ActionClient(
            self._node, ComputePathToPose, "action_compute_path_to_pose_2d"
        )

        # 3D planner (service-based map, full 3D)
        self.action_client_path_planner_3d = ActionClient(
            self._node, ComputePathToPose, "action_compute_path_to_pose_3d"
        )

        logger.info(f"Action clients initialized for {self.namespace}")

    def _callback_sim_clock(self, msg: Clock):
        """Update skill_manager's simulation time from sim_clock topic"""
        sim_time = msg.clock.sec + msg.clock.nanosec / 1e9
        if self.skill_manager is not None:
            self.skill_manager.sim_time = sim_time

    def _callback_odom(self, msg: Odometry):
        """Update current robot pose from odometry topic"""
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        self._current_position = [pos.x, pos.y, pos.z]
        # Convert from xyzw to wxyz
        self._current_quat_wxyz = [ori.w, ori.x, ori.y, ori.z]
        self._odom_received = True

    def get_current_pose(self) -> Tuple[List[float], List[float]]:
        """Get current robot pose from odom topic"""
        return self._current_position.copy(), self._current_quat_wxyz.copy()

    def start(self):
        """Start ROS executor in background thread"""
        if self._ros_thread is not None:
            logger.warning(
                f"SkillROSInterface already started for {self.namespace}"
            )
            return

        # Create executor and add all nodes
        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self._node)
        self._executor.add_node(self.node_planner_ompl_2d)
        self._executor.add_node(self.node_planner_ompl_3d)
        self._executor.add_node(self.node_trajectory_generator)
        self._executor.add_node(self.node_controller_mpc)

        # Start thread
        self._ros_thread = threading.Thread(
            target=self._spin_executor,
            daemon=True,
            name=f"SkillROSInterface_{self.namespace}"
        )
        self._ros_thread.start()

        logger.info(f"SkillROSInterface started for {self.namespace}")

    def stop(self):
        """Stop ROS executor and cleanup"""
        logger.info(f"Stopping SkillROSInterface for {self.namespace}")

        self._stop_event.set()

        if self._executor:
            self._executor.shutdown()

        if self._ros_thread:
            self._ros_thread.join(timeout=2.0)

        if self._node:
            self._node.destroy_node()

        logger.info(f"SkillROSInterface stopped for {self.namespace}")

    def _spin_executor(self):
        """Spin ROS executor (runs in background thread)"""
        logger.info(f"ROS executor spinning for {self.namespace}")

        while not self._stop_event.is_set() and rclpy.ok():
            try:
                self._executor.spin_once(timeout_sec=0.1)
            except Exception as e:
                logger.error(
                    f"Error in executor spin for {self.namespace}: {e}",
                    exc_info=True
                )

        logger.info(f"ROS executor stopped for {self.namespace}")

    def _goal_callback(self, goal_request):
        """
        Handle incoming goal requests
        
        Args:
            goal_request: ROS action goal request
        
        Returns:
            GoalResponse.ACCEPT or GoalResponse.REJECT
        """
        skill_name = goal_request.skill

        # Check if skill is registered
        if skill_name not in self.skill_manager.skills:
            available_skills = self.skill_manager.list_skills()
            logger.warning(
                f"Rejected skill request: {skill_name} "
                f"(not registered). Available: {available_skills}"
            )
            return GoalResponse.REJECT

        logger.info(
            f"Accepted skill request: {skill_name} for {self.namespace}"
        )
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        """
        Handle cancel requests
        
        Args:
            goal_handle: ROS action goal handle
        
        Returns:
            CancelResponse.ACCEPT
        """
        logger.info(
            f"Cancel requested for goal: {goal_handle.goal_id} "
            f"on {self.namespace}"
        )
        return CancelResponse.ACCEPT

    def _action_callback(self, goal_handle):
        """
        Main action callback - executes skill in loop
        
        This runs in a separate thread (ROS action server thread)
        and continuously calls skill_manager.execute_skill() until completion.
        
        Args:
            goal_handle: ROS action goal handle
        
        Returns:
            SkillExecution.Result
        """
        # Parse request
        skill_name = goal_handle.request.skill
        params = self._parse_params(goal_handle.request.params)

        logger.info(
            f"Executing skill: {skill_name} on {self.namespace} "
            f"with params: {params}"
        )

        # Send initial feedback
        feedback = SkillExecution.Feedback()
        feedback.status = f"Skill {skill_name} started"
        goal_handle.publish_feedback(feedback)

        # Reset skill state before starting (in case it was completed before)
        self.skill_manager.reset_skill(skill_name)

        # Execute skill loop (10Hz)
        result = None
        while goal_handle.is_active and not self._stop_event.is_set():
            # Execute skill state machine
            result = self.skill_manager.execute_skill(skill_name, **params)

            status = result.get("status", "processing")
            message = result.get("message", "")
            progress = result.get("progress", 0)

            # Publish feedback
            feedback.status = f"{status}: {message} ({progress}%)"
            goal_handle.publish_feedback(feedback)

            # Check completion
            if status in ["completed", "failed"]:
                break

            # 10Hz loop rate
            time.sleep(0.1)

        # Return final result
        if result is None:
            result = {"status": "failed", "message": "No result"}

        # Debug: log the final result
        logger.info(f"Final result for {skill_name}: {result}")

        final_status = result.get("status", "failed")
        final_message = result.get("message", "Unknown result")
        success = (final_status == "completed")

        logger.info(
            f"Parsed result - status: '{final_status}', "
            f"message: '{final_message}', success: {success}"
        )

        if success:
            goal_handle.succeed()
            logger.info(
                f"Skill {skill_name} completed on {self.namespace}: "
                f"{final_message}"
            )
        else:
            goal_handle.abort()
            logger.warning(
                f"Skill {skill_name} failed on {self.namespace}: "
                f"{final_message}"
            )

        return SkillExecution.Result(success=success, message=final_message)

    def _parse_params(self, params_list) -> dict:
        """
        Parse ROS action parameters to Python dict
        
        Args:
            params_list: List of Parameter messages
        
        Returns:
            Dict of parsed parameters
        """
        params = {}
        for p in params_list:
            try:
                # Try to parse as JSON
                params[p.key] = json.loads(p.value)
            except (json.JSONDecodeError, ValueError):
                # Use as string if not JSON
                params[p.key] = p.value

        return params

    # =========================================================================
    # Navigation Node Getters (for skills to access)
    # =========================================================================

    def get_action_client_path_planner_2d(self):
        """Get 2D path planner action client"""
        return self.action_client_path_planner_2d

    def get_action_client_path_planner_3d(self):
        """Get 3D path planner action client"""
        return self.action_client_path_planner_3d

    def get_node_planner_ompl_2d(self):
        """Get 2D OMPL planner node"""
        return self.node_planner_ompl_2d

    def get_node_planner_ompl_3d(self):
        """Get 3D OMPL planner node"""
        return self.node_planner_ompl_3d

    def get_node_trajectory_generator(self):
        """Get trajectory generator node"""
        return self.node_trajectory_generator

    def get_node_controller_mpc(self):
        """Get MPC controller node"""
        return self.node_controller_mpc

    def __del__(self):
        """Cleanup on deletion"""
        if not self._stop_event.is_set():
            self.stop()
