"""
Robot ROS Manager - Manages all ROS infrastructure for a robot

This class encapsulates all ROS-related functionality that was previously
in the Robot class, achieving complete decoupling between simulation layer
and ROS layer.
"""

import threading
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import Odometry

from ros.node_robot import NodeRobot
from robot.skill.base.navigation import NodePlannerOmpl, NodeTrajectoryGenerator, NodeMpcController
from log.log_manager import LogManager

logger = LogManager.get_logger(__name__)


class RobotRosManager:
    """
    Manages all ROS infrastructure for a single robot
    
    Responsibilities:
    - ROS node management
    - Action clients
    - Navigation nodes (Planner, Trajectory, MPC)
    - Executor and threading
    - Publishers and subscribers
    """
    
    def __init__(self, robot, namespace: str, topics: dict):
        """
        Initialize ROS manager for a robot
        
        Args:
            robot: Robot instance (simulation layer)
            namespace: Robot namespace
            topics: ROS topics configuration
        """
        self.robot = robot
        self.namespace = namespace
        self.topics = topics
        
        # ROS nodes
        self.node = None
        self.node_planner_ompl = None
        self.node_trajectory_generator = None
        self.node_controller_mpc = None
        
        # Action clients
        self.action_client_path_planner = None
        
        # Executor and threading
        self.executor = None
        self.ros_thread = None
        self.stop_event = threading.Event()
        
        # Initialize ROS infrastructure
        self._init_ros_nodes()
        self._init_action_clients()
        self._init_executor()
    
    def _init_ros_nodes(self):
        """Initialize all ROS nodes"""
        # Main robot node
        self.node = NodeRobot(namespace=self.namespace, topics=self.topics)
        self.node.set_robot_instance(self.robot)
        
        # Navigation nodes
        self.node_planner_ompl = NodePlannerOmpl(namespace=self.namespace)
        self.node_trajectory_generator = NodeTrajectoryGenerator(namespace=self.namespace)
        self.node_controller_mpc = NodeMpcController(namespace=self.namespace)
        
        logger.info(f"ROS nodes initialized for {self.namespace}")
    
    def _init_action_clients(self):
        """Initialize action clients"""
        self.action_client_path_planner = ActionClient(
            self.node,
            ComputePathToPose,
            "action_compute_path_to_pose"
        )
        logger.info(f"Action clients initialized for {self.namespace}")
    
    def _init_executor(self):
        """Initialize executor and add all nodes"""
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.node)
        self.executor.add_node(self.node_planner_ompl)
        self.executor.add_node(self.node_trajectory_generator)
        self.executor.add_node(self.node_controller_mpc)
        logger.info(f"Executor initialized for {self.namespace}")
    
    def start(self):
        """Start ROS thread"""
        if self.ros_thread is None or not self.ros_thread.is_alive():
            self.stop_event.clear()
            self.ros_thread = threading.Thread(
                target=self._spin_ros,
                daemon=True,
                name=f"ROS_{self.namespace}"
            )
            self.ros_thread.start()
            logger.info(f"ROS thread started for {self.namespace}")
            
            # Give ROS time to initialize
            import time
            time.sleep(0.1)
    
    def _spin_ros(self):
        """ROS spinning loop (runs in separate thread)"""
        logger.info(f"ROS thread spinning for {self.namespace}")
        try:
            while not self.stop_event.is_set():
                self.executor.spin_once(timeout_sec=0.05)
        except Exception as e:
            logger.error(f"ROS thread error for {self.namespace}: {e}")
        finally:
            logger.info(f"ROS thread stopped for {self.namespace}")
    
    def stop(self):
        """Stop ROS thread and cleanup"""
        if self.ros_thread and self.ros_thread.is_alive():
            logger.info(f"Stopping ROS thread for {self.namespace}")
            self.stop_event.set()
            self.ros_thread.join(timeout=2.0)
            
            if self.ros_thread.is_alive():
                logger.warning(f"ROS thread did not stop gracefully for {self.namespace}")
    
    def publish_odometry(self, pos, quat, vel_linear, vel_angular):
        """
        Publish odometry message
        
        Args:
            pos: Position [x, y, z]
            quat: Quaternion [x, y, z, w]
            vel_linear: Linear velocity [x, y, z]
            vel_angular: Angular velocity [x, y, z]
        """
        odom_msg = Odometry()
        odom_msg.header.stamp = self.node.get_clock().now().to_msg()
        odom_msg.header.frame_id = "map"
        odom_msg.child_frame_id = self.namespace
        
        # Position
        odom_msg.pose.pose.position.x = float(pos[0])
        odom_msg.pose.pose.position.y = float(pos[1])
        odom_msg.pose.pose.position.z = float(pos[2])
        
        # Orientation (convert from wxyz to xyzw)
        odom_msg.pose.pose.orientation.x = float(quat[1])
        odom_msg.pose.pose.orientation.y = float(quat[2])
        odom_msg.pose.pose.orientation.z = float(quat[3])
        odom_msg.pose.pose.orientation.w = float(quat[0])
        
        # Velocity
        odom_msg.twist.twist.linear.x = float(vel_linear[0])
        odom_msg.twist.twist.linear.y = float(vel_linear[1])
        odom_msg.twist.twist.linear.z = float(vel_linear[2])
        odom_msg.twist.twist.angular.x = float(vel_angular[0])
        odom_msg.twist.twist.angular.y = float(vel_angular[1])
        odom_msg.twist.twist.angular.z = float(vel_angular[2])
        
        self.node.publisher_dict["odom"].publish(odom_msg)
    
    def get_node(self):
        """Get main ROS node"""
        return self.node
    
    def get_action_client_path_planner(self):
        """Get path planner action client"""
        return self.action_client_path_planner
    
    def get_node_planner_ompl(self):
        """Get OMPL planner node"""
        return self.node_planner_ompl
    
    def get_node_trajectory_generator(self):
        """Get trajectory generator node"""
        return self.node_trajectory_generator
    
    def get_node_controller_mpc(self):
        """Get MPC controller node"""
        return self.node_controller_mpc
    
    def __del__(self):
        """Cleanup on deletion"""
        self.stop()
