# Local project imports
from log.log_manager import LogManager

# ROS2 imports
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# ROS message imports
from gsi_msgs.gsi_msgs_helper import SkillExecution

from rosgraph_msgs.msg import Clock

logger = LogManager.get_logger(__name__)


class NodeRobot(Node):
    """
    为单个机器人实体管理所有ROS2通信的节点。
    """

    def __init__(self, namespace: str, topics: dict = None):
        super().__init__(node_name=f"node_{namespace}", namespace=namespace)
        self.namespace = namespace
        self.robot_instance = None
        self.topics = topics or {}

        # 创建publishers和subscribers
        self._create_publishers()
        self._create_subscribers()
        self._create_action_servers()

        logger.info(
            f"ROS2 Node for {self.namespace} has been created with topics: {list(self.topics.keys())}"
        )

    def _create_publishers(self):
        """根据配置创建publishers"""
        self.publisher_dict = {}

        if "odom" in self.topics:
            self.publisher_odom = self.create_publisher(
                Odometry, self.topics["odom"], 10
            )
            self.publisher_dict["odom"] = self.publisher_odom

    def _create_subscribers(self):
        """根据配置创建subscribers"""
        self.subscriber_dict = {}

        # if "cmd_vel" in self.topics:
        #     self.subscriber_cmd_vel = self.create_subscription(
        #         Twist, self.topics["cmd_vel"], self.callback_cmd_vel, 10
        #     )
        #     self.subscriber_dict["cmd_vel"] = self.subscriber_cmd_vel

        self.subscriber_sim_clock = self.create_subscription(
            Clock, "/isaacsim_simulation_clock", self.callback_sim_clock, 10
        )
        self.subscriber_dict["sim_clock"] = self.subscriber_sim_clock

    def _create_action_servers(self):
        """创建action servers"""
        self.action_server_dict = {}

        self.action_server_skill = ActionServer(
            self,
            action_type=SkillExecution,
            action_name=f"skill_execution",
            execute_callback=self.execute_callback_wrapper,
        )
        self.action_server_dict["skill_execution"] = self.action_server_skill

    # def callback_cmd_vel(self, msg):
    #     linear_vel = [msg.linear.x, msg.linear.y, msg.linear.z]
    #     angular_vel = [msg.angular.x, msg.angular.y, msg.angular.z]
    #     self.robot_instance.set_velocity_command(linear_vel, angular_vel)

    def callback_sim_clock(self, msg: Clock):
        sim_time = msg.clock.sec + msg.clock.nanosec / 1e9
        self.robot_instance.update_sim_time(sim_time)

    def set_robot_instance(self, robot):
        self.robot_instance = robot

    def execute_callback_wrapper(self, goal_handle):
        """
        ROS2 action server的callback wrapper，处理skill执行请求
        使用新的 SkillManager 架构
        """
        import json
        import time

        request = goal_handle.request.skill_request
        task_name = request.skill_list[0].skill
        
        # Parse parameters - convert string values to appropriate types
        params = {}
        for p in request.skill_list[0].params:
            try:
                # Try to parse as JSON first
                params[p.key] = json.loads(p.value)
            except (json.JSONDecodeError, ValueError):
                # If not JSON, use as string
                params[p.key] = p.value

        # Get skill manager from robot
        if not hasattr(self.robot_instance, 'skill_manager'):
            goal_handle.abort()
            return SkillExecution.Result(
                success=False, 
                message=f"Robot does not have skill_manager"
            )
        
        skill_manager = self.robot_instance.skill_manager
        
        # Check if skill is registered
        if task_name not in skill_manager.skills:
            goal_handle.abort()
            available_skills = list(skill_manager.skills.keys())
            return SkillExecution.Result(
                success=False, 
                message=f"Unknown skill: {task_name}. Available: {available_skills}"
            )

        # Send initial feedback
        feedback = SkillExecution.Feedback()
        feedback.status = f"Skill {task_name} started"
        goal_handle.publish_feedback(feedback)

        # Execute skill in loop until completion
        while goal_handle.is_active:
            # Execute skill
            result = skill_manager.execute_skill(task_name, **params)
            
            status = result.get("status", "processing")
            message = result.get("message", "")
            progress = result.get("progress", 0)

            # Send feedback
            feedback.status = f"{status}: {message} ({progress}%)"
            goal_handle.publish_feedback(feedback)

            # Check if skill is complete
            if status in ["completed", "failed"]:
                break

            # 10Hz frequency
            time.sleep(0.1)

        # Get final result
        final_result = skill_manager.execute_skill(task_name, **params)
        final_status = final_result.get("status", "failed")
        final_message = final_result.get("message", "Unknown result")

        # Create ROS2 action result
        success = final_status == "completed"
        result = SkillExecution.Result(success=success, message=final_message)

        if success:
            goal_handle.succeed()
            return result
        else:
            goal_handle.abort()
            return result
