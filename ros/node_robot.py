# =============================================================================
# Node Robot Module - Individual Robot ROS2 Communication
# =============================================================================
#
# This module provides ROS2 node implementation for managing all ROS2
# communication for individual robot entities, including odometry publishing
# and command velocity subscription.
#
# =============================================================================

# Standard library imports

# Local project imports
from log.log_manager import LogManager

# ROS2 imports
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# Custom ROS message imports
from gsi_msgs.gsi_msgs_helper import SkillExecution

from rosgraph_msgs.msg import Clock

logger = LogManager.get_logger(__name__)


class NodeRobot(Node):
    """
    为单个机器人实体管理所有ROS2通信的节点。
    """

    def __init__(self, namespace: str):
        super().__init__(node_name=f"node_{namespace}", namespace=namespace)
        self.namespace = namespace
        self.robot_instance = None

        self.publisher_odom = self.create_publisher(Odometry, "odom", 10)
        self.subscriber_cmd_vel = self.create_subscription(
            Twist, "cmd_vel", self.callback_cmd_vel, 10
        )
        self.action_server_skill = ActionServer(
            self,
            action_type=SkillExecution,
            action_name=f"skill_execution",
            execute_callback=self.callback_execute_skill,
        )
        
        # 订阅Isaac Sim仿真时钟
        self.subscriber_sim_clock = self.create_subscription(
            Clock, "/isaacsim_simulation_clock", self.callback_sim_clock, 10
        )
        
        logger.info(f"ROS2 Node for {self.namespace} has been created.")

    def callback_cmd_vel(self, msg):
        if self.robot_instance and hasattr(self.robot_instance, "callback_cmd_vel"):
            self.robot_instance.callback_cmd_vel(msg)
        else:
            self.get_logger().warning("No robot instance connected for cmd_vel")

    def callback_sim_clock(self, msg: Clock):
        self.robot_instance.sim_time = msg.clock.sec + msg.clock.nanosec / 1e9

    def set_robot_instance(self, robot):
        self.robot_instance = robot

    def callback_execute_skill(self, goal_handle):

        if self.robot_instance.active_goal_handle:
            goal_handle.abort()
            return SkillExecution.Result(success=False, message="机器人正忙")

        try:
            request = goal_handle.request.skill_request
            task_name = request.skill_list[0].skill
            params = {p.key: p.value for p in request.skill_list[0].params}
            params["robot"] = self.robot_instance

            self.prepare_skill_execution(goal_handle, task_name, params)
            return SkillExecution.Result(success=True, message=f"技能 {task_name} 启动成功")
        except Exception as e:
            goal_handle.abort()
            return SkillExecution.Result(success=False, message=f"启动失败: {str(e)}")

    def prepare_skill_execution(self, goal_handle, task_name, params) -> None:
        """准备技能执行，在 physics step 中执行"""
        from robot.skill.base.navigation.navigate_to import navigate_to_skill
        from robot.skill.base.manipulation.pick_up import pick_up_skill
        from robot.skill.base.manipulation.put_down import put_down_skill

        SKILL_TABLE = {
            "navigation": navigate_to_skill,
            "pickup": pick_up_skill,
            "putdown": put_down_skill,
        }

        try:
            self.robot_instance.skill_generator = SKILL_TABLE[task_name](**params)
            self.robot_instance.active_goal_handle = goal_handle
            self.robot_instance.skill_feedback_msg = SkillExecution.Feedback()
        except Exception as e:
            raise e
