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
            execute_callback=self.execute_callback_wrapper,
        )
        

        self.subscriber_sim_clock = self.create_subscription(
            Clock, "/isaacsim_simulation_clock", self.callback_sim_clock, 10
        )
        
        logger.info(f"ROS2 Node for {self.namespace} has been created.")

    def callback_cmd_vel(self, msg):
        linear_vel = [msg.linear.x, msg.linear.y, msg.linear.z]
        angular_vel = [msg.angular.x, msg.angular.y, msg.angular.z]
        self.robot_instance.set_velocity_command(linear_vel, angular_vel)
    def callback_sim_clock(self, msg: Clock):
        sim_time = msg.clock.sec + msg.clock.nanosec / 1e9
        self.robot_instance.update_sim_time(sim_time)

    def set_robot_instance(self, robot):
        self.robot_instance = robot

    def execute_callback_wrapper(self, goal_handle):
        """
        ROS2 action server的callback wrapper，处理skill执行请求
        """
        if self.robot_instance.skill_function is not None:
            goal_handle.abort()
            return SkillExecution.Result(success=False, message="机器人正忙")

        try:
            request = goal_handle.request.skill_request
            task_name = request.skill_list[0].skill
            params = {p.key: p.value for p in request.skill_list[0].params}
            params["robot"] = self.robot_instance

            self.prepare_skill_execution(goal_handle, task_name, params)
            
            # 发送初始feedback表示技能已启动
            initial_feedback = SkillExecution.Feedback()
            initial_feedback.status = f"技能 {task_name} 启动成功"
            goal_handle.publish_feedback(initial_feedback)
            
            # 循环监控skill执行状态并发送feedback
            import time
            
            while self.robot_instance.skill_function is not None and goal_handle.is_active:
                # 读取robot的状态信息
                feedback = SkillExecution.Feedback()
                
                # 获取技能状态
                skill_status = self.robot_instance.get_skill_status()
                status = skill_status.get("status", "processing")
                reason = skill_status.get("reason", "")
                progress = skill_status.get("progress", 0)
                feedback.status = f"{status}: {reason} ({progress}%)"
                
                # 发送feedback
                goal_handle.publish_feedback(feedback)
                
                # 10Hz频率，等待0.1秒
                time.sleep(0.1)
            
            # skill执行完成，获取结果
            skill_result = self.robot_instance.skill_result
            delattr(self.robot_instance, 'skill_result')
            
            # 创建ROS2 action result
            result = SkillExecution.Result(
                success=skill_result["success"], 
                message=skill_result["message"]
            )
            
            # 根据结果调用succeed或abort
            if skill_result["success"]:
                goal_handle.succeed()
                return result
            else:
                goal_handle.abort()
                return result
            
        except Exception as e:
            goal_handle.abort()
            return SkillExecution.Result(success=False, message=f"启动失败: {repr(e)}")

    def prepare_skill_execution(self, goal_handle, task_name, params) -> None:
        from robot.skill.base.navigation.navigate_to import navigate_to_skill
        from robot.skill.base.manipulation.pick_up import pick_up_skill
        from robot.skill.base.manipulation.put_down import put_down_skill
        from robot.skill.base.take_photo import take_photo

        SKILL_TABLE = {
            "navigation": navigate_to_skill,
            "pickup": pick_up_skill,
            "putdown": put_down_skill,
            # "take_photo": take_photo,
        }

        self.robot_instance.skill_function = SKILL_TABLE[task_name]
        self.robot_instance.skill_params = params
        # 不再设置active_goal_handle和skill_feedback_msg，保持skill执行的纯净性
