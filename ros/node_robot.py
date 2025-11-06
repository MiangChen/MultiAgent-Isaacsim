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
        支持新的多技能架构
        """

        request = goal_handle.request.skill_request
        task_name = request.skill_list[0].skill
        params = {p.key: p.value for p in request.skill_list[0].params}

        # 使用新的多技能架构启动技能
        skill_function = self.get_skill_function(task_name)
        if skill_function is None:
            goal_handle.abort()
            return SkillExecution.Result(
                success=False, message=f"未知技能: {task_name}"
            )

        # 启动技能
        skill_name = self.robot_instance.start_skill(skill_function, **params)

        # 发送初始feedback表示技能已启动
        feedback = SkillExecution.Feedback()
        feedback.status = f"技能 {task_name} 启动成功"
        goal_handle.publish_feedback(feedback)

        # 循环监控skill执行状态并发送feedback
        import time

        while (
            skill_name in self.robot_instance.skill_functions and goal_handle.is_active
        ):
            # 获取技能状态
            skill_state = self.robot_instance.skill_states.get(skill_name, "UNKNOWN")
            skill_error = self.robot_instance.skill_errors.get(skill_name)
            skill_feedback = self.robot_instance.skill_feedbacks.get(skill_name, {})

            status = skill_feedback.get("status", "processing")
            message = skill_feedback.get("message", "")
            progress = skill_feedback.get("progress", 0)

            feedback.status = f"{status}: {message} ({progress}%)"
            goal_handle.publish_feedback(feedback)

            # 检查技能是否完成
            if status in ["finished", "failed", "completed"]:
                break

            # 10Hz频率，等待0.1秒 todo
            time.sleep(0.1)

        # 获取最终结果
        final_feedback = self.robot_instance.skill_feedbacks.get(skill_name, {})
        final_status = final_feedback.get("status", "failed")
        final_message = final_feedback.get("message", "Unknown result")

        # 创建ROS2 action result
        success = final_status in ["finished", "completed"]
        result = SkillExecution.Result(success=success, message=final_message)

        if success:
            goal_handle.succeed()
            return result
        else:
            goal_handle.abort()
            return result

    def get_skill_function(self, task_name):
        """获取技能函数"""
        from robot.skill.base.navigation.navigate_to import navigate_to_skill
        from robot.skill.base.manipulation.pick_up import pick_up_skill
        from robot.skill.base.manipulation.put_down import put_down_skill
        from robot.skill.base.take_photo import take_photo
        from robot.skill.base.object_detection import object_detection_skill
        from robot.skill.drone.take_off.take_off import take_off
        from robot.skill.base.exploration.explore import explore_skill
        from robot.skill.base.track import track_skill

        SKILL_TABLE = {
            "navigation": navigate_to_skill,
            "pickup": pick_up_skill,
            "putdown": put_down_skill,
            "take_photo": take_photo,
            "object_detection": object_detection_skill,
            "take_off": take_off,
            "explore": explore_skill,
            "track": track_skill,
        }

        return SKILL_TABLE.get(task_name)
