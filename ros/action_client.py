#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from plan_msgs.action import ExecutePlan, ExecuteSkill
from gsi_msgs.gsi_msgs_helper import Plan, SkillInfo
from log.log_manager import LogManager

logger = LogManager.get_logger(__name__)


class PlanActionClient(Node):
    """计划执行Action客户端"""
    
    def __init__(self):
        super().__init__('plan_action_client')
        
        self._action_client = ActionClient(self, ExecutePlan, 'execute_plan')
        self.current_goal_handle = None
        
        logger.info("Plan Action Client initialized")

    def send_plan(self, plan: Plan, robot_id: str, feedback_callback=None):
        """发送计划执行请求"""
        
        # 等待服务器可用
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            logger.error("Plan action server not available")
            return None
        
        # 创建目标
        goal_msg = ExecutePlan.Goal()
        goal_msg.plan = plan
        goal_msg.robot_id = robot_id
        
        logger.info(f"Sending plan execution request for robot: {robot_id}")
        
        # 发送目标
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=feedback_callback or self._default_feedback_callback
        )
        
        # 添加回调处理目标响应
        send_goal_future.add_done_callback(self._goal_response_callback)
        
        return send_goal_future

    def _goal_response_callback(self, future):
        """处理目标响应"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            logger.warning("Plan execution goal rejected")
            return
        
        logger.info("Plan execution goal accepted")
        self.current_goal_handle = goal_handle
        
        # 等待结果
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future):
        """处理执行结果"""
        result = future.result().result
        
        if result.success:
            logger.info(f"Plan execution completed successfully: {result.message}")
        else:
            logger.error(f"Plan execution failed: {result.message}")
        
        self.current_goal_handle = None

    def _default_feedback_callback(self, feedback_msg):
        """默认反馈处理"""
        feedback = feedback_msg.feedback
        logger.info(
            f"Plan progress: {feedback.current_timestep}/{feedback.total_timesteps} "
            f"({feedback.progress_percentage:.1f}%) - Status: {feedback.status}"
        )
        
        if hasattr(feedback, 'current_skill') and feedback.current_skill:
            logger.info(f"Current skill: {feedback.current_skill.skill}")

    def cancel_current_goal(self):
        """取消当前执行的目标"""
        if self.current_goal_handle:
            logger.info("Canceling current plan execution")
            cancel_future = self.current_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self._cancel_callback)
            return cancel_future
        else:
            logger.warning("No active goal to cancel")
            return None

    def _cancel_callback(self, future):
        """处理取消响应"""
        cancel_response = future.result()
        if cancel_response.goals_canceling:
            logger.info("Plan execution canceled successfully")
        else:
            logger.warning("Failed to cancel plan execution")


class SkillActionClient(Node):
    """单个技能执行Action客户端"""
    
    def __init__(self):
        super().__init__('skill_action_client')
        
        self._action_client = ActionClient(self, ExecuteSkill, 'execute_skill')
        
        logger.info("Skill Action Client initialized")

    def send_skill(self, skill: SkillInfo, robot_id: str, feedback_callback=None):
        """发送技能执行请求"""
        
        # 等待服务器可用
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            logger.error("Skill action server not available")
            return None
        
        # 创建目标
        goal_msg = ExecuteSkill.Goal()
        goal_msg.skill = skill
        goal_msg.robot_id = robot_id
        
        logger.info(f"Sending skill execution request: {skill.skill} for robot: {robot_id}")
        
        # 发送目标
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=feedback_callback or self._default_feedback_callback
        )
        
        send_goal_future.add_done_callback(self._goal_response_callback)
        return send_goal_future

    def _goal_response_callback(self, future):
        """处理目标响应"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            logger.warning("Skill execution goal rejected")
            return
        
        logger.info("Skill execution goal accepted")
        
        # 等待结果
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future):
        """处理执行结果"""
        result = future.result().result
        
        if result.success:
            logger.info(f"Skill execution completed in {result.execution_time:.2f}s: {result.message}")
        else:
            logger.error(f"Skill execution failed: {result.message}")

    def _default_feedback_callback(self, feedback_msg):
        """默认反馈处理"""
        feedback = feedback_msg.feedback
        logger.info(
            f"Skill progress: {feedback.progress_percentage:.1f}% - "
            f"Status: {feedback.status} - Phase: {feedback.current_phase}"
        )


# 便利函数
def create_plan_client():
    """创建计划客户端的便利函数"""
    return PlanActionClient()


def create_skill_client():
    """创建技能客户端的便利函数"""
    return SkillActionClient()


# 示例使用
def example_usage():
    """示例用法"""
    rclpy.init()
    
    try:
        # 创建客户端
        plan_client = create_plan_client()
        
        # 创建示例计划
        from gsi_msgs.gsi_msgs_helper import Plan, TimestepSkills, RobotSkill, SkillInfo, Parameter
        
        # 创建技能
        skill = SkillInfo()
        skill.skill = "move_to"
        skill.object_id = "target_1"
        skill.task_id = "delivery_task"
        
        param = Parameter()
        param.key = "x"
        param.value = "10.0"
        skill.params = [param]
        
        # 创建机器人技能
        robot_skill = RobotSkill()
        robot_skill.robot_id = "robot_1"
        robot_skill.skill_list = [skill]
        
        # 创建时间步
        timestep = TimestepSkills()
        timestep.timestep = 0
        timestep.robots = [robot_skill]
        
        # 创建计划
        plan = Plan()
        plan.steps = [timestep]
        
        # 发送计划
        future = plan_client.send_plan(plan, "robot_1")
        
        # 等待完成
        rclpy.spin(plan_client)
        
    except KeyboardInterrupt:
        logger.info("Client shutting down")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    example_usage()