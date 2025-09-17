#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from plan_msgs.action import ExecutePlan, ExecuteSkill
from gsi_msgs.gsi_msgs_helper import RobotFeedback, SkillInfo
from log.log_manager import LogManager

logger = LogManager.get_logger(__name__)


class PlanActionServer(Node):
    """计划执行Action服务器"""
    
    def __init__(self):
        super().__init__('plan_action_server')
        
        self._action_server = ActionServer(
            self,
            ExecutePlan,
            'execute_plan',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup()
        )
        
        # 当前执行状态
        self.current_goals = {}  # goal_handle -> execution_state
        
        logger.info("Plan Action Server initialized")

    def goal_callback(self, goal_request):
        """处理新的目标请求"""
        logger.info(f"Received plan execution request for robot: {goal_request.robot_id}")
        
        # 检查计划是否有效
        if not goal_request.plan or not goal_request.plan.steps:
            logger.warning("Received empty plan")
            return GoalResponse.REJECT
            
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """处理取消请求"""
        logger.info(f"Received cancel request for goal: {goal_handle.goal_id}")
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """执行计划的主要逻辑"""
        logger.info(f"Executing plan for robot: {goal_handle.request.robot_id}")
        
        plan = goal_handle.request.plan
        robot_id = goal_handle.request.robot_id
        
        # 存储执行状态
        self.current_goals[goal_handle] = {
            'robot_id': robot_id,
            'plan': plan,
            'current_timestep': 0,
            'total_timesteps': len(plan.steps)
        }
        
        try:
            # 执行计划中的每个时间步
            for timestep_idx, timestep in enumerate(plan.steps):
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    logger.info(f"Plan execution canceled for robot: {robot_id}")
                    return ExecutePlan.Result(success=False, message="Canceled by user")
                
                # 更新当前时间步
                self.current_goals[goal_handle]['current_timestep'] = timestep_idx
                
                # 执行当前时间步的技能
                success = await self._execute_timestep(goal_handle, timestep, timestep_idx)
                
                if not success:
                    result = ExecutePlan.Result(
                        success=False, 
                        message=f"Failed at timestep {timestep_idx}"
                    )
                    goal_handle.abort()
                    return result
                
                # 发送进度反馈
                feedback = ExecutePlan.Feedback()
                feedback.current_timestep = timestep_idx + 1
                feedback.total_timesteps = len(plan.steps)
                feedback.progress_percentage = (timestep_idx + 1) / len(plan.steps) * 100.0
                feedback.status = "executing"
                
                goal_handle.publish_feedback(feedback)
            
            # 执行成功
            result = ExecutePlan.Result(
                success=True,
                message="Plan executed successfully"
            )
            goal_handle.succeed()
            logger.info(f"Plan execution completed for robot: {robot_id}")
            return result
            
        except Exception as e:
            logger.error(f"Error executing plan: {e}")
            result = ExecutePlan.Result(success=False, message=str(e))
            goal_handle.abort()
            return result
        finally:
            # 清理执行状态
            if goal_handle in self.current_goals:
                del self.current_goals[goal_handle]

    async def _execute_timestep(self, goal_handle, timestep, timestep_idx):
        """执行单个时间步"""
        robot_id = goal_handle.request.robot_id
        
        # 找到当前机器人在这个时间步的技能
        robot_skills = None
        for robot_skill in timestep.robots:
            if robot_skill.robot_id == robot_id:
                robot_skills = robot_skill.skill_list
                break
        
        if not robot_skills:
            logger.warning(f"No skills found for robot {robot_id} at timestep {timestep_idx}")
            return True  # 没有技能也算成功
        
        # 执行所有技能
        for skill in robot_skills:
            success = await self._execute_skill(goal_handle, skill)
            if not success:
                return False
                
        return True

    async def _execute_skill(self, goal_handle, skill):
        """执行单个技能"""
        # 这里应该调用实际的技能执行逻辑
        # 目前作为示例，我们模拟执行过程
        
        logger.info(f"Executing skill: {skill.skill} for robot: {goal_handle.request.robot_id}")
        
        # 发送技能级别的反馈
        feedback = ExecutePlan.Feedback()
        feedback.current_skill = skill
        feedback.status = "executing"
        goal_handle.publish_feedback(feedback)
        
        # 模拟技能执行时间
        import asyncio
        await asyncio.sleep(1.0)  # 实际应该调用技能执行器
        
        # 这里应该集成你的技能执行系统
        # 例如调用 skill.skill 模块中的相应函数
        
        return True  # 假设执行成功


class SkillActionServer(Node):
    """单个技能执行Action服务器"""
    
    def __init__(self):
        super().__init__('skill_action_server')
        
        self._action_server = ActionServer(
            self,
            ExecuteSkill,
            'execute_skill',
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup()
        )
        
        logger.info("Skill Action Server initialized")

    async def execute_callback(self, goal_handle):
        """执行单个技能"""
        skill = goal_handle.request.skill
        robot_id = goal_handle.request.robot_id
        
        logger.info(f"Executing skill: {skill.skill} for robot: {robot_id}")
        
        try:
            # 发送开始反馈
            feedback = ExecuteSkill.Feedback()
            feedback.status = "starting"
            feedback.progress_percentage = 0.0
            goal_handle.publish_feedback(feedback)
            
            # 模拟技能执行过程
            import asyncio
            for progress in [25, 50, 75, 100]:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return ExecuteSkill.Result(success=False, message="Canceled")
                
                await asyncio.sleep(0.5)  # 模拟执行时间
                
                feedback.progress_percentage = float(progress)
                feedback.status = "executing" if progress < 100 else "completed"
                goal_handle.publish_feedback(feedback)
            
            # 执行成功
            result = ExecuteSkill.Result(
                success=True,
                message="Skill executed successfully",
                execution_time=2.0
            )
            goal_handle.succeed()
            return result
            
        except Exception as e:
            logger.error(f"Error executing skill: {e}")
            result = ExecuteSkill.Result(success=False, message=str(e))
            goal_handle.abort()
            return result


def main():
    rclpy.init()
    
    try:
        # 创建服务器节点
        plan_server = PlanActionServer()
        skill_server = SkillActionServer()
        
        # 使用多线程执行器
        executor = MultiThreadedExecutor()
        executor.add_node(plan_server)
        executor.add_node(skill_server)
        
        logger.info("Action servers started")
        executor.spin()
        
    except KeyboardInterrupt:
        logger.info("Action servers shutting down")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()