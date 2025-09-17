#!/usr/bin/env python3
"""
GSI Action系统使用示例
演示如何使用ROS2 Action来执行计划和技能
"""

import rclpy
import time
from gsi_msgs.gsi_msgs_helper import Plan, TimestepSkills, RobotSkill, SkillInfo, Parameter
from ros.ros_manager import RosManager
from log.log_manager import LogManager

logger = LogManager.get_logger(__name__)


def create_sample_plan():
    """创建示例计划"""
    
    # 创建参数
    def create_param(key, value):
        param = Parameter()
        param.key = key
        param.value = value
        return param
    
    # 创建技能1：移动到位置
    move_skill = SkillInfo()
    move_skill.skill = "move_to"
    move_skill.object_id = "target_position_1"
    move_skill.task_id = "delivery_task_1"
    move_skill.params = [
        create_param("x", "10.0"),
        create_param("y", "5.0"),
        create_param("z", "0.0")
    ]
    
    # 创建技能2：抓取物体
    grasp_skill = SkillInfo()
    grasp_skill.skill = "grasp"
    grasp_skill.object_id = "package_1"
    grasp_skill.task_id = "delivery_task_1"
    grasp_skill.params = [
        create_param("force", "10.0"),
        create_param("approach_distance", "0.1")
    ]
    
    # 创建技能3：移动到目标
    deliver_skill = SkillInfo()
    deliver_skill.skill = "move_to"
    deliver_skill.object_id = "delivery_point_1"
    deliver_skill.task_id = "delivery_task_1"
    deliver_skill.params = [
        create_param("x", "20.0"),
        create_param("y", "10.0"),
        create_param("z", "0.0")
    ]
    
    # 创建机器人技能序列
    robot_skill_1 = RobotSkill()
    robot_skill_1.robot_id = "robot_1"
    robot_skill_1.skill_list = [move_skill]
    
    robot_skill_2 = RobotSkill()
    robot_skill_2.robot_id = "robot_1"
    robot_skill_2.skill_list = [grasp_skill]
    
    robot_skill_3 = RobotSkill()
    robot_skill_3.robot_id = "robot_1"
    robot_skill_3.skill_list = [deliver_skill]
    
    # 创建时间步
    timestep_1 = TimestepSkills()
    timestep_1.timestep = 0
    timestep_1.robots = [robot_skill_1]
    
    timestep_2 = TimestepSkills()
    timestep_2.timestep = 1
    timestep_2.robots = [robot_skill_2]
    
    timestep_3 = TimestepSkills()
    timestep_3.timestep = 2
    timestep_3.robots = [robot_skill_3]
    
    # 创建完整计划
    plan = Plan()
    plan.steps = [timestep_1, timestep_2, timestep_3]
    
    return plan


def plan_feedback_callback(feedback_msg):
    """计划执行反馈回调"""
    feedback = feedback_msg.feedback
    logger.info(
        f"📊 Plan Progress: {feedback.current_timestep}/{feedback.total_timesteps} "
        f"({feedback.progress_percentage:.1f}%) - Status: {feedback.status}"
    )
    
    if hasattr(feedback, 'current_skill') and feedback.current_skill and feedback.current_skill.skill:
        logger.info(f"🔧 Current Skill: {feedback.current_skill.skill}")


def skill_feedback_callback(feedback_msg):
    """技能执行反馈回调"""
    feedback = feedback_msg.feedback
    logger.info(
        f"⚙️  Skill Progress: {feedback.progress_percentage:.1f}% - "
        f"Status: {feedback.status}"
    )


def demonstrate_action_system():
    """演示Action系统的使用"""
    
    logger.info("🚀 Starting GSI Action System Demonstration")
    
    # 初始化ROS
    rclpy.init()
    
    try:
        # 创建ROS管理器并设置为Action模式
        ros_manager = RosManager()
        ros_manager.set_mode('action')
        ros_manager.start()
        
        # 等待节点启动
        time.sleep(2)
        
        logger.info("📋 Creating sample plan...")
        plan = create_sample_plan()
        
        logger.info("📤 Sending plan execution request...")
        
        # 发送计划执行请求
        future = ros_manager.send_plan_action(
            plan=plan,
            robot_id="robot_1",
            feedback_callback=plan_feedback_callback
        )
        
        if future is None:
            logger.error("❌ Failed to send plan action")
            return
        
        logger.info("✅ Plan action sent successfully")
        
        # 等待一段时间让计划执行
        logger.info("⏳ Waiting for plan execution...")
        time.sleep(10)
        
        # 演示单个技能执行
        logger.info("\n🔧 Demonstrating individual skill execution...")
        
        # 创建单个技能
        single_skill = SkillInfo()
        single_skill.skill = "rotate"
        single_skill.object_id = "robot_1"
        single_skill.task_id = "test_task"
        single_skill.params = [
            Parameter(key="angle", value="90.0"),
            Parameter(key="speed", value="0.5")
        ]
        
        # 发送技能执行请求
        skill_future = ros_manager.send_skill_action(
            skill=single_skill,
            robot_id="robot_1",
            feedback_callback=skill_feedback_callback
        )
        
        if skill_future:
            logger.info("✅ Skill action sent successfully")
            time.sleep(5)
        
        # 演示取消功能
        logger.info("\n🛑 Demonstrating plan cancellation...")
        
        # 发送另一个计划
        cancel_future = ros_manager.send_plan_action(
            plan=plan,
            robot_id="robot_1",
            feedback_callback=plan_feedback_callback
        )
        
        if cancel_future:
            time.sleep(2)  # 让计划开始执行
            
            # 取消计划
            cancel_result = ros_manager.cancel_current_plan()
            if cancel_result:
                logger.info("✅ Plan cancellation requested")
            else:
                logger.warning("⚠️  Failed to cancel plan")
        
        # 保持运行一段时间以观察结果
        logger.info("⏳ Keeping system running to observe results...")
        time.sleep(5)
        
    except KeyboardInterrupt:
        logger.info("🛑 Demonstration interrupted by user")
    except Exception as e:
        logger.error(f"❌ Error during demonstration: {e}")
    finally:
        # 清理
        logger.info("🧹 Cleaning up...")
        ros_manager.stop()
        rclpy.shutdown()
        logger.info("✅ Demonstration completed")


def compare_modes():
    """比较Topic模式和Action模式的差异"""
    
    logger.info("\n📊 Comparison: Topic vs Action Mode")
    logger.info("=" * 50)
    
    logger.info("📡 Topic Mode:")
    logger.info("  ✓ 简单的发布/订阅模式")
    logger.info("  ✓ 低延迟")
    logger.info("  ✗ 无执行状态跟踪")
    logger.info("  ✗ 无取消机制")
    logger.info("  ✗ 无进度反馈")
    
    logger.info("\n🎯 Action Mode:")
    logger.info("  ✓ 完整的执行状态跟踪")
    logger.info("  ✓ 实时进度反馈")
    logger.info("  ✓ 支持取消操作")
    logger.info("  ✓ 结果确认机制")
    logger.info("  ✗ 稍高的通信开销")
    
    logger.info("\n💡 建议使用场景:")
    logger.info("  📡 Topic Mode: 简单状态广播、高频数据传输")
    logger.info("  🎯 Action Mode: 复杂任务执行、需要反馈的长时间操作")


if __name__ == '__main__':
    # 设置日志级别
    import logging
    logging.basicConfig(level=logging.INFO)
    
    # 显示模式比较
    compare_modes()
    
    # 运行演示
    demonstrate_action_system()