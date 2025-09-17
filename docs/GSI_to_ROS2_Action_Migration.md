# GSI消息到ROS2 Action的转换方案

## 概述

本文档描述了如何将GSI系统中的消息机制转换为ROS2 Action模式，特别是针对计划执行和机器人反馈的处理。

## 转换动机

### 当前Topic模式的限制
- ❌ 无法跟踪任务执行状态
- ❌ 缺乏进度反馈机制
- ❌ 无法取消正在执行的任务
- ❌ 没有执行结果确认

### Action模式的优势
- ✅ 完整的任务生命周期管理
- ✅ 实时进度反馈
- ✅ 支持任务取消
- ✅ 明确的成功/失败结果
- ✅ 更好的错误处理

## 架构设计

### 1. Action定义

#### ExecutePlan.action
```
# Goal: 要执行的计划
Plan plan
string robot_id
---
# Result: 执行结果
bool success
string message
SkillInfo[] completed_skills
---
# Feedback: 执行过程中的反馈
int32 current_timestep
int32 total_timesteps
SkillInfo current_skill
string status
float32 progress_percentage
```

#### ExecuteSkill.action
```
# Goal: 要执行的单个技能
SkillInfo skill
string robot_id
---
# Result: 技能执行结果
bool success
string message
float32 execution_time
---
# Feedback: 技能执行反馈
string status
float32 progress_percentage
string current_phase
```

### 2. 核心组件

#### Action服务器 (action_server.py)
- `PlanActionServer`: 处理完整计划的执行
- `SkillActionServer`: 处理单个技能的执行
- 支持并发执行多个目标
- 提供详细的执行反馈

#### Action客户端 (action_client.py)
- `PlanActionClient`: 发送计划执行请求
- `SkillActionClient`: 发送技能执行请求
- 处理反馈和结果回调
- 支持任务取消

#### ROS管理器更新 (ros_manager.py)
- 支持Topic和Action两种模式
- 动态模式切换
- 统一的接口封装

## 使用方法

### 1. 基本使用

```python
from ros.ros_manager import RosManager
from gsi_msgs.gsi_msgs_helper import Plan

# 创建ROS管理器
ros_manager = RosManager()
ros_manager.set_mode('action')  # 设置为Action模式
ros_manager.start()

# 发送计划执行请求
future = ros_manager.send_plan_action(
    plan=my_plan,
    robot_id="robot_1",
    feedback_callback=my_feedback_callback
)
```

### 2. 反馈处理

```python
def plan_feedback_callback(feedback_msg):
    feedback = feedback_msg.feedback
    print(f"Progress: {feedback.progress_percentage:.1f}%")
    print(f"Current skill: {feedback.current_skill.skill}")
```

### 3. 任务取消

```python
# 取消当前执行的计划
ros_manager.cancel_current_plan()
```

## 迁移步骤

### 阶段1: 准备工作
1. ✅ 创建Action定义文件
2. ✅ 更新CMakeLists.txt
3. ✅ 实现Action服务器和客户端

### 阶段2: 集成测试
1. 🔄 测试Action服务器功能
2. 🔄 验证反馈机制
3. 🔄 测试取消功能

### 阶段3: 生产部署
1. ⏳ 更新现有代码使用Action接口
2. ⏳ 性能优化和监控
3. ⏳ 文档和培训

## 性能考虑

### 通信开销
- Action模式比Topic模式有稍高的通信开销
- 通过合理的反馈频率控制可以优化性能
- 建议计划级反馈: 2Hz，技能级反馈: 5Hz

### 并发处理
- 支持多个机器人同时执行不同计划
- 每个Action服务器可处理多个并发目标
- 使用多线程执行器提高响应性

### 内存管理
- Action状态会占用额外内存
- 及时清理完成的Action状态
- 监控长时间运行的Action

## 配置选项

参见 `config/action_config.yaml` 文件：

- 超时设置
- 并发限制
- 反馈频率
- 重试策略
- 性能监控

## 故障排除

### 常见问题

1. **Action服务器无响应**
   - 检查服务器是否正确启动
   - 验证网络连接
   - 查看日志输出

2. **反馈延迟**
   - 调整反馈发布频率
   - 检查网络带宽
   - 优化反馈消息大小

3. **任务取消失败**
   - 确认Action支持取消
   - 检查取消请求时机
   - 验证取消逻辑实现

### 调试工具

```bash
# 查看Action服务器状态
ros2 action list

# 监控Action执行
ros2 action send_goal /execute_plan plan_msgs/action/ExecutePlan "{plan: {...}, robot_id: 'robot_1'}"

# 查看Action反馈
ros2 topic echo /execute_plan/_action/feedback
```

## 最佳实践

1. **错误处理**: 始终处理Action执行失败的情况
2. **超时设置**: 为长时间运行的任务设置合理超时
3. **反馈频率**: 平衡信息量和网络开销
4. **状态管理**: 及时清理完成的Action状态
5. **日志记录**: 记录关键执行节点用于调试

## 示例代码

完整的使用示例请参考 `ros/example_action_usage.py` 文件。

## 总结

通过将GSI消息系统转换为ROS2 Action模式，我们获得了：

- 🎯 更好的任务执行控制
- 📊 实时进度监控
- 🛑 灵活的取消机制
- ✅ 明确的执行结果
- 🔧 更强的错误处理能力

这种转换为GSI系统提供了更加健壮和可控的任务执行框架。