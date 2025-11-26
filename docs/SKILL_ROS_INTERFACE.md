# Skill ROS Interface - 使用文档

## 架构概述

### 三层架构（CARLA 风格）

```
┌─────────────────────────────────────────────────────────────┐
│                   Application Layer                          │
│  ┌──────────────────┐         ┌──────────────────────┐     │
│  │  SkillManager    │◄────────│ SkillROSInterface    │     │
│  │  - 技能注册       │         │  - ROS Action Server │     │
│  │  - 状态管理       │         │  - 参数解析          │     │
│  │  - 执行逻辑       │         │  - Feedback 发布     │     │
│  └──────────────────┘         └──────────────────────┘     │
└─────────────────────────────────────────────────────────────┘
                              ▲
                              │ ROS Action
                              │
┌─────────────────────────────────────────────────────────────┐
│                      ROS Layer                               │
│  ┌──────────────────┐         ┌──────────────────────┐     │
│  │   NodeRobot      │         │  RobotRosManager     │     │
│  │  - odom pub      │         │  - 管理所有 ROS 节点  │     │
│  │  - cmd_vel sub   │         │  - Executor          │     │
│  └──────────────────┘         └──────────────────────┘     │
└─────────────────────────────────────────────────────────────┘
                              ▲
                              │
┌─────────────────────────────────────────────────────────────┐
│                   Simulation Layer                           │
│  ┌──────────────────────────────────────────────────────┐   │
│  │                    Robot                              │   │
│  │  - 状态（position, velocity）                         │   │
│  │  - 控制（apply_control）                              │   │
│  │  - 传感器                                             │   │
│  └──────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────┘
```

## 职责划分

### SkillManager（保持不变）
- **职责**：技能的核心逻辑
- **功能**：
  - 全局技能注册表（`@SkillManager.register()`）
  - 技能状态管理（states, data, errors）
  - 技能执行（`execute_skill()`）
  - 反馈构造（`form_feedback()`）

### SkillROSInterface（新增）
- **职责**：为 SkillManager 提供 ROS 接口
- **功能**：
  - 创建 ROS Action Server
  - 接收并解析 ROS action 请求
  - 调用 SkillManager 执行技能
  - 发布实时 feedback
  - 返回最终 result

### 关键特性
- **每个机器人一个实例**：jetbot_0 和 cf2x_0 各有独立的 SkillROSInterface
- **独立的 ROS node**：不复用 NodeRobot
- **独立的线程**：不阻塞主仿真循环
- **支持并发**：多个机器人可以同时接收和执行技能

## 使用方法

### 1. 在 main.py 中创建（Application Layer）

```python
import rclpy
from application import SkillManager, SkillROSInterface

# Initialize ROS
rclpy.init()

# 1. 创建机器人 (Simulation Layer)
robot = RobotJetbot(cfg_robot=cfg)
robot.initialize()

# 2. 创建 ROS Manager (ROS Layer)
ros_manager = RobotRosManager(
    robot=robot,
    namespace="jetbot_0",
    topics={
        "odom": "/jetbot_0/odom",
        "cmd_vel": "/jetbot_0/cmd_vel"
    }
)
robot.ros_manager = ros_manager
ros_manager.start()

# 3. 创建 Skill Manager (Application Layer)
skill_manager = SkillManager(robot, auto_register=True)
robot.skill_manager = skill_manager

# 4. 创建 Skill ROS Interface (Application Layer)
skill_ros_interface = SkillROSInterface(
    robot=robot,
    skill_manager=skill_manager,
    namespace="jetbot_0"
)
skill_ros_interface.start()

# 5. 运行仿真
world.play()

# ... simulation loop ...

# 6. 清理
skill_ros_interface.stop()
ros_manager.stop()
rclpy.shutdown()
```

### 2. 多机器人示例

```python
# 创建多个机器人
robots = []
skill_ros_interfaces = []

for i in range(3):
    namespace = f"jetbot_{i}"
    
    # 创建机器人
    robot = RobotJetbot(cfg_robot={...})
    robot.initialize()
    
    # 创建 ROS Manager
    ros_manager = RobotRosManager(robot, namespace, topics={...})
    robot.ros_manager = ros_manager
    ros_manager.start()
    
    # 创建 Skill Manager
    skill_manager = SkillManager(robot, auto_register=True)
    robot.skill_manager = skill_manager
    
    # 创建 Skill ROS Interface
    skill_ros_interface = SkillROSInterface(
        robot, skill_manager, namespace
    )
    skill_ros_interface.start()
    
    robots.append(robot)
    skill_ros_interfaces.append(skill_ros_interface)

# 运行仿真
world.play()

# 清理
for interface in skill_ros_interfaces:
    interface.stop()
```

### 3. ROS 命令使用

```bash
# 单个机器人
ros2 action send_goal /jetbot_0/skill_execution plan_msgs/action/SkillExecution \
  "{skill: navigate_to, params: [{key: goal_pos, value: '[3, 3, 0]'}]}" --feedback

# 多个机器人并发
ros2 action send_goal /jetbot_0/skill_execution plan_msgs/action/SkillExecution \
  "{skill: explore, params: [{key: boundary, value: '[[-4.4, 12], [3.0, 27.4]]'}]}" --feedback &

ros2 action send_goal /cf2x_0/skill_execution plan_msgs/action/SkillExecution \
  "{skill: take_off, params: [{key: altitude, value: '2.0'}]}" --feedback &

# 查看可用的 action servers
ros2 action list

# 查看 action 信息
ros2 action info /jetbot_0/skill_execution

# 取消正在执行的 action
ros2 action send_goal /jetbot_0/skill_execution plan_msgs/action/SkillExecution \
  "{skill: navigate_to, params: [...]}" --feedback
# 然后按 Ctrl+C 取消
```

### 4. 直接调用（不通过 ROS）

```python
# 也可以直接调用 SkillManager（不通过 ROS）
result = skill_manager.execute_skill(
    'navigate_to',
    goal_pos=[10, 20, 0]
)

print(result)  # {'status': 'processing', 'message': '...', 'progress': 50}
```

## 技能开发

### 技能注册（保持不变）

```python
from application import SkillManager

@SkillManager.register()
def my_skill(**kwargs):
    robot = kwargs.get("robot")
    skill_manager = kwargs.get("skill_manager")
    skill_name = "my_skill"
    
    # 获取当前状态
    current_state = skill_manager.get_skill_state(skill_name)
    
    # 状态机
    if current_state in [None, "INITIALIZING"]:
        # 初始化逻辑
        skill_manager.set_skill_state(skill_name, "EXECUTING")
        return skill_manager.form_feedback("processing", "Initializing", 10)
    
    elif current_state == "EXECUTING":
        # 执行逻辑
        # ... do something ...
        
        if done:
            skill_manager.set_skill_state(skill_name, "COMPLETED")
            return skill_manager.form_feedback("completed", "Done", 100)
        else:
            return skill_manager.form_feedback("processing", "Working", 50)
    
    elif current_state == "COMPLETED":
        return skill_manager.form_feedback("completed", "Completed", 100)
    
    elif current_state == "FAILED":
        error = skill_manager.skill_errors.get(skill_name, "Unknown")
        return skill_manager.form_feedback("failed", error)
```

## 架构优势

### 1. 清晰的职责分离
- **Simulation Layer**：纯仿真逻辑，不知道 ROS 和技能
- **ROS Layer**：纯通信逻辑，不知道技能
- **Application Layer**：业务逻辑 + ROS 接口

### 2. 灵活性
- 可以选择是否创建 SkillROSInterface
- 可以直接调用 SkillManager（不通过 ROS）
- 可以只用 ROS 接口

### 3. 可扩展性
- 容易添加新的 ROS 接口（如 SkillStreamingInterface）
- 容易支持其他通信协议（如 gRPC）
- 技能开发不需要关心 ROS

### 4. 并发支持
- 每个机器人独立的 action server
- 多机器人可以同时执行技能
- 线程安全（每个机器人独立的状态）

### 5. 符合 CARLA 架构
- 分层清晰
- 接口明确
- 易于理解和维护

## 常见问题

### Q: 为什么不在 NodeRobot 中创建 action server？
A: 为了保持 ROS Layer 的纯粹性。NodeRobot 只负责基础通信（odom, cmd_vel），技能执行属于 Application Layer 的业务逻辑。

### Q: 为什么每个机器人需要独立的 SkillROSInterface？
A: 因为每个机器人有独立的 namespace 和 action server。这样可以支持并发执行，多个机器人可以同时接收和执行技能。

### Q: SkillROSInterface 和 SkillManager 的关系？
A: SkillROSInterface 是 SkillManager 的 ROS 包装器。SkillManager 负责核心逻辑，SkillROSInterface 负责 ROS 通信。

### Q: 可以不创建 SkillROSInterface 吗？
A: 可以。如果不需要 ROS 接口，可以直接使用 SkillManager。SkillROSInterface 是可选的。

### Q: 如何调试技能执行？
A: 
1. 查看日志：`tail -f logs/application.log`
2. 使用 ROS 工具：`ros2 action list`, `ros2 action info`
3. 直接调用 SkillManager（不通过 ROS）进行测试

## 迁移指南

### 从旧架构迁移

**旧代码（NodeRobot 中的 action server）：**
```python
# 自动创建，用户无法控制
robot = RobotJetbot(...)
ros_manager = RobotRosManager(robot, ...)  # 内部创建 action server
```

**新代码（Application Layer 显式创建）：**
```python
# 用户显式创建，更清晰
robot = RobotJetbot(...)
ros_manager = RobotRosManager(robot, ...)  # 不创建 action server
skill_manager = SkillManager(robot, auto_register=True)
skill_ros_interface = SkillROSInterface(robot, skill_manager, namespace)
skill_ros_interface.start()
```

### 技能代码无需修改
- 技能装饰器：`@SkillManager.register()` 保持不变
- 技能函数签名：`def skill(**kwargs)` 保持不变
- 技能参数：`robot`, `skill_manager` 保持不变
- SkillManager API 保持不变

## 总结

新架构通过引入 `SkillROSInterface`，实现了：
1. 清晰的三层架构（Simulation, ROS, Application）
2. 职责分离（SkillManager 负责逻辑，SkillROSInterface 负责通信）
3. 灵活性（可选的 ROS 接口）
4. 并发支持（多机器人独立执行）
5. 符合 CARLA 设计理念

这是一个更加清晰、灵活、可维护的架构设计。
