# 仿真层架构总结

## 📐 三层架构设计

```
┌─────────────────────────────────────────────────────────────┐
│                    Application Layer                         │
│  ┌──────────────────┐  ┌──────────────────┐                │
│  │  Skill System    │  │  ROS Bridge      │                │
│  │  - SkillManager  │  │  - cmd_vel       │                │
│  │  - @register     │  │  - action server │                │
│  └──────────────────┘  └──────────────────┘                │
└─────────────────────────────────────────────────────────────┘
                            ↓
┌─────────────────────────────────────────────────────────────┐
│                    Simulation Layer                          │
│  ┌──────────────────┐  ┌──────────────────┐                │
│  │  World           │  │  Control         │                │
│  │  - spawn_actor() │  │  - RobotControl  │                │
│  │  - Blueprint     │  │  - apply_control │                │
│  └──────────────────┘  └──────────────────┘                │
└─────────────────────────────────────────────────────────────┘
                            ↓
┌─────────────────────────────────────────────────────────────┐
│                      Isaac Sim                               │
│              Physics Engine + Rendering                      │
└─────────────────────────────────────────────────────────────┘
```

---

## ✅ 已完成的核心组件

### 1. Simulation Layer（CARLA 风格）

#### 核心类
- ✅ `simulation/server.py` - Server 类，管理 simulation_app 启动
- ✅ `simulation/world.py` - World 类，统一的 spawn_actor 接口
- ✅ `simulation/actor.py` - Actor 基类
- ✅ `simulation/robot_actor.py` - RobotActor 类，封装 Robot 实例
- ✅ `simulation/transform.py` - Transform, Location, Rotation 数据类
- ✅ `simulation/blueprint.py` - Blueprint 系统
- ✅ `simulation/control.py` - RobotControl 类（CARLA 风格）

#### Blueprint 系统
```python
# 预注册的机器人类型
- robot.jetbot
- robot.h1
- robot.g1
- robot.cf2x
- robot.autel

# 预注册的静态物体
- static.prop.box
- static.prop.car
```

#### 使用示例
```python
# 创建机器人
blueprint_library = world.get_blueprint_library()
robot_bp = blueprint_library.find('robot.jetbot')
robot_bp.set_attribute('id', 0)
robot_bp.set_attribute('namespace', 'robot_0')
robot = world.spawn_actor(robot_bp, transform)

# 创建静态物体
car_bp = blueprint_library.find('static.prop.car')
car_bp.set_attribute('name', 'car0')
car_bp.set_attribute('scale', [2, 5, 1.0])
car = world.spawn_actor(car_bp, transform)
```

### 2. Control System（CARLA 风格）

#### RobotControl 类
```python
from simulation import RobotControl

control = RobotControl()
control.linear_velocity = [1.0, 0.0, 0.0]   # X 轴前进
control.angular_velocity = [0.0, 0.0, 0.5]  # Z 轴旋转
robot.apply_control(control)
```

#### 控制流程
```
应用层
  ├── Python API: robot.apply_control(control)
  └── ROS Topics: /<namespace>/cmd_vel
          ↓
桥接层
  └── RosControlBridge: ROS Twist -> RobotControl
          ↓
仿真层
  └── robot.apply_control() -> set_velocity_command()
          ↓
Isaac Sim
```

### 3. ROS Integration

#### RobotRosManager
每个机器人的 ROS 基础设施管理, 主要是创建Pub和Sub：
```python
from ros.robot_ros_manager import RobotRosManager

ros_manager = RobotRosManager(
    robot=robot,
    namespace=robot.namespace,
    topics=robot.body.cfg_robot.topics
)
robot.set_ros_manager(ros_manager)
ros_manager.start()
```

**功能：**
- ROS 节点管理（NodeRobot）
- Action clients（路径规划）
- Navigation nodes（Planner, Trajectory, MPC）
- Executor 和线程管理
- Publishers 和 subscribers

#### RosControlBridgeManager
ROS cmd_vel 到仿真层的桥接, 主要是解决ROS2的消息和仿真层之间的数据问题：
```python
from ros.ros_control_bridge import RosControlBridgeManager

ros_bridge_manager = RosControlBridgeManager()
ros_bridge_manager.add_robots(robots)
ros_bridge_manager.start()
```

**功能：**
- 订阅 `/<namespace>/cmd_vel`
- 转换 Twist 消息为 RobotControl
- 自动匹配机器人和 topic

**使用示例：**
```bash
# 控制机器人前进
ros2 topic pub /robot_0/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 1.0, y: 0.0, z: 0.0}}"
```

### 4. Skill System

#### SkillManager（装饰器注册 + 管理）

**装饰器注册技能：**
```python
from application import SkillManager

@SkillManager.register()
def navigate_to(robot, goal_pos, **kwargs):
    """技能实现"""
    pass

# 或指定自定义名称, 并在skill_config.yaml中配置该技能可以给哪些机器人用
@SkillManager.register()
def my_skill(robot, **kwargs):
    pass
```

**创建管理器并执行技能：**
```python
from application import SkillManager

# 自动注册所有全局技能
skill_manager = SkillManager(robot, auto_register=True)
robot.skill_manager = skill_manager

# 执行技能
result = skill_manager.execute_skill('navigate_to', goal_pos=[10, 20, 0])

# 列出可用技能
print(skill_manager.list_skills())
```

**设计特点：**
- 使用类变量存储全局技能注册表
- 装饰器自动注册到全局注册表
- 每个机器人实例可选择性使用全局技能
- 简化架构，无需单独的 Registry 类

#### 已实现的技能

**运动技能：**
- `navigate_to` - 导航到目标点（A*, RRT, MPC）
- `explore` - 自主探索区域
- `track` - 跟踪目标
- `move` - 简单移动

**无人机技能：**
- `take_off` - 起飞到指定高度
- `land` - 降落（未完全实现）

**感知技能：**
- `take_photo` - 拍照
- `detect` - 目标检测
- `object_detection` - 物体检测

**操作技能：**
- `pick_up` - 抓取物体
- `put_down` - 放置物体

#### ROS Action 接口
```bash
# 通过 ROS2 action 执行技能
ros2 action send_goal /robot_0/skill_execution plan_msgs/action/SkillExecution \
  '{skill_request: {skill_list: [{skill: "navigate_to", params: [{key: "goal_pos", value: "[10, 20, 0]"}]}]}}' --feedback
```

---

## ⚙️ 配置系统

### ROS Topics 配置

**配置文件：** `config/config_parameter.yaml`

```yaml
robot_topics:
  jetbot:
    odom: "odom"
    cmd_vel: "cmd_vel"
    camera: "camera"
  
  cf2x:
    odom: "odom"
    cmd_vel: "cmd_vel"
    camera: "camera"
  
  drone_autel:
    cmd_vel: "cmd_vel"
    odom: "odom"
    camera: "camera"
  
  g1:
    odom: "odom"
    cmd_vel: "cmd_vel"
    camera: "camera"
  
  h1:
    odom: "odom"
    cmd_vel: "cmd_vel"
    camera: "camera"
```

### 配置工作流程

```
1. CfgRobot.__post_init__()
   ↓
2. 从 ROBOT_TOPICS[self.type] 获取 topics 配置
   ↓
3. RobotRosManager 使用 topics 创建 ROS 节点
   ↓
4. NodeRobot._create_publishers() 创建 publishers
   ↓
5. robot.publish_robot_state() 发布数据
```

**关键点：**
- `type` 字段必须与配置文件中的 key 匹配
- 每个机器人类型必须配置 `odom` topic（如果需要发布状态）
- `cmd_vel` 是可选的（如果需要 ROS 控制）

---

## 📁 文件结构

```
simulation/                          # 仿真层（CARLA 风格）
├── __init__.py                     # 导出所有公共类
├── server.py                       # Server 类
├── world.py                        # World 类
│   ├── spawn_actor()              # 统一创建接口
│   ├── load_actors_from_config()  # 从配置加载
│   ├── get_blueprint_library()    # 获取 blueprint 库
│   └── tick()                     # 仿真步进
├── actor.py                        # Actor 基类
├── robot_actor.py                  # RobotActor 类
├── transform.py                    # Transform 数据类
├── blueprint.py                    # Blueprint 系统
└── control.py                      # RobotControl 类

ros/                                 # ROS 层
├── robot_ros_manager.py            # 每个机器人的 ROS 管理
│   ├── RobotRosManager            # ROS 基础设施管理
│   ├── NodeRobot                  # 主 ROS 节点
│   ├── NodePlannerOmpl            # 路径规划节点
│   ├── NodeTrajectoryGenerator    # 轨迹生成节点
│   └── NodeMpcController          # MPC 控制节点
├── ros_control_bridge.py           # ROS 控制桥接
│   ├── RosControlBridge           # 单个机器人桥接
│   └── RosControlBridgeManager    # 多机器人管理
└── node_robot.py                   # ROS 节点实现
    ├── Publishers (odom, camera)
    ├── Subscribers (cmd_vel, clock)
    └── Action Servers (skill_execution)

application/                         # 应用层
├── skill_manager.py                # 技能管理器
│   ├── SkillManager               # 技能管理和执行
│   ├── @register 装饰器           # 全局技能注册
│   └── _global_skills             # 全局技能注册表（类变量）
└── skills/                         # 技能实现
    ├── base/                       # 基础技能
    │   ├── navigation/            # 导航技能
    │   ├── exploration/           # 探索技能
    │   └── detection/             # 检测技能
    ├── drone/                      # 无人机技能
    │   └── takeoff.py
    ├── manipulation/               # 操作技能
    │   ├── grasp.py
    │   └── place.py
    └── perception/                 # 感知技能
        └── take_photo.py

robot/
├── robot.py                        # Robot 基类
│   ├── apply_control()            # 统一控制接口
│   ├── set_velocity_command()     # 设置速度命令
│   └── publish_robot_state()      # 发布状态
└── cfg/                            # 机器人配置
    ├── cfg_robot.py               # 基础配置
    ├── cfg_drone_cf2x.py          # CF2X 配置
    └── cfg_drone_autel.py         # Autel 配置
```

---

## 🎯 标准使用流程

### 完整示例（main_example.py）

```python
# 1. 初始化
import rclpy
from containers import get_container, reset_container

rclpy.init(args=None)
reset_container()
container = get_container()
container.wire(modules=[__name__])

# 获取服务
world = container.world_configured()
simulation_app = container.server().get_simulation_app()

# 2. 加载机器人
robots = world.load_actors_from_config("config/robot_swarm_cfg.yaml")

# 3. 设置 ROS（每个机器人）
from ros.robot_ros_manager import RobotRosManager

for robot in robots:
    ros_manager = RobotRosManager(
        robot=robot,
        namespace=robot.namespace,
        topics=robot.body.cfg_robot.topics
    )
    robot.set_ros_manager(ros_manager)
    ros_manager.start()

# 4. 初始化机器人
world.reset()
world.initialize_robots()

# 5. 添加物理回调
for i, robot in enumerate(robots):
    world.get_isaac_world().add_physics_callback(
        f"physics_step_robot_{i}", 
        robot.on_physics_step
    )

# 6. Application Layer Setup
# 6.1 ROS Control Bridge
from ros.ros_control_bridge import RosControlBridgeManager

ros_bridge_manager = RosControlBridgeManager()
ros_bridge_manager.add_robots(robots)
ros_bridge_manager.start()

# 6.2 Skill System
from application import SkillManager

for robot in robots:
    skill_manager = SkillManager(robot, auto_register=True)
    robot.skill_manager = skill_manager

# 7. 主循环
while simulation_app.is_running():
    world.tick()

# 8. 清理
ros_bridge_manager.stop()
rclpy.shutdown()
```

---

## 🎮 控制方式

### 1. Python API（直接控制）
```python
from simulation import RobotControl

control = RobotControl()
control.linear_velocity = [1.0, 0.0, 0.0]
control.angular_velocity = [0.0, 0.0, 0.5]
robot.apply_control(control)
```

### 2. ROS Topic（速度控制）
```bash
ros2 topic pub /robot_0/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"
```

### 3. ROS Action（技能系统）
```bash
# 导航
ros2 action send_goal /robot_0/skill_execution plan_msgs/action/SkillExecution \
  '{skill_request: {skill_list: [{skill: "navigate_to", params: [{key: "goal_pos", value: "[10, 20, 0]"}]}]}}' --feedback

# 起飞（无人机）
ros2 action send_goal /cf2x_0/skill_execution plan_msgs/action/SkillExecution \
  '{skill_request: {skill_list: [{skill: "take_off", params: [{key: "altitude", value: "1.0"}]}]}}' --feedback

# 探索
ros2 action send_goal /jetbot_0/skill_execution plan_msgs/action/SkillExecution \
  "{skill_request: {skill_list: [{skill: explore, params: [{key: boundary, value: '[[-4.4, 12, 0], [3.3, 19.4, 0]]'}]}]}}" --feedback
```

---

## 🌟 核心特性

### 1. 统一的创建方式
所有物体（机器人和静态物体）都通过 `world.spawn_actor()` 创建

### 2. Blueprint 系统
- 预注册所有已知类型
- 支持属性设置和查询
- 自动分发到正确的创建方法

### 3. 解耦设计
- **仿真层**：不依赖 ROS，纯 Python API
- **ROS 层**：可选的 ROS 集成
- **应用层**：高级技能和任务

### 4. 多种控制方式
- 直接控制（Python API）
- ROS 速度控制（cmd_vel）
- ROS 技能控制（action）

### 5. 技能系统
- **装饰器注册**：`@SkillManager.register()` 自动注册技能
- **全局注册表**：使用类变量 `_global_skills` 存储所有技能
- **实例管理**：每个机器人有独立的 SkillManager 实例
- **自动加载**：`application/__init__.py` 导入所有技能模块触发注册

### 6. 自动化管理
- 自动技能注册（装饰器）
- 自动 ROS 节点管理
- 自动 topic 映射

---

## 🎓 设计原则

1. **统一接口** - 所有物体通过 `world.spawn_actor()` 创建
2. **CARLA 风格** - API 设计参考 CARLA，保持一致性
3. **封装复杂性** - 隐藏 Isaac Sim 的底层细节
4. **解耦设计** - 仿真层、ROS 层、应用层职责清晰
5. **类型安全** - 使用 Transform 等数据类，避免裸数组
6. **配置驱动** - 通过配置文件管理机器人类型和 topics
7. **简化架构** - 避免过度设计，合并相关功能到单一类



---

## 📝 技能系统详解

### 技能分类

**运动技能（Mobility）**
- `navigate_to` - 导航到目标点
- `explore` - 自主探索
- `track` - 目标跟踪
- `move` - 简单移动
- `take_off` - 起飞（无人机）

**感知技能（Perception）**
- `take_photo` - 拍照
- `detect` - 目标检测
- `object_detection` - 物体检测

**操作技能（Manipulation）**
- `pick_up` - 抓取物体
- `put_down` - 放置物体

### 技能系统架构

**核心概念：**
- **全局注册表**：`SkillManager._global_skills`（类变量）存储所有已注册的技能
- **装饰器注册**：`@SkillManager.register()` 在模块导入时自动注册技能
- **实例管理**：每个机器人有独立的 SkillManager 实例，管理该机器人的技能执行

**工作流程：**
```
1. 模块导入
   application/__init__.py imports application.skills
   ↓
2. 装饰器执行
   @SkillManager.register() 注册技能到 _global_skills
   ↓
3. 创建实例
   skill_manager = SkillManager(robot, auto_register=True)
   ↓
4. 复制技能
   从 _global_skills 复制到实例的 self.skills
   ↓
5. 执行技能
   skill_manager.execute_skill('navigate_to', ...)
```

### 使用示例

**1. 定义技能**
```python
from application import SkillManager

@SkillManager.register()
def my_skill(robot, param1, param2, **kwargs):
    """
    自定义技能实现
    
    Args:
        robot: 机器人实例（自动注入）
        param1, param2: 技能参数
        skill_manager: 技能管理器（自动注入）
    """
    skill_manager = kwargs.get('skill_manager')
    
    # 技能逻辑
    result = do_something(robot, param1, param2)
    
    return {"status": "success", "result": result}
```

**2. 创建管理器**
```python
from application import SkillManager

# 自动注册所有全局技能
skill_manager = SkillManager(robot, auto_register=True)
robot.skill_manager = skill_manager
```

**3. 执行技能**
```python
# Python API
result = skill_manager.execute_skill('my_skill', param1=value1, param2=value2)

# ROS Action
# ros2 action send_goal /robot_0/skill_execution plan_msgs/action/SkillExecution \
#   '{skill_request: {skill_list: [{skill: "my_skill", params: [...]}]}}' --feedback
```

**4. 查看可用技能**
```python
# 全局注册的所有技能
print(SkillManager.list_global_skills())

# 当前机器人实例的技能
print(skill_manager.list_skills())
```

### 技能执行流程

```
ROS Action Request
    ↓
NodeRobot.execute_callback_wrapper()
    ↓
SkillManager.execute_skill()
    ↓
技能函数执行
    ↓
返回结果
    ↓
ROS Action Response
```

### 依赖注入

技能函数可以使用 `@inject` 装饰器自动注入依赖：

```python
from dependency_injector.wiring import inject, Provide
from application import SkillManager


@SkillManager.register()
@inject
def navigate_to(
        robot,
        goal_pos,
        grid_map=Provide["grid_map"],
        scene_manager=Provide["scene_manager"],
        **kwargs
):
    """导航技能 - 自动注入 grid_map 和 scene_manager"""
    path = grid_map.plan_path(robot.position, goal_pos)
    # ...
```

**注意：** 装饰器顺序很重要，`@inject` 应该在 `@SkillManager.register()` 之后。

### 技能状态管理

SkillManager 提供状态管理功能，用于跟踪技能执行状态：

```python
# 设置/获取技能状态
skill_manager.set_skill_state('navigate_to', 'EXECUTING')
state = skill_manager.get_skill_state('navigate_to')

# 设置/获取技能数据
skill_manager.set_skill_data('navigate_to', 'target_pos', [10, 20, 0])
target = skill_manager.get_skill_data('navigate_to', 'target_pos')

# 构造反馈消息
feedback = skill_manager.form_feedback(
    status="processing",
    message="Navigating to target",
    progress=50
)
```

---

## 🔄 数据流

### 状态发布流程

```
robot.on_physics_step()
    ↓
robot.update_state()
    ↓
robot.publish_robot_state()
    ↓
ros_manager.publish_odometry()
    ↓
ROS odom topic
```

### 控制接收流程

```
ROS cmd_vel topic
    ↓
RosControlBridge._cmd_vel_callback()
    ↓
创建 RobotControl 对象
    ↓
robot.apply_control(control)
    ↓
robot.set_velocity_command()
    ↓
Isaac Sim 执行
```

---

## 🚀 后续工作

### 短期 - 完善现有功能
- [ ] 完善所有技能的实现
- [ ] 添加更多机器人类型
- [ ] 优化性能和稳定性

### 中期 - 传感器系统
- [ ] 统一的传感器接口（参考 CARLA）
- [ ] 传感器数据回调机制
- [ ] 多种传感器类型支持

### 长期 - 高级功能
- [ ] Actor 生命周期管理
- [ ] 碰撞检测和物理事件
- [ ] 录制和回放
- [ ] 多机器人协同框架

---

## 📚 参考文档

- `docs/ROS_DECOUPLING_FINAL_SUMMARY.md` - ROS 解耦总结
- `docs/APPLICATION_LAYER_REFACTOR.md` - 应用层重构
- `docs/ROS_ACTION_INTERFACE.md` - ROS Action 接口
- `docs/QUICK_REFERENCE.md` - 快速参考
- `application/skills/README.md` - 技能开发指南
