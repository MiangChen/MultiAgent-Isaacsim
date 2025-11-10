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
│  │  World           │  │  Actor System    │                │
│  │  - spawn_actor() │  │  - RobotActor    │                │
│  │  - Blueprint     │  │  - StaticActor   │                │
│  └──────────────────┘  └──────────────────┘                │
└─────────────────────────────────────────────────────────────┘
                            ↓
┌─────────────────────────────────────────────────────────────┐
│                      Isaac Sim                               │
│              Physics Engine + Rendering                      │
└─────────────────────────────────────────────────────────────┘
```

## 🎯 核心设计原则

1. **分层边界清晰** - Application 层不直接调用 Isaac Sim API
2. **CARLA 风格 API** - 统一的 spawn_actor, apply_control 接口
3. **状态与命令分离** - 区分实际状态和控制命令
4. **同步控制** - MPC 直接设置速度，避免 ROS 延迟
5. **类型安全** - 使用 Blueprint, Transform, Control 等数据类

---

## ✅ 核心组件

### 1. World 类（CARLA 风格）

**统一的 Actor 创建接口：**
```python
# 创建机器人
blueprint_library = world.get_blueprint_library()
robot_bp = blueprint_library.find('robot.jetbot')
robot_bp.set_attribute('namespace', 'robot_0')
robot_actor = world.spawn_actor(robot_bp, transform)  # 返回 RobotActor

# 创建静态物体
car_bp = blueprint_library.find('static.prop.car')
car_bp.set_attribute('name', 'car0')
car_bp.set_attribute('scale', [2, 5, 1.0])
car_actor = world.spawn_actor(car_bp, transform)  # 返回 StaticActor
```

**关键方法：**
- `spawn_actor(blueprint, transform)` - 统一创建接口
- `get_blueprint_library()` - 获取 Blueprint 库
- `load_actors_from_config(path)` - 从配置文件加载
- `get_actors()` - 获取所有 Actor
- `tick()` - 仿真步进

**内部实现：**
- 使用 `blueprint.has_tag('static')` 判断类型
- 静态物体 → `_spawn_static_prop()` → 返回 `StaticActor`
- 机器人 → `_spawn_robot()` → 返回 `RobotActor`

---

### 2. Actor 系统

**Actor 基类：**
```python
class Actor:
    def get_id(self) -> int
    def get_type_id(self) -> str
    def get_transform(self) -> Transform
    def set_transform(self, transform: Transform)
    def get_location(self) -> Location
    def get_velocity(self) -> Vector3D
    def destroy()
```

**RobotActor（动态 Actor）：**
```python
class RobotActor(Actor):
    def __init__(self, robot, world):
        self.robot = robot  # 引用 Robot 实例
        robot.actor = self  # 双向引用
    
    def get_type_id(self) -> str:
        return f"robot.{self.robot.cfg_robot.type}"
```

**StaticActor（静态 Actor）：**
```python
class StaticActor(Actor):
    def __init__(self, prim_path, world, semantic_label):
        self._prim_path = prim_path
        self._semantic_label = semantic_label
    
    def get_type_id(self) -> str:
        return f"static.prop.{self._semantic_label}"
```

**设计优点：**
- 统一的接口，无论机器人还是静态物体
- 双向引用：`robot.actor` 和 `actor.robot`
- 自动注册到 World 的 Actor 列表

---

### 3. Robot 类 - 状态与命令分离（CARLA 风格）

**核心概念：**
- **状态变量**（State）：从 Isaac Sim 读取的实际值
- **命令变量**（Command）：控制器设置的目标值

**状态变量（私有，只读）：**
```python
# 在 on_physics_step 中从 Isaac Sim 更新
self.position = torch.tensor([0.0, 0.0, 0.0])
self.quat = torch.tensor([0.0, 0.0, 0.0, 1.0])
self._velocity = torch.tensor([0.0, 0.0, 0.0])  # 实际线速度
self._angular_velocity = torch.tensor([0.0, 0.0, 0.0])  # 实际角速度

# 公共接口（CARLA 风格）
def get_velocity(self) -> torch.Tensor
def get_angular_velocity(self) -> torch.Tensor
def get_world_pose() -> Tuple[torch.Tensor, torch.Tensor]
```

**命令变量（公共，可写）：**
```python
# 由控制器设置，在 controller_simplified 中应用到 Isaac Sim
self.target_velocity = torch.tensor([0.0, 0.0, 0.0])  # 目标线速度
self.target_angular_velocity = torch.tensor([0.0, 0.0, 0.0])  # 目标角速度

# 公共接口
def set_target_velocity(linear_velocity, angular_velocity=None)
def apply_control(control: RobotControl)
```

**关键：避免覆盖问题**
```python
def publish_robot_state(self):
    """在 on_physics_step 中调用，更新状态"""
    pos, quat = self._body.get_world_pose()
    vel, ang_vel = self._body.get_world_vel()
    
    # 只更新状态变量
    self.position = pos
    self.quat = quat
    self._velocity = vel
    self._angular_velocity = ang_vel
    
    # 不更新 target_velocity/target_angular_velocity！
    # 它们是命令，由 MPC/控制器设置
```

---

### 4. 控制流程（解耦设计）

**Robot 层（on_physics_step）：**
```python
def on_physics_step(self, step_size):
    # 1. 从 Isaac Sim 读取状态，更新 position, quat, _velocity, _angular_velocity
    self.publish_robot_state()
    
    # 2. 更新相机视野
    self._update_camera_view()
    
    # 3. 将 target_velocity, target_angular_velocity 应用到 Isaac Sim
    # Note: target_velocity 由 MPC (Application 层) 通过 clock 回调设置
    self.controller_simplified()
```

**Application 层（MPC 自动触发）：**
```python
# application/skills/base/navigation/node_controller_mpc.py
def clock_callback(self, msg: Clock):
    """订阅 /isaacsim_simulation_clock，每次 world.tick() 后自动调用"""
    self.latest_sim_time = msg.clock.sec + msg.clock.nanosec / 1e9
    
    # 自动调用 control_loop（Application 层控制）
    self.control_loop()

def control_loop(self):
    """MPC 计算并直接设置 robot.target_velocity"""
    optimal_command = self.mpc_controller.solve(...)
    
    if self.robot:
        self.robot.target_velocity = torch.tensor([...])
        self.robot.target_angular_velocity = torch.tensor([...])
```

**完全解耦的设计：**
- ✅ Robot 层不知道 MPC 的存在
- ✅ MPC 通过 ROS clock 自动触发
- ✅ MPC 直接设置 `target_velocity`（同步，无延迟）
- ✅ Robot 只负责应用命令到 Isaac Sim

---

### 5. MPC 控制器 - 同步控制

**问题：ROS 异步延迟**
```python
# 错误方式：通过 ROS topic（异步，有延迟）
def control_loop(self):
    optimal_command = self.mpc_controller.solve(...)
    
    # 发布到 ROS topic
    cmd_msg = Twist()
    cmd_msg.linear.x = optimal_command[0]
    self.cmd_vel_pub.publish(cmd_msg)
    
    # ROS bridge 在另一个线程中接收，有延迟！
    # 当前帧的 controller_simplified() 会使用旧速度
```

**解决方案：直接设置（同步，无延迟）**
```python
class NodeMpcController(Node):
    def __init__(self, namespace: str, robot=None):
        self.robot = robot  # 直接引用 robot
    
    def control_loop(self):
        optimal_command = self.mpc_controller.solve(...)
        
        # 直接设置目标速度（同步，无延迟）
        if self.robot:
            self.robot.target_velocity = torch.tensor([
                optimal_command[0], 
                optimal_command[1], 
                optimal_command[2]
            ])
            self.robot.target_angular_velocity = torch.tensor([
                0.0, 0.0, optimal_command[3]
            ])
        
        # 仍然发布到 ROS（用于监控/调试）
        self.cmd_vel_pub.publish(cmd_msg)
```

**创建时传递 robot 引用：**
```python
# ros/ros_manager_robot.py
self.node_controller_mpc = NodeMpcController(
    namespace=self.namespace, 
    robot=self.robot  # 传递 robot 引用
)
```

---

### 6. Blueprint 系统

**预注册的类型：**
```python
# 机器人
- robot.jetbot
- robot.h1
- robot.g1
- robot.cf2x
- robot.autel
- robot.target

# 静态物体
- static.prop.box
- static.prop.car
```

**使用 tags 判断类型：**
```python
def spawn_actor(self, blueprint, transform=None):
    # 使用 tags 判断，而不是 robot_class is None
    if blueprint.has_tag('static'):
        return self._spawn_static_prop(blueprint, transform)
    
    if blueprint.has_tag('robot'):
        return self._spawn_robot(blueprint, transform)
    
    # Fallback
    return self._spawn_static_prop(blueprint, transform)
```

**优点：**
- 更灵活，可以有多个 tags
- 易于扩展（如 'vehicle', 'drone' 等）
- 不依赖 `robot_class is None` 这种隐式判断

---

### 7. 分层边界 - Application 层不调用 Isaac Sim API

**问题：**
```python
# 错误：Application 层直接调用 Isaac Sim API
pos, quat = robot.body.get_world_pose()  # 渲染期间会报错！
```

**解决方案：**
```python
# robot/robot.py
class Robot:
    def __init__(self):
        self._body: BodyRobot = None  # 私有，仅内部使用
    
    # 公共接口（Application 层使用）
    def get_world_pose(self):
        """返回缓存的状态，不调用 Isaac Sim API"""
        return self.position, self.quat
    
    def get_velocity(self):
        """返回缓存的实际速度"""
        return self._velocity
    
    def get_config(self):
        """返回配置"""
        return self._body.cfg_robot
    
    # 向后兼容（带警告）
    @property
    def body(self):
        warnings.warn("Direct access to robot.body is deprecated", DeprecationWarning)
        return self._body
```

**Application 层使用：**
```python
# application/skills/base/navigation/navigate_to.py
def navigate_to(robot, goal_pos, **kwargs):
    # 正确：使用公共接口
    start_pos, start_quat = robot.get_world_pose()
    
    # 错误：不要直接访问 body
    # start_pos, start_quat = robot.body.get_world_pose()
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
  "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {z: 0.5}}"
```

### 3. ROS Action（技能系统）
```bash
ros2 action send_goal /robot_0/skill_execution plan_msgs/action/SkillExecution \
  '{skill_request: {skill_list: [{skill: "navigate_to", params: [{key: "goal_pos", value: "[10, 20, 0]"}]}]}}' --feedback
```

### 4. MPC 控制（自动）
```python
# MPC 在 on_physics_step 中自动运行
# 直接设置 robot.target_velocity
# 无需手动干预
```

---

## 📁 文件结构

```
simulation/                          # 仿真层（CARLA 风格）
├── __init__.py                     # 导出公共类
├── server.py                       # Server 类
├── world.py                        # World 类
│   ├── spawn_actor()              # 统一创建接口
│   ├── _spawn_robot()             # 创建机器人
│   ├── _spawn_static_prop()       # 创建静态物体
│   └── load_actors_from_config()  # 从配置加载
├── actor.py                        # Actor 基类
├── robot_actor.py                  # RobotActor 类
├── static_actor.py                 # StaticActor 类（新增）
├── transform.py                    # Transform 数据类
├── blueprint.py                    # Blueprint 系统
└── control.py                      # RobotControl 类

robot/
├── robot.py                        # Robot 基类
│   ├── 状态变量（私有）
│   │   ├── position, quat
│   │   ├── _velocity
│   │   └── _angular_velocity
│   ├── 命令变量（公共）
│   │   ├── target_velocity
│   │   └── target_angular_velocity
│   ├── 公共接口
│   │   ├── get_world_pose()
│   │   ├── get_velocity()
│   │   ├── get_angular_velocity()
│   │   ├── set_target_velocity()
│   │   └── apply_control()
│   └── 内部方法
│       ├── publish_robot_state()
│       ├── controller_simplified()
│       └── on_physics_step()
└── body/                           # Body 实现（Isaac Sim 层）
    ├── body_robot.py
    ├── body_jetbot.py
    └── ...

application/
├── skill_manager.py                # 技能管理器
└── skills/                         # 技能实现
    ├── base/navigation/
    │   ├── navigate_to.py
    │   └── node_controller_mpc.py  # MPC 控制器
    └── ...

ros/
├── robot_ros_manager.py            # ROS 管理器
│   ├── 管理 NodeRobot 和导航节点
│   ├── 管理 executor 和线程
│   └── 创建 NodeMpcController(robot=robot)
└── node_robot.py                   # ROS Node（包含 cmd_vel 支持）
    ├── Publishers: odom
    ├── Subscribers: sim_clock, cmd_vel
    ├── Action Servers: skill_execution
    └── Action Clients: path_planner
```

---

## 🔄 完整数据流

### 导航控制流程（navigate_to skill）

```
1. 用户发起导航请求
   ROS Action: /robot_0/skill_execution
   ↓
2. Skill 执行
   navigate_to() 发送路径规划请求
   ↓
3. 路径规划（ROS）
   NodePlannerOmpl 计算路径
   ↓
4. 轨迹生成（ROS）
   NodeTrajectoryGenerator 生成带时间戳的轨迹
   ↓
5. 仿真循环（每帧）
   world.tick()
   ├─ Isaac Sim 步进
   ├─ 发布 /isaacsim_simulation_clock
   │
   ├─ [Application 层] MPC.clock_callback() 自动触发
   │  └─ control_loop()
   │     └─ robot.target_velocity = ...  # 直接设置命令
   │
   └─ [Robot 层] robot.on_physics_step()
      ├─ publish_robot_state()           # 更新状态
      └─ controller_simplified()         # 应用命令到 Isaac Sim
         └─ _body.set_linear_velocities(target_velocity)
   ↓
6. 状态反馈
   publish_robot_state() 读取新位置
   发布 odom 到 ROS
   MPC 使用新位置计算下一步
```

### 关键点

1. **完全解耦**：Robot 层不调用 MPC，MPC 通过 clock 自动触发
2. **MPC 直接设置速度**：`robot.target_velocity = ...`（同步，无延迟）
3. **状态不覆盖命令**：`publish_robot_state()` 只更新 `_velocity`，不更新 `target_velocity`
4. **命令应用到 Isaac Sim**：`controller_simplified()` 使用 `target_velocity`
5. **分层清晰**：Application 层计算命令，Robot 层执行命令

---

## 🎯 标准使用流程

```python
# 1. 初始化
import rclpy
from containers import get_container, reset_container

rclpy.init(args=None)
reset_container()
container = get_container()
world = container.world_configured()

# 2. 加载机器人（返回 Actor 列表）
robot_actors = world.load_actors_from_config("config/robot_swarm_cfg.yaml")
robots = [actor.robot for actor in robot_actors]  # 提取 Robot 对象

# 3. 创建静态物体
blueprint_library = world.get_blueprint_library()

car_bp = blueprint_library.find('static.prop.car')
car_bp.set_attribute('name', 'car0')
car_bp.set_attribute('scale', [2, 5, 1.0])
car_actor = world.spawn_actor(car_bp, Transform(location=Location(10, 5, 0)))

# 4. 设置 ROS（每个机器人）
from ros.ros_manager_robot import RobotRosManager

for robot in robots:
    ros_manager = RobotRosManager(
        robot=robot,
        namespace=robot.namespace,
        topics=robot.get_topics()  # 使用公共接口
    )
    robot.set_ros_manager(ros_manager)
    ros_manager.start()

# 5. 初始化
world.reset()
world.initialize_robots()

# 6. 添加物理回调
for i, robot in enumerate(robots):
    world.get_isaac_world().add_physics_callback(
        f"physics_step_robot_{i}",
        robot.on_physics_step
    )

# 7. cmd_vel 支持（已集成到 NodeRobot）
# Note: cmd_vel subscriber 已经集成到 NodeRobot 中
# 不需要额外的 RosControlBridge
# 只需确保 topics 配置中包含 cmd_vel：
# topics = {
#     "odom": f"/{namespace}/odom",
#     "cmd_vel": f"/{namespace}/cmd_vel",
# }

# 8. Skill System
from application import SkillManager

for robot in robots:
    skill_manager = SkillManager(robot, auto_register=True)
    robot.skill_manager = skill_manager

# 9. 主循环
while simulation_app.is_running():
    world.tick()

# 10. 清理
ros_bridge_manager.stop()
rclpy.shutdown()
```

---

## 🌟 关键改进总结

### 1. Actor 系统统一
- ✅ `spawn_actor()` 统一返回 Actor 对象
- ✅ RobotActor 和 StaticActor 继承自 Actor 基类
- ✅ 使用 Blueprint tags 判断类型，而不是 `robot_class is None`

### 2. 状态与命令分离（CARLA 风格）
- ✅ 状态变量：`_velocity`, `_angular_velocity`（实际值，只读）
- ✅ 命令变量：`target_velocity`, `target_angular_velocity`（目标值，可写）
- ✅ `publish_robot_state()` 只更新状态，不覆盖命令

### 3. 同步控制（避免 ROS 延迟）
- ✅ MPC 直接设置 `robot.target_velocity`
- ✅ 不依赖 ROS topic 的异步回调
- ✅ 在同一个 physics step 内完成：计算 → 设置 → 应用

### 4. 分层边界清晰
- ✅ `robot._body` 私有，Application 层不可访问
- ✅ 公共接口：`get_world_pose()`, `get_velocity()` 等
- ✅ 返回缓存值，不在 Application 层调用 Isaac Sim API

### 5. MPC 完全解耦（新增）
- ✅ MPC 通过订阅 `/isaacsim_simulation_clock` 自动触发
- ✅ Robot 层不调用 MPC，保持分层独立
- ✅ Application 层控制器自主运行

### 6. 命名约定（CARLA 风格）
- ✅ `get_velocity()` - 获取实际速度（状态）
- ✅ `target_velocity` - 目标速度（命令）
- ✅ `apply_control(control)` - 应用控制
- ✅ 符合 CARLA 和 ROS 的通用约定

---

## 🐛 常见问题与解决方案

### 问题 1：机器人不动（MPC 发布速度但机器人速度为 0）

**原因：**
`publish_robot_state()` 从 Isaac Sim 读取当前速度（0），覆盖了 MPC 设置的 `target_velocity`。

**解决方案：**
- 区分状态变量（`_velocity`）和命令变量（`target_velocity`）
- `publish_robot_state()` 只更新状态，不更新命令
- MPC 直接设置 `target_velocity`，不通过 ROS topic

### 问题 2：Application 层调用 Isaac Sim API 导致渲染错误

**原因：**
Application 层直接访问 `robot.body.get_world_pose()`，在渲染期间调用会报错。

**解决方案：**
- 将 `body` 改为 `_body`（私有）
- 提供公共接口：`get_world_pose()`, `get_velocity()` 等
- 返回缓存的状态值，不直接调用 Isaac Sim API

### 问题 3：spawn_actor 返回类型不一致

**原因：**
静态物体返回 `prim_path`（字符串），机器人返回 `robot` 对象。

**解决方案：**
- 创建 `StaticActor` 类包装静态物体
- `spawn_actor()` 统一返回 Actor 对象
- 使用 Blueprint tags 判断类型

### 问题 4：MPC 速度命令延迟一帧

**原因：**
MPC 通过 ROS topic 发布速度，ROS bridge 在另一个线程异步接收，有延迟。

**解决方案：**
- MPC 直接设置 `robot.target_velocity`（同步）
- 仍然发布到 ROS topic（用于监控）
- 在 `on_physics_step` 中按正确顺序执行

---

## 📚 相关文档

- `VELOCITY_NAMING_CONVENTION.md` - 速度命名约定（CARLA 风格）
- `CARLA_VELOCITY_NAMING_RESEARCH.md` - CARLA 命名研究
- `ARCHITECTURE_FIX_SUMMARY.md` - 架构修复总结
- `docs/ROS_DECOUPLING_FINAL_SUMMARY.md` - ROS 解耦总结
- `docs/WORLD_API_COMPARISON.md` - World API 对比
- `application/skills/README.md` - 技能开发指南

## 🚀 后续工作

### 短期
- [ ] 完善所有技能的实现
- [ ] 添加更多静态物体类型
- [ ] 优化性能和稳定性

### 中期
- [ ] 统一的传感器接口（参考 CARLA）
- [ ] Vehicle 类型支持（带物理控制）
- [ ] 碰撞检测和物理事件

### 长期
- [ ] Actor 生命周期管理
- [ ] 录制和回放
- [ ] 多机器人协同框架
- [ ] 完整的 CARLA API 兼容层

---

**最后更新：** 2024年（基于最新架构改进）
