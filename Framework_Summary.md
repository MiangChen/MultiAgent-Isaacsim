# 架构总结

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
self._position = torch.tensor([0.0, 0.0, 0.0])
self._quat = torch.tensor([0.0, 0.0, 0.0, 1.0])
self._linear_velocity = torch.tensor([0.0, 0.0, 0.0])  # 实际线速度
self._angular_velocity = torch.tensor([0.0, 0.0, 0.0])  # 实际角速度


# 公共接口（CARLA 风格）
def get_velocity(self) -> torch.Tensor


    def get_angular_velocity(self) -> torch.Tensor

    def get_world_pose() -> Tuple[torch.Tensor, torch.Tensor]
```

**命令变量（公共，可写）：**

```python
# 由控制器设置，在 controller_simplified 中应用到 Isaac Sim
self.target_linear_velocity = torch.tensor([0.0, 0.0, 0.0])  # 目标线速度
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
    self._position = pos
    self._quat = quat
    self._linear_velocity = vel
    self._angular_velocity = ang_vel

    # 不更新 target_linear_velocity/target_angular_velocity！
    # 它们是命令，由 MPC/控制器设置
```

---

***

### 6. Blueprint 类

Blueprint 系统提供了类似 CARLA 的 Actor 创建机制，通过类型标识符和属性配置来创建不同类型的对象。

#### 6.1 Blueprint 库

**获取 Blueprint 库：**
```python
blueprint_library = world.get_blueprint_library()

# 查找特定类型的 Blueprint
robot_bp = blueprint_library.find('robot.jetbot')
car_bp = blueprint_library.find('static.prop.car')

# 获取所有可用的 Blueprint
all_blueprints = blueprint_library.filter('*')  # 返回所有
robot_blueprints = blueprint_library.filter('robot.*')  # 只返回机器人
```

#### 6.2 预注册的类型

**机器人类型（robot.*）：**
```python
- robot.jetbot      # Jetbot 轮式机器人
- robot.h1          # Unitree H1 人形机器人
- robot.g1          # Unitree G1 人形机器人
- robot.cf2x        # Crazyflie 2.x 四旋翼无人机
- robot.autel       # Autel 无人机
- robot.target      # 目标机器人（用于跟踪任务）
```

**静态物体类型（static.prop.*）：**
```python
- static.prop.box   # 立方体障碍物
- static.prop.car   # 汽车模型（静态）
```

#### 6.3 Blueprint 属性配置

**机器人 Blueprint 属性：**
```python
robot_bp = blueprint_library.find('robot.jetbot')

# 设置命名空间（必需）
robot_bp.set_attribute('namespace', 'robot_0')

# 设置初始位置（可选，也可以在 spawn_actor 时指定）
robot_bp.set_attribute('position', [0.0, 0.0, 0.5])
robot_bp.set_attribute('orientation', [0.0, 0.0, 0.0, 1.0])  # 四元数

# 设置传感器配置（可选）
robot_bp.set_attribute('enable_lidar', True)
robot_bp.set_attribute('enable_camera', True)
```

**静态物体 Blueprint 属性：**
```python
car_bp = blueprint_library.find('static.prop.car')

# 设置名称（必需）
car_bp.set_attribute('name', 'car0')

# 设置缩放（可选，默认 [1.0, 1.0, 1.0]）
car_bp.set_attribute('scale', [2.0, 5.0, 1.0])  # [x, y, z]

# 设置语义标签（可选，用于传感器识别）
car_bp.set_attribute('semantic_label', 'car')

# 设置颜色（可选）
car_bp.set_attribute('color', [1.0, 0.0, 0.0])  # RGB
```

#### 6.4 使用 Tags 判断类型

Blueprint 使用 tags 系统来标识对象类型，这比使用类名或其他隐式判断更灵活。

**内部实现：**
```python
class ActorBlueprint:
    def __init__(self, id: str, tags: List[str]):
        self.id = id  # 例如 'robot.jetbot'
        self.tags = tags  # 例如 ['robot', 'wheeled']
        self._attributes = {}
    
    def has_tag(self, tag: str) -> bool:
        """检查是否包含指定 tag"""
        return tag in self.tags
    
    def set_attribute(self, key: str, value):
        """设置属性"""
        self._attributes[key] = value
    
    def get_attribute(self, key: str):
        """获取属性"""
        return self._attributes.get(key)
```

**World 中的类型判断：**
```python
def spawn_actor(self, blueprint, transform=None):
    """根据 Blueprint 的 tags 创建对应的 Actor"""
    
    # 使用 tags 判断类型（推荐方式）
    if blueprint.has_tag('static'):
        return self._spawn_static_prop(blueprint, transform)
    
    if blueprint.has_tag('robot'):
        return self._spawn_robot(blueprint, transform)
    
    # 未来可扩展
    if blueprint.has_tag('vehicle'):
        return self._spawn_vehicle(blueprint, transform)
    
    if blueprint.has_tag('sensor'):
        return self._spawn_sensor(blueprint, transform)
    
    # Fallback：默认作为静态物体
    return self._spawn_static_prop(blueprint, transform)
```

#### 6.5 完整使用示例

**创建机器人：**
```python
# 1. 获取 Blueprint
blueprint_library = world.get_blueprint_library()
robot_bp = blueprint_library.find('robot.jetbot')

# 2. 配置属性
robot_bp.set_attribute('namespace', 'robot_0')

# 3. 创建 Transform
transform = Transform(
    location=Location(x=0.0, y=0.0, z=0.5),
    rotation=Rotation(roll=0.0, pitch=0.0, yaw=0.0)
)

# 4. 生成 Actor
robot_actor = world.spawn_actor(robot_bp, transform)

# 5. 获取 Robot 对象（如果需要）
robot = robot_actor.robot
```

**创建静态物体：**
```python
# 1. 获取 Blueprint
car_bp = blueprint_library.find('static.prop.car')

# 2. 配置属性
car_bp.set_attribute('name', 'car0')
car_bp.set_attribute('scale', [2.0, 5.0, 1.0])
car_bp.set_attribute('semantic_label', 'car')

# 3. 创建 Transform
transform = Transform(location=Location(x=10.0, y=5.0, z=0.0))

# 4. 生成 Actor
car_actor = world.spawn_actor(car_bp, transform)

# 5. 获取 Actor 信息
print(f"Actor ID: {car_actor.get_id()}")
print(f"Actor Type: {car_actor.get_type_id()}")  # 输出: static.prop.car
```

#### 6.6 设计优点

**1. 灵活的类型系统：**
- 一个 Blueprint 可以有多个 tags（例如：`['robot', 'wheeled', 'ground']`）
- 可以根据不同的 tag 进行过滤和判断
- 易于添加新的类型分类

**2. 易于扩展：**
```python
# 添加新类型只需注册新的 Blueprint
blueprint_library.register(
    ActorBlueprint(
        id='vehicle.sedan',
        tags=['vehicle', 'wheeled', 'dynamic']
    )
)

# World 中添加对应的生成方法
def spawn_actor(self, blueprint, transform=None):
    if blueprint.has_tag('vehicle'):
        return self._spawn_vehicle(blueprint, transform)
    # ...
```

**3. 显式类型判断：**
- 不依赖 `robot_class is None` 这种隐式判断
- 代码意图清晰，易于理解和维护
- 避免了类型判断的歧义

**4. 属性系统灵活：**
- 每个 Blueprint 可以有不同的属性
- 属性在创建时配置，不需要修改类定义
- 支持运行时动态配置

#### 6.7 与 CARLA 的对比

| 特性 | CARLA | 本系统 |
|------|-------|--------|
| Blueprint 库 | `world.get_blueprint_library()` | ✅ 相同 |
| 类型标识符 | `vehicle.tesla.model3` | ✅ 类似（`robot.jetbot`） |
| Tags 系统 | ✅ 支持 | ✅ 支持 |
| 属性配置 | `bp.set_attribute()` | ✅ 相同 |
| 统一创建接口 | `world.spawn_actor()` | ✅ 相同 |
| 返回类型 | Actor 对象 | ✅ Actor 对象 |

---

### 7. ROS Robot Manager 类

ROS Robot Manager 负责管理单个机器人的所有 ROS 基础设施，实现了仿真层与 ROS 层的完全解耦。每个机器人都有独立的 ROS Manager 实例，管理其 ROS 节点、话题、服务和 Action。

#### 7.1 架构概览

**ROS Robot Manager 的职责：**
```
┌─────────────────────────────────────────────────────────────┐
│              RobotRosManager (per robot)                     │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  ROS Nodes                                            │  │
│  │  - NodeRobot (主节点)                                 │  │
│  │  - NodePlannerOmpl (路径规划)                         │  │
│  │  - NodeTrajectoryGenerator (轨迹生成)                 │  │
│  │  - NodeMpcController (MPC 控制器)                     │  │
│  └──────────────────────────────────────────────────────┘  │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  ROS Communication                                    │  │
│  │  - Publishers: /robot_0/odom                          │  │
│  │  - Subscribers: /robot_0/cmd_vel, /sim_clock         │  │
│  │  - Action Servers: /robot_0/skill_execution          │  │
│  │  - Action Clients: /robot_0/compute_path_to_pose     │  │
│  └──────────────────────────────────────────────────────┘  │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  Threading & Execution                                │  │
│  │  - MultiThreadedExecutor                              │  │
│  │  - ROS Thread (独立线程运行)                          │  │
│  │  - Stop Event (优雅关闭)                              │  │
│  └──────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
```

#### 7.2 核心组件

**RobotRosManager 类：**
```python
class RobotRosManager:
    """管理单个机器人的所有 ROS 基础设施"""
    
    def __init__(self, robot, namespace: str, topics: dict):
        """
        初始化 ROS Manager
        
        Args:
            robot: Robot 实例（仿真层）
            namespace: 机器人命名空间（例如 'robot_0'）
            topics: ROS 话题配置字典
        """
        self.robot = robot
        self.namespace = namespace
        self.topics = topics
        
        # ROS 节点
        self.node = None                          # 主节点
        self.node_planner_ompl = None             # OMPL 路径规划器
        self.node_trajectory_generator = None     # 轨迹生成器
        self.node_controller_mpc = None           # MPC 控制器
        
        # Action 客户端
        self.action_client_path_planner = None
        
        # 执行器和线程
        self.executor = None
        self.ros_thread = None
        self.stop_event = threading.Event()
```

#### 7.3 初始化流程

**完整的初始化过程：**
```python
# 1. 创建 ROS Manager
ros_manager = RobotRosManager(
    robot=robot,
    namespace='robot_0',
    topics={
        'odom': '/robot_0/odom',
        'cmd_vel': '/robot_0/cmd_vel',
    }
)

# 2. 内部自动初始化（在 __init__ 中）
# - _init_ros_nodes()        # 创建所有 ROS 节点
# - _init_action_clients()   # 创建 Action 客户端
# - _init_executor()         # 创建执行器并添加节点

# 3. 设置到 Robot
robot.set_ros_manager(ros_manager)

# 4. 启动 ROS 线程
ros_manager.start()
```

**内部初始化方法：**
```python
def _init_ros_nodes(self):
    """初始化所有 ROS 节点"""
    # 主节点（处理 odom, cmd_vel, skill_execution）
    self.node = NodeRobot(
        namespace=self.namespace,
        topics=self.topics
    )
    self.node.set_robot_instance(self.robot)
    
    # 导航节点
    self.node_planner_ompl = NodePlannerOmpl(
        namespace=self.namespace
    )
    self.node_trajectory_generator = NodeTrajectoryGenerator(
        namespace=self.namespace
    )
    self.node_controller_mpc = NodeMpcController(
        namespace=self.namespace,
        robot=self.robot  # 传递 robot 引用用于直接控制
    )

def _init_action_clients(self):
    """初始化 Action 客户端"""
    self.action_client_path_planner = ActionClient(
        self.node,
        ComputePathToPose,
        'action_compute_path_to_pose'
    )

def _init_executor(self):
    """初始化执行器并添加所有节点"""
    self.executor = MultiThreadedExecutor()
    self.executor.add_node(self.node)
    self.executor.add_node(self.node_planner_ompl)
    self.executor.add_node(self.node_trajectory_generator)
    self.executor.add_node(self.node_controller_mpc)
```

#### 7.4 NodeRobot - 主 ROS 节点

NodeRobot 是每个机器人的主要 ROS 节点，负责基本的 ROS 通信。

**Publishers（发布者）：**
```python
# Odometry（里程计）
/robot_0/odom  # nav_msgs/Odometry
# 发布频率：每个 physics step（通常 60Hz）
# 内容：位置、姿态、线速度、角速度
```

**Subscribers（订阅者）：**
```python
# cmd_vel（速度命令）
/robot_0/cmd_vel  # geometry_msgs/Twist
# 用途：外部控制（遥控、键盘、joystick）
# 回调：自动转换为 RobotControl 并应用

# Simulation Clock（仿真时钟）
/isaacsim_simulation_clock  # rosgraph_msgs/Clock
# 用途：同步仿真时间
# 触发：MPC 控制器的 control_loop
```

**Action Servers（动作服务器）：**
```python
# Skill Execution（技能执行）
/robot_0/skill_execution  # plan_msgs/SkillExecution
# 用途：执行高级技能（navigate_to, follow_path 等）
# 反馈：实时状态和进度
```

**cmd_vel 回调示例：**
```python
def callback_cmd_vel(self, msg: Twist):
    """
    接收外部速度命令并应用到机器人
    
    支持的控制方式：
    - ros2 topic pub
    - teleop_twist_keyboard
    - joystick
    """
    from simulation.control import RobotControl
    
    control = RobotControl()
    control.linear_velocity = [msg.linear.x, msg.linear.y, msg.linear.z]
    control.angular_velocity = [msg.angular.x, msg.angular.y, msg.angular.z]
    
    if self.robot_instance:
        self.robot_instance.apply_control(control)
```

#### 7.5 导航节点

**NodePlannerOmpl（路径规划器）：**
```python
# Action Server
/robot_0/action_compute_path_to_pose  # nav2_msgs/ComputePathToPose

# 功能：
# - 使用 OMPL 算法计算无碰撞路径
# - 支持多种规划算法（RRT, RRT*, PRM 等）
# - 考虑障碍物和机器人尺寸
# - 返回路径点列表
```

**NodeTrajectoryGenerator（轨迹生成器）：**
```python
# Action Server
/robot_0/action_generate_trajectory  # plan_msgs/GenerateTrajectory

# 功能：
# - 将路径转换为带时间戳的轨迹
# - 考虑速度和加速度限制
# - 生成平滑的速度曲线
# - 用于 MPC 跟踪
```

**NodeMpcController（MPC 控制器）：**
```python
# Subscriber
/isaacsim_simulation_clock  # rosgraph_msgs/Clock

# 功能：
# - 订阅仿真时钟，自动触发控制循环
# - 使用 MPC 算法计算最优控制
# - 直接设置 robot.target_velocity（同步，无延迟）
# - 发布 cmd_vel 用于监控和调试

# 关键特性：
# ✅ 自动触发（通过 clock_callback）
# ✅ 同步控制（直接设置速度）
# ✅ 完全解耦（Robot 层不知道 MPC 的存在）
```

#### 7.6 线程管理

**独立的 ROS 线程：**
```python
def start(self):
    """启动 ROS 线程"""
    if self.ros_thread is None or not self.ros_thread.is_alive():
        self.stop_event.clear()
        self.ros_thread = threading.Thread(
            target=self._spin_ros,
            daemon=True,
            name=f"ROS_{self.namespace}"
        )
        self.ros_thread.start()

def _spin_ros(self):
    """ROS 旋转循环（在独立线程中运行）"""
    try:
        while not self.stop_event.is_set():
            self.executor.spin_once(timeout_sec=0.05)
    except Exception as e:
        logger.error(f"ROS thread error: {e}")

def stop(self):
    """停止 ROS 线程并清理"""
    if self.ros_thread and self.ros_thread.is_alive():
        self.stop_event.set()
        self.ros_thread.join(timeout=2.0)
```

**线程安全设计：**
- ROS 通信在独立线程中运行
- 使用 `stop_event` 实现优雅关闭
- MultiThreadedExecutor 处理多个节点
- 避免阻塞主仿真循环

#### 7.7 完整使用示例

**单机器人设置：**
```python
import rclpy
from containers import get_container
from ros.ros_manager_robot import RobotRosManager

# 1. 初始化 ROS
rclpy.init(args=None)

# 2. 创建机器人
container = get_container()
world = container.world_configured()
robot_actors = world.load_actors_from_config("config/robot_cfg.yaml")
robot = robot_actors[0].robot

# 3. 配置 ROS 话题
topics = {
    'odom': f'/{robot.namespace}/odom',
    'cmd_vel': f'/{robot.namespace}/cmd_vel',
}

# 4. 创建并启动 ROS Manager
ros_manager = RobotRosManager(
    robot=robot,
    namespace=robot.namespace,
    topics=topics
)
robot.set_ros_manager(ros_manager)
ros_manager.start()

# 5. 初始化机器人
world.reset()
world.initialize_robots()

# 6. 添加物理回调
world.get_isaac_world().add_physics_callback(
    "physics_step_robot_0",
    robot.on_physics_step
)

# 7. 主循环
while simulation_app.is_running():
    world.tick()

# 8. 清理
ros_manager.stop()
rclpy.shutdown()
```

**多机器人设置：**
```python
# 加载多个机器人
robot_actors = world.load_actors_from_config("config/robot_swarm_cfg.yaml")
robots = [actor.robot for actor in robot_actors]

# 为每个机器人创建 ROS Manager
ros_managers = []
for i, robot in enumerate(robots):
    topics = {
        'odom': f'/{robot.namespace}/odom',
        'cmd_vel': f'/{robot.namespace}/cmd_vel',
    }
    
    ros_manager = RobotRosManager(
        robot=robot,
        namespace=robot.namespace,
        topics=topics
    )
    robot.set_ros_manager(ros_manager)
    ros_manager.start()
    ros_managers.append(ros_manager)
    
    # 添加物理回调
    world.get_isaac_world().add_physics_callback(
        f"physics_step_robot_{i}",
        robot.on_physics_step
    )

# 主循环
while simulation_app.is_running():
    world.tick()

# 清理所有 ROS Manager
for ros_manager in ros_managers:
    ros_manager.stop()
```

#### 7.8 ROS 通信示例

**发布 Odometry：**
```python
# 在 robot.on_physics_step() 中自动调用
def publish_robot_state(self):
    """发布机器人状态到 ROS"""
    if self.ros_manager:
        pos, quat = self.get_world_pose()
        vel = self.get_velocity()
        ang_vel = self.get_angular_velocity()
        
        self.ros_manager.publish_odometry(
            pos=pos,
            quat=quat,
            vel_linear=vel,
            vel_angular=ang_vel
        )
```

**通过 cmd_vel 控制：**
```bash
# 方式 1：直接发布话题
ros2 topic pub /robot_0/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {z: 0.5}}"

# 方式 2：使用键盘遥控
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args --remap cmd_vel:=/robot_0/cmd_vel

# 方式 3：使用 joystick
ros2 launch teleop_twist_joy teleop-launch.py \
  joy_config:='xbox' \
  --ros-args --remap cmd_vel:=/robot_0/cmd_vel
```

**执行技能：**
```bash
# 导航到目标点
ros2 action send_goal /robot_0/skill_execution plan_msgs/action/SkillExecution \
  '{skill_request: {
    skill_list: [{
      skill: "navigate_to",
      params: [
        {key: "goal_pos", value: "[10, 20, 0]"},
        {key: "timeout", value: "30.0"}
      ]
    }]
  }}' --feedback

# 跟随路径
ros2 action send_goal /robot_0/skill_execution plan_msgs/action/SkillExecution \
  '{skill_request: {
    skill_list: [{
      skill: "follow_path",
      params: [
        {key: "waypoints", value: "[[0,0,0], [5,5,0], [10,0,0]]"}
      ]
    }]
  }}' --feedback
```

**监控状态：**
```bash
# 查看 Odometry
ros2 topic echo /robot_0/odom

# 查看可用的 Action
ros2 action list

# 查看 Action 信息
ros2 action info /robot_0/skill_execution

# 查看所有话题
ros2 topic list | grep robot_0
```

#### 7.9 设计优点

**1. 完全解耦：**
- Robot 类不包含任何 ROS 代码
- ROS Manager 可以独立启动和停止
- 仿真层和 ROS 层完全分离

**2. 独立线程：**
- ROS 通信不阻塞仿真循环
- 多个机器人的 ROS 节点并行运行
- 优雅的启动和关闭机制

**3. 统一接口：**
- 所有机器人使用相同的 ROS Manager 结构
- 标准的话题命名规范（`/namespace/topic`）
- 一致的 Action 接口

**4. 易于扩展：**
```python
# 添加新的 Publisher
def _create_publishers(self):
    # 现有的 publishers
    if "odom" in self.topics:
        self.publisher_odom = self.create_publisher(...)
    
    # 添加新的 publisher
    if "imu" in self.topics:
        self.publisher_imu = self.create_publisher(
            Imu, self.topics["imu"], 10
        )

# 添加新的 Subscriber
def _create_subscribers(self):
    # 现有的 subscribers
    if "cmd_vel" in self.topics:
        self.subscriber_cmd_vel = self.create_subscription(...)
    
    # 添加新的 subscriber
    if "goal" in self.topics:
        self.subscriber_goal = self.create_subscription(
            PoseStamped, self.topics["goal"], self.callback_goal, 10
        )
```

**5. 灵活的话题配置：**
```python
# 最小配置
topics = {
    'odom': '/robot_0/odom',
}

# 完整配置
topics = {
    'odom': '/robot_0/odom',
    'cmd_vel': '/robot_0/cmd_vel',
    'imu': '/robot_0/imu',
    'camera': '/robot_0/camera/image_raw',
    'lidar': '/robot_0/scan',
}
```

#### 7.10 与其他系统的集成

**与 Skill System 集成：**
```python
# Skill Manager 通过 ROS Action 触发
# NodeRobot 的 execute_callback_wrapper 处理请求
def execute_callback_wrapper(self, goal_handle):
    """处理技能执行请求"""
    request = goal_handle.request.skill_request
    task_name = request.skill_list[0].skill
    params = self._parse_params(request.skill_list[0].params)
    
    # 使用 Robot 的 SkillManager
    skill_manager = self.robot_instance.skill_manager
    
    # 执行技能
    while goal_handle.is_active:
        result = skill_manager.execute_skill(task_name, **params)
        # 发布反馈和检查完成状态
        ...
```

**与 MPC 控制器集成：**
```python
# MPC 通过 clock 回调自动触发
class NodeMpcController(Node):
    def __init__(self, namespace: str, robot=None):
        self.robot = robot  # 直接引用 robot
        
        # 订阅仿真时钟
        self.subscriber_sim_clock = self.create_subscription(
            Clock,
            '/isaacsim_simulation_clock',
            self.clock_callback,
            10
        )
    
    def clock_callback(self, msg: Clock):
        """每次 world.tick() 后自动调用"""
        self.control_loop()
    
    def control_loop(self):
        """计算并直接设置速度"""
        optimal_command = self.mpc_controller.solve(...)
        
        if self.robot:
            # 直接设置（同步，无延迟）
            self.robot.target_linear_velocity = torch.tensor([...])
            self.robot.target_angular_velocity = torch.tensor([...])
```

#### 7.11 常见问题

**问题 1：ROS 节点启动失败**
```python
# 确保 rclpy 已初始化
rclpy.init(args=None)

# 确保在创建 ROS Manager 之前创建了 robot
robot = robot_actors[0].robot

# 确保调用了 start()
ros_manager.start()
```

**问题 2：cmd_vel 不起作用**
```python
# 检查话题配置
topics = {
    'cmd_vel': f'/{robot.namespace}/cmd_vel',  # 必须包含
}

# 检查话题是否正确
ros2 topic list | grep cmd_vel

# 测试发布
ros2 topic pub /robot_0/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 1.0}}" --once
```

**问题 3：多机器人命名冲突**
```python
# 确保每个机器人有唯一的 namespace
for i, robot in enumerate(robots):
    ros_manager = RobotRosManager(
        robot=robot,
        namespace=f'robot_{i}',  # 唯一的命名空间
        topics={
            'odom': f'/robot_{i}/odom',
            'cmd_vel': f'/robot_{i}/cmd_vel',
        }
    )
```

***



### 4. 控制流程（解耦设计）

**Robot 层（on_physics_step）：**
```python
def on_physics_step(self, step_size):
    # 1. 从 Isaac Sim 读取状态，更新 _position, _quat, _linear_velocity, _angular_velocity
    self.publish_robot_state()
    
    # 2. 更新相机视野
    self._update_camera_view()
    
    # 3. 将 target_linear_velocity, target_angular_velocity 应用到 Isaac Sim
    # Note: target_linear_velocity 由 MPC (Application 层) 通过 clock 回调设置
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
    """MPC 计算并直接设置 robot.target_linear_velocity"""
    optimal_command = self.mpc_controller.solve(...)

    if self.robot:
        self.robot.target_linear_velocity = torch.tensor([...])
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
            self.robot.target_linear_velocity = torch.tensor([
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

---

### 8. 分层边界 - Application 层不调用 Isaac Sim API

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
# 直接设置 robot.target_linear_velocity
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
