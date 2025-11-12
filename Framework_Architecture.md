# Isaac Sim 机器人仿真框架架构文档

## 📐 架构概览

本框架采用三层架构设计，实现了清晰的职责分离和模块解耦：

```
┌─────────────────────────────────────────────────────┐
│           Application Layer (应用层)                 │
│  • Skill System (技能系统)                           │
│  • ROS Bridge (ROS桥接)                             │
└─────────────────────────────────────────────────────┘
                        ↓
┌─────────────────────────────────────────────────────┐
│          Simulation Layer (仿真层)                   │
│  • World (世界管理)                                  │
│  • Actor System (角色系统)                           │
│  • Blueprint (蓝图系统)                              │
└─────────────────────────────────────────────────────┘
                        ↓
┌─────────────────────────────────────────────────────┐
│            Isaac Sim (物理引擎层)                    │
│  • Physics Engine (物理引擎)                         │
│  • Rendering (渲染)                                  │
└─────────────────────────────────────────────────────┘
```

---

## 📊 第一部分：类图与核心组件

### 1. ConfigManager (配置管理器)

**职责**：统一管理所有配置文件的加载和路径解析

**核心属性**：
- `config: Dict` - 配置字典
- `config_path: Path` - 配置文件路径

**核心方法**：
```python
def load() -> None
    # 加载 YAML 配置文件
    
def get(key: str) -> Any
    # 支持点状访问，如 'world.name'
    
def _derive_paths() -> None
    # 自动计算项目根目录和场景路径
```

**关键特性**：
- 自动计算项目根目录（消除硬编码）
- 动态搜索 asset 目录下的场景文件
- 支持嵌套配置访问

---

### 2. World (世界管理器)

**职责**：管理仿真世界，提供 CARLA 风格的统一 API

**核心属性**：
- `_isaac_world: IsaacWorld` - Isaac Sim 世界实例
- `_actors: Dict[int, Actor]` - Actor 注册表
- `_blueprint_library: BlueprintLibrary` - 蓝图库
- `_scene_manager: SceneManager` - 场景管理器

**核心方法**：
```python
def spawn_actor(blueprint, transform) -> Actor
    # 统一的 Actor 创建接口（CARLA 风格）
    
def load_actors_from_config(path: str) -> List[Actor]
    # 从 YAML 配置批量创建 Actor
    
def tick()
    # 仿真步进（每帧调用）
    
def get_blueprint_library() -> BlueprintLibrary
    # 获取蓝图库
```

**设计模式**：
- 使用 Blueprint 的 tags 判断类型（而非类名）
- 统一返回 Actor 对象（RobotActor 或 StaticActor）
- 自动注册所有创建的 Actor

---

### 3. Blueprint System (蓝图系统)

**职责**：提供类似 CARLA 的对象创建机制

**Blueprint 类**：
```python
class Blueprint:
    id: str                    # 类型标识符，如 'robot.jetbot'
    robot_class: Type          # 机器人类（如果是机器人）
    tags: List[str]            # 类型标签，如 ['robot', 'wheeled']
    _attributes: Dict          # 属性字典
```

**BlueprintLibrary 类**：
- 预注册所有机器人类型（jetbot, h1, g1, cf2x 等）
- 预注册静态物体类型（box, car 等）
- 提供 `find()` 和 `filter()` 方法查找蓝图

**使用示例**：
```python
# 获取蓝图
bp = world.get_blueprint_library().find('robot.jetbot')

# 配置属性
bp.set_attribute('namespace', 'robot_0')
bp.set_attribute('position', [0, 0, 0.5])

# 创建 Actor
actor = world.spawn_actor(bp, transform)
```

**设计优势**：
- 使用 tags 系统灵活判断类型
- 属性系统支持运行时配置
- 易于扩展新类型

---

### 4. Actor System (角色系统)

**Actor 基类**：
```python
class Actor:
    _actor_id: int             # 唯一 ID
    _world: World              # 所属世界
    _prim_path: str            # USD 路径
    
    def get_id() -> int
    def get_type_id() -> str
    def get_transform() -> Transform
    def get_velocity() -> Vector3D
    def destroy()
```

**RobotActor (动态角色)**：
```python
class RobotActor(Actor):
    robot: Robot               # 引用 Robot 实例
    
    # 双向引用：robot.actor ↔ actor.robot
```

**StaticActor (静态角色)**：
```python
class StaticActor(Actor):
    _semantic_label: str       # 语义标签
    
    # 用于静态物体（障碍物、道具等）
```

**设计优势**：
- 统一接口，无论动态还是静态
- 双向引用便于访问
- 自动注册到 World

---

### 5. Robot (机器人类)

**职责**：管理单个机器人的状态和控制

**核心设计：状态与命令分离**

**状态变量（私有，只读）**：
```python
_position: Tensor              # 实际位置
_quat: Tensor                  # 实际姿态
_linear_velocity: Tensor       # 实际线速度
_angular_velocity: Tensor      # 实际角速度
```

**命令变量（公共，可写）**：
```python
target_linear_velocity: Tensor   # 目标线速度
target_angular_velocity: Tensor  # 目标角速度
```

**核心方法**：
```python
def get_world_pose() -> Tuple[Tensor, Tensor]
    # 返回缓存的位置和姿态（不调用 Isaac Sim API）
    
def get_velocity() -> Tensor
    # 返回实际速度（状态）
    
def apply_control(control: RobotControl)
    # 应用控制命令
    
def on_physics_step(step_size)
    # 物理步进回调
    # 1. publish_robot_state() - 更新状态
    # 2. controller_simplified() - 应用命令
```

**关键设计**：
- 状态变量由 Isaac Sim 更新，只读
- 命令变量由控制器设置，可写
- 避免状态覆盖命令的问题

---

### 6. SkillManager (技能管理器)

**职责**：管理机器人技能的注册和执行

**核心属性**：
```python
_global_skills: Dict[str, Callable]  # 类变量：全局技能注册表
robot: Robot                          # 机器人实例
skills: Dict[str, Callable]           # 实例技能
skill_states: Dict                    # 技能状态
skill_data: Dict                      # 技能数据
```

**核心方法**：
```python
@classmethod
def register(skill_name: str)
    # 装饰器：注册技能到全局注册表
    
def execute_skill(skill_name: str, **kwargs) -> Dict
    # 执行技能（状态机模式）
    
def register_all_global_skills()
    # 从全局注册表注册所有技能
```

**使用示例**：
```python
# 注册技能
@SkillManager.register()
def navigate_to(robot, goal_pos, **kwargs):
    # 技能实现
    pass

# 创建管理器
skill_manager = SkillManager(robot, auto_register=True)

# 执行技能
result = skill_manager.execute_skill('navigate_to', goal_pos=[10, 20, 0])
```

**设计优势**：
- 全局注册表，所有机器人共享技能定义
- 装饰器语法简洁
- 状态机模式支持多步骤技能

---

### 7. ROS Manager System (ROS 管理系统)

#### 7.1 RosManagerIsaac (全局 ROS 管理器)

**职责**：管理全局 ROS 节点（场景监控、任务执行等）

**核心属性**：
```python
executor: MultiThreadedExecutor    # ROS 执行器
node: Dict[str, Node]              # 节点字典
thread: Thread                     # ROS 线程
stop_event: Event                  # 停止事件
```

**核心方法**：
```python
def build_nodes()
    # 根据配置构建 ROS 节点
    
def start()
    # 启动 ROS 线程
    
def stop()
    # 停止 ROS 线程
```

#### 7.2 RobotRosManager (机器人 ROS 管理器)

**职责**：管理单个机器人的所有 ROS 基础设施

**核心属性**：
```python
robot: Robot                       # 机器人实例
namespace: str                     # 命名空间
topics: Dict                       # 话题配置

# ROS 节点
node: NodeRobot                    # 主节点
node_planner_ompl: NodePlannerOmpl # 路径规划
node_trajectory_generator: Node    # 轨迹生成
node_controller_mpc: NodeMpcController # MPC 控制器

# 执行器和线程
executor: MultiThreadedExecutor
ros_thread: Thread
stop_event: Event
```

**核心方法**：
```python
def _init_ros_nodes()
    # 初始化所有 ROS 节点
    
def _init_executor()
    # 创建执行器并添加节点
    
def start()
    # 启动独立的 ROS 线程
    
def stop()
    # 优雅关闭 ROS 线程
```

**ROS 通信**：
- **Publishers**: `/robot_0/odom` (里程计)
- **Subscribers**: `/robot_0/cmd_vel` (速度命令), `/sim_clock` (仿真时钟)
- **Action Servers**: `/robot_0/skill_execution` (技能执行)
- **Action Clients**: `/robot_0/compute_path_to_pose` (路径规划)

**设计优势**：
- 完全解耦：Robot 类不包含 ROS 代码
- 独立线程：不阻塞仿真循环
- 统一接口：所有机器人使用相同结构
- 易于扩展：灵活的话题配置

---

## 🔄 第二部分：时序逻辑与数据流

### 流程 1：系统初始化流程

**参与者**：
- `main_example.py` - 主程序入口
- `ConfigManager` - 配置管理器
- `DI Container` - 依赖注入容器
- `World` - 世界管理器
- `BlueprintLibrary` - 蓝图库

**执行步骤**：

**阶段 1：加载配置**
1. 主程序调用 `ConfigManager.load()`
2. 读取 `config/config_parameter.yaml` 文件
3. 调用 `_derive_paths()` 自动计算项目根目录
4. 使用 `Path(__file__).parent.parent` 获取项目根路径
5. 动态搜索 `asset/*.json` 文件，查找场景 USD 路径
6. 将配置存储到 `config` 字典中
7. 返回配置给主程序

**阶段 2：容器初始化**
1. 主程序调用 `reset_container()` 重置容器
2. 调用 `get_container()` 获取 DI 容器实例
3. 容器自动创建所有服务实例：
   - World
   - SceneManager
   - RosManagerIsaac
   - GridMap
   - SemanticMap
   - ViewportManager
4. 返回配置好的 World 实例

**阶段 3：蓝图库初始化**
1. 主程序调用 `world.get_blueprint_library()`
2. World 创建 `BlueprintLibrary` 实例
3. BlueprintLibrary 调用 `_register_default_robots()`
4. 预注册所有机器人类型：
   - robot.jetbot
   - robot.h1
   - robot.g1
   - robot.cf2x
   - robot.drone_autel
   - robot.target
5. 调用 `_register_static_props()`
6. 预注册静态物体类型：
   - static.prop.box
   - static.prop.car
7. 返回配置好的蓝图库

**关键设计**：
- 配置路径自动计算，消除硬编码
- 依赖注入实现松耦合
- 蓝图预注册支持快速查找

---

### 流程 2：机器人创建流程

**参与者**：
- `main_example.py` - 主程序
- `World` - 世界管理器
- `Blueprint` - 蓝图对象
- `Robot Class` - 具体机器人类（如 RobotJetbot）
- `RobotActor` - 机器人角色包装器

**执行步骤**：

**阶段 1：加载配置文件**
1. 主程序调用 `world.load_actors_from_config('robot_swarm_cfg.yaml')`
2. World 读取 YAML 配置文件
3. 解析机器人列表（类型、命名空间、位置等）

**阶段 2：为每个机器人创建 Actor（循环）**

**步骤 2.1：查找蓝图**
1. World 调用 `blueprint_library.find('robot.jetbot')`
2. BlueprintLibrary 从预注册的蓝图中查找
3. 返回对应的 Blueprint 对象

**步骤 2.2：配置蓝图属性**
1. World 调用 `blueprint.set_attribute('namespace', 'robot_0')`
2. 设置机器人命名空间
3. 调用 `blueprint.set_attribute('position', [0, 0, 0.5])`
4. 设置初始位置
5. 设置其他属性（orientation, enable_lidar 等）

**步骤 2.3：创建 Actor**
1. World 调用 `spawn_actor(blueprint, transform)`
2. 检查 `blueprint.has_tag('robot')` 判断类型
3. 调用 `_spawn_robot(blueprint, transform)`

**步骤 2.4：实例化 Robot**
1. 从 blueprint 获取 `robot_class`（如 RobotJetbot）
2. 从 blueprint 获取所有属性作为 `cfg_robot`
3. 调用 `RobotJetbot(cfg_robot)` 实例化机器人
4. Robot 内部初始化 `_body` (BodyRobot)
5. 将 robot 添加到 Isaac Sim 场景

**步骤 2.5：创建 RobotActor 包装器**
1. 创建 `RobotActor(robot, world)`
2. 建立双向引用：`robot.actor = actor`
3. 调用 `world.register_actor(actor)`
4. 分配唯一的 actor_id
5. 将 actor 添加到 `world._actors` 字典

**步骤 2.6：返回结果**
1. 返回 `robot_actor` 给主程序
2. 主程序提取 Robot 对象：`robots = [actor.robot for actor in robot_actors]`

**关键设计**：
- 使用 Blueprint 统一创建接口
- 基于 tags 判断类型，灵活扩展
- 双向引用便于访问
- 自动注册到 World

---

### 流程 3：Skill 和 ROS 注入流程

**参与者**：
- `main_example.py` - 主程序
- `Robot` - 机器人实例
- `RobotRosManager` - 机器人 ROS 管理器
- `SkillManager` - 技能管理器
- `NodeRobot` - ROS 主节点

**执行步骤**：

**阶段 1：ROS 注入**

**步骤 1.1：创建 ROS Manager**
1. 主程序创建 `RobotRosManager(robot, namespace, topics)`
2. 传入机器人实例、命名空间和话题配置

**步骤 1.2：初始化 ROS 节点**
1. RosManager 调用 `_init_ros_nodes()`
2. 创建 `NodeRobot(namespace, topics)` - 主节点
3. 创建 `NodePlannerOmpl(namespace)` - 路径规划器
4. 创建 `NodeTrajectoryGenerator(namespace)` - 轨迹生成器
5. 创建 `NodeMpcController(namespace, robot=robot)` - MPC 控制器
6. 注意：MPC 控制器直接持有 robot 引用

**步骤 1.3：初始化执行器**
1. RosManager 调用 `_init_executor()`
2. 创建 `MultiThreadedExecutor`
3. 将所有节点添加到执行器：
   - executor.add_node(node)
   - executor.add_node(node_planner_ompl)
   - executor.add_node(node_trajectory_generator)
   - executor.add_node(node_controller_mpc)

**步骤 1.4：注入到 Robot**
1. 主程序调用 `robot.set_ros_manager(ros_manager)`
2. Robot 存储：`self.ros_manager = ros_manager`

**步骤 1.5：启动 ROS 线程**
1. 主程序调用 `ros_manager.start()`
2. RosManager 创建独立的 ROS 线程
3. 线程中运行 `_spin_ros()` 方法
4. 循环调用 `executor.spin_once(timeout_sec=0.05)`
5. ROS 线程在后台持续运行，不阻塞主循环

**阶段 2：Skill 注入**

**步骤 2.1：创建 Skill Manager**
1. 主程序创建 `SkillManager(robot, auto_register=True)`
2. 传入机器人实例和自动注册标志

**步骤 2.2：注册全局技能**
1. SkillManager 调用 `register_all_global_skills()`
2. 从类变量 `_global_skills` 复制所有技能
3. 这些技能是通过 `@SkillManager.register()` 装饰器注册的
4. 包括：navigate_to, follow_path, explore 等

**步骤 2.3：注入到 Robot**
1. 主程序调用 `robot.skill_manager = skill_manager`
2. Robot 存储：`self.skill_manager = skill_manager`
3. Robot 现在可以执行所有注册的技能

**阶段 3：物理回调注入**

**步骤 3.1：注册物理回调**
1. 主程序调用 `world.get_isaac_world().add_physics_callback()`
2. 传入回调名称：`'physics_step_robot_0'`
3. 传入回调函数：`robot.on_physics_step`
4. Isaac Sim 将在每个物理步进时调用此函数

**关键设计**：
- ROS 完全解耦，通过依赖注入
- 独立的 ROS 线程，不阻塞仿真
- Skill 全局注册，所有机器人共享
- 物理回调自动触发状态更新和控制

---

### 流程 4：仿真 Tick 机制（完整控制循环）

**参与者**：
- `Main Loop` - 主循环
- `World` - 世界管理器
- `Isaac Sim` - 物理引擎
- `Robot` - 机器人实例
- `NodeMpcController` - MPC 控制器
- `ROS Thread` - ROS 线程

**执行步骤**：

**阶段 1：主循环触发**
1. 主循环调用 `world.tick()`
2. World 调用 `isaac_world.step(render=True)`
3. Isaac Sim 执行物理仿真步进
4. Isaac Sim 发布仿真时钟到 `/isaacsim_simulation_clock`

**阶段 2：ROS 线程并行运行（MPC 控制）**

**步骤 2.1：时钟回调触发**
1. ROS 线程中，MPC 节点订阅了 `/isaacsim_simulation_clock`
2. 收到时钟消息，触发 `clock_callback()`
3. 更新 `latest_sim_time`

**步骤 2.2：MPC 计算控制**
1. clock_callback 调用 `control_loop()`
2. MPC 控制器调用 `mpc_controller.solve(...)`
3. 计算最优控制命令（线速度和角速度）

**步骤 2.3：直接设置目标速度**
1. MPC 直接设置 `robot.target_linear_velocity = torch.tensor([...])`
2. MPC 直接设置 `robot.target_angular_velocity = torch.tensor([...])`
3. **关键**：这是同步设置，无延迟
4. 不通过 ROS topic，避免异步延迟

**阶段 3：物理回调执行**

**步骤 3.1：触发物理回调**
1. Isaac Sim 调用注册的物理回调 `robot.on_physics_step(step_size)`

**步骤 3.2：更新状态（publish_robot_state）**
1. Robot 调用 `publish_robot_state()`
2. 调用 `_body.get_world_pose()` 从 Isaac Sim 读取位置和姿态
3. Isaac Sim 返回 `position, quat`
4. 调用 `_body.get_world_vel()` 从 Isaac Sim 读取速度
5. Isaac Sim 返回 `velocity, angular_velocity`
6. 更新状态变量：
   - `self._position = pos`
   - `self._quat = quat`
   - `self._linear_velocity = vel`
   - `self._angular_velocity = ang_vel`
7. **关键**：不更新命令变量 `target_linear_velocity` 和 `target_angular_velocity`
8. 调用 `ros_manager.publish_odometry(...)` 发布里程计到 ROS
9. ROS 线程发布 `/robot_0/odom` 话题

**步骤 3.3：应用命令（controller_simplified）**
1. Robot 调用 `controller_simplified()`
2. 调用 `_body.set_linear_velocities(target_linear_velocity)`
3. 调用 `_body.set_angular_velocities(target_angular_velocity)`
4. **关键**：应用的是 MPC 在本帧开始时设置的目标速度
5. Isaac Sim 更新机器人的速度

**阶段 4：循环继续**
1. Isaac Sim 步进完成
2. World 返回 `tick()` 完成
3. 主循环继续下一帧

**关键设计**：
- **状态与命令分离**：状态变量只读，命令变量可写
- **同步控制**：MPC 直接设置速度，不通过 ROS topic
- **完全解耦**：Robot 层不知道 MPC 的存在
- **无延迟**：命令在同一帧内计算和应用
- **并行执行**：ROS 线程和主循环并行运行

**数据流总结**：
```
1. world.tick() 
   → Isaac Sim 步进 
   → 发布 /sim_clock

2. ROS 线程（并行）
   → MPC.clock_callback() 
   → control_loop() 
   → robot.target_velocity = ... （同步设置）

3. robot.on_physics_step()
   → publish_robot_state() 
      → 读取 Isaac Sim 状态
      → 更新 _position, _velocity（状态变量）
      → 发布 /odom
   → controller_simplified()
      → 应用 target_velocity（命令变量）
      → 写入 Isaac Sim
```

---

### 流程 5：技能执行流程（navigate_to 示例）

**参与者**：
- `用户/ROS Client` - 发起请求
- `Action Server` - ROS Action 服务器
- `SkillManager` - 技能管理器
- `NodePlannerOmpl` - OMPL 路径规划器
- `NodeTrajectoryGenerator` - 轨迹生成器
- `NodeMpcController` - MPC 控制器
- `Robot` - 机器人实例

**执行步骤**：

**阶段 1：用户发起请求**
1. 用户通过 ROS Action 发送技能执行请求
2. 话题：`/robot_0/skill_execution`
3. 消息类型：`plan_msgs/action/SkillExecution`
4. 参数：
   - skill: 'navigate_to'
   - goal_pos: [10, 20, 0]

**阶段 2：Action Server 接收请求**
1. NodeRobot 的 Action Server 接收请求
2. 调用 `execute_callback_wrapper(goal_handle)`
3. 解析请求参数
4. 调用 `skill_manager.execute_skill('navigate_to', goal_pos=[10, 20, 0])`

**阶段 3：技能执行（状态机模式）**

**状态：INIT（初始化）**
1. SkillManager 检查技能状态，发现是首次调用
2. 设置状态为 INIT
3. 获取机器人当前位置：`robot.get_world_pose()`
4. 发送路径规划请求到 NodePlannerOmpl
5. 请求参数：
   - start: 当前位置
   - goal: [10, 20, 0]
6. NodePlannerOmpl 使用 OMPL 算法计算路径
7. 考虑障碍物和机器人尺寸
8. 返回路径点列表：`[[0,0,0], [2,3,0], [5,8,0], [10,20,0]]`
9. SkillManager 存储路径到 `skill_data`
10. 设置状态为 PLANNING_DONE

**状态：PLANNING_DONE（规划完成）**
1. SkillManager 检查状态为 PLANNING_DONE
2. 发送轨迹生成请求到 NodeTrajectoryGenerator
3. 传入路径点列表
4. NodeTrajectoryGenerator 生成带时间戳的轨迹
5. 考虑速度和加速度限制
6. 生成平滑的速度曲线
7. 返回轨迹：`[(t0, pos0, vel0), (t1, pos1, vel1), ...]`
8. SkillManager 将轨迹设置到 NodeMpcController
9. MPC 控制器存储参考轨迹
10. 设置状态为 EXECUTING

**状态：EXECUTING（执行中）**
1. SkillManager 每帧检查执行状态
2. 获取机器人当前位置：`robot.get_world_pose()`
3. 计算到目标的距离
4. 检查是否到达目标（距离 < 阈值）

**并行：MPC 自动运行**
1. MPC 控制器在后台自动运行（通过 clock_callback）
2. 跟踪参考轨迹
3. 计算最优控制命令
4. 直接设置 `robot.target_velocity`
5. Robot 在 `on_physics_step` 中应用速度

**反馈发布**
1. SkillManager 构造反馈消息：
   - status: 'processing'
   - progress: 计算完成百分比
   - distance: 到目标的距离
2. 调用 `goal_handle.publish_feedback(feedback)`
3. Action Server 发布反馈到 ROS
4. 用户收到实时反馈

**循环检查**
1. 每帧重复上述步骤
2. 直到到达目标或超时

**状态：SUCCEEDED（成功）**
1. SkillManager 检测到到达目标
2. 设置状态为 SUCCEEDED
3. 构造成功结果：
   - status: 'succeeded'
   - message: 'Reached goal'
4. 返回结果给 Action Server
5. Action Server 调用 `goal_handle.succeed()`
6. 用户收到 Action 完成通知

**关键设计**：
- 状态机模式支持多步骤异步操作
- MPC 自动跟踪轨迹，无需手动控制
- 实时反馈让用户了解执行进度
- 完全通过 ROS Action 接口，标准化

---

## 🚀 总结

### 核心优势

1. **清晰的三层架构**
   - Application 层：技能和 ROS 桥接
   - Simulation 层：统一的 CARLA 风格 API
   - Isaac Sim 层：物理引擎和渲染

2. **完全解耦的设计**
   - Robot 不包含 ROS 代码
   - Application 层不调用 Isaac Sim API
   - 通过依赖注入实现功能扩展

3. **灵活的 Blueprint 系统**
   - 统一的对象创建接口
   - 基于 tags 的类型判断
   - 易于扩展新类型

4. **强大的 ROS 集成**
   - 独立的 ROS 线程
   - 完整的导航栈支持
   - 灵活的话题配置

5. **状态与命令分离**
   - 避免状态覆盖命令
   - 同步控制，无延迟
   - 清晰的数据流

### 适用场景

- 多机器人仿真
- 导航和路径规划
- 技能学习和测试
- ROS 算法验证
- 机器人协同任务

---

**文档版本**：v2.0  
**最后更新**：2024年  
**作者**：Framework Team
