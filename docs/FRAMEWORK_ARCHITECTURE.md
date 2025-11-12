# Isaac Sim 机器人仿真框架架构文档

**版本**: v6.0  
**最后更新**: 2024年11月12日  
**状态**: ✅ 生产就绪

---

## 📐 架构概览

本框架采用三层架构设计，参考 CARLA 架构，实现清晰的职责分离和模块解耦。

```
┌─────────────────────────────────────────────────────┐
│           Application Layer (应用层)                 │
│  • Skill System (技能系统)                           │
│  • ROS Bridge (ROS 桥接)                            │
│  • MPC Controller (模型预测控制)                     │
└─────────────────────────────────────────────────────┘
                        ↓
┌─────────────────────────────────────────────────────┐
│          Simulation Layer (仿真层)                   │
│  • World (世界管理)                                  │
│  • Actor System (角色系统)                           │
│  • Blueprint System (蓝图系统)                       │
│  • Sensor System (传感器系统)                        │
└─────────────────────────────────────────────────────┘
                        ↓
┌─────────────────────────────────────────────────────┐
│            Isaac Sim (物理引擎层)                    │
│  • Physics Engine (物理引擎)                         │
│  • Rendering (渲染)                                  │
│  • Sensors (传感器实现)                              │
└─────────────────────────────────────────────────────┘
```

---

## 🎯 核心设计原则

1. **CARLA 风格 API** - 统一的 `spawn_actor()`, `apply_control()` 接口
2. **分层边界清晰** - Application 层不直接调用 Isaac Sim API
3. **状态与命令分离** - 区分实际状态和控制命令
4. **同步控制** - MPC 直接设置速度，避免 ROS 延迟
5. **模块化设计** - 每个传感器独立管理

---

## 📦 第一部分：核心组件

### 1. World (世界管理器)

**职责**: 管理仿真世界，提供 CARLA 风格的统一 API

**核心方法**:
```python
# 创建 Actor
world.spawn_actor(blueprint, transform, attach_to=parent)

# 获取蓝图库
bp_library = world.get_blueprint_library()

# 仿真步进
world.tick()

# 从配置加载
actors = world.load_actors_from_config('config.yaml')
```

**设计特点**:
- 使用 Blueprint tags 判断类型
- 统一返回 Actor 对象
- 自动注册所有创建的 Actor

---

### 2. Blueprint System (蓝图系统)

**目录结构**:
```
simulation/
├── actor_blueprint.py          # ActorBlueprint, BlueprintLibrary
└── sensors/
    ├── sensor_blueprint_base.py    # SensorBlueprint 基类
    ├── camera/
    │   └── camera_blueprint.py     # RGBCameraBlueprint
    └── lidar/
        └── lidar_blueprint.py      # RayCastLidarBlueprint
```

**使用示例**:
```python
# 获取蓝图
bp_library = world.get_blueprint_library()
camera_bp = bp_library.find('sensor.camera.rgb')

# 配置属性
camera_bp.set_attribute('image_size_x', 1280)
camera_bp.set_attribute('image_size_y', 720)

# 创建 Actor
camera = world.spawn_actor(camera_bp, transform, attach_to=robot)
```

**Blueprint 类型**:
- `robot.*` - 机器人 (jetbot, h1, g1, cf2x, drone_autel)
- `static.prop.*` - 静态物体 (box, car)
- `sensor.camera.*` - 相机 (rgb, depth)
- `sensor.lidar.*` - LiDAR (isaac, omni)

---

### 3. Actor System (角色系统)

**类层次结构**:
```
Actor (基类)
├── RobotActor          # 机器人 Actor
├── StaticActor         # 静态物体 Actor
└── SensorActor         # 传感器 Actor 基类
    ├── RGBCamera       # RGB 相机
    ├── LidarIsaacSensor    # Isaac LiDAR 传感器
    └── LidarOmniSensor     # Omni LiDAR 传感器
```

**Actor 基类**:
```python
class Actor:
    def get_id() -> int
    def get_type_id() -> str
    def get_transform() -> Transform
    def set_transform(transform: Transform)
    def get_velocity() -> Vector3D
    def destroy()
```

**RobotActor**:
```python
class RobotActor(Actor):
    robot: Robot    # 双向引用: robot.actor ↔ actor.robot
```

**SensorActor**:
```python
class SensorActor(Actor):
    sensor: Any     # 传感器实现 (Camera, LidarIsaac)
    
    def listen(callback: Callable)
    def stop()
    def is_listening() -> bool
```

---

### 4. Sensor System (传感器系统)

**目录结构**:
```
simulation/sensors/
├── sensor.py                   # SensorActor 基类
├── sensor_blueprint_base.py    # SensorBlueprint 基类
├── data.py                     # CameraData, LidarData
│
├── camera_actor.py             # RGBCamera Actor
├── camera/
│   ├── camera_blueprint.py    # RGBCameraBlueprint
│   ├── camera.py              # Camera 实现 (Isaac Sim)
│   └── cfg_camera.py          # CfgCamera
│
├── lidar_actor.py              # LidarIsaacSensor, LidarOmniSensor
└── lidar/
    ├── lidar_blueprint.py     # IsaacLidarBlueprint, OmniLidarBlueprint
    ├── lidar_isaac.py         # LidarIsaac 实现 (Isaac Sim API)
    ├── lidar_omni.py          # LidarOmni 实现 (Omni API)
    └── cfg_lidar.py           # CfgLidar
```

**使用示例**:
```python
# 1. 获取蓝图
camera_bp = bp_library.find('sensor.camera.rgb')
camera_bp.set_attribute('image_size_x', 1280)

# 2. 创建传感器
camera = world.spawn_actor(
    camera_bp,
    Transform(Location(x=0.2, z=0.1)),
    attach_to=robot_actor
)

# 3. 监听数据
camera.listen(lambda img: img.save_to_disk(f'frame_{img.frame}.png'))
```

**设计特点**:
- 每个传感器有独立的文件夹
- Blueprint 和实现分离
- 延迟导入（Blueprint 不需要 Isaac Sim）
- 统一的 `listen()` 接口

---

### 5. Robot (机器人类)

**状态与命令分离**:

```python
class Robot:
    # 状态变量（私有，只读）
    _position: Tensor
    _quat: Tensor
    _linear_velocity: Tensor
    _angular_velocity: Tensor
    
    # 命令变量（公共，可写）
    target_linear_velocity: Tensor
    target_angular_velocity: Tensor
    
    # 公共接口
    def get_world_pose() -> Tuple[Tensor, Tensor]
    def get_velocity() -> Tensor
    def apply_control(control: RobotControl)
    def on_physics_step(step_size)
```

**关键设计**:
- 状态变量由 Isaac Sim 更新，只读
- 命令变量由控制器设置，可写
- 避免状态覆盖命令的问题

---

### 6. ROS Integration (ROS 集成)

**RobotRosManager**:
```python
class RobotRosManager:
    # ROS 节点
    node: NodeRobot
    node_planner_ompl: NodePlannerOmpl
    node_trajectory_generator: NodeTrajectoryGenerator
    node_controller_mpc: NodeMpcController
    
    # Sensor ROS bridges (CARLA style)
    sensor_bridges: Dict[int, SensorRosBridge]
    
    # 执行器和线程
    executor: MultiThreadedExecutor
    ros_thread: Thread
```

**ROS 通信**:
- **Publishers**: `/robot_0/odom`, `/robot_0/lidar/points`, `/robot_0/camera/image`
- **Subscribers**: `/robot_0/cmd_vel`, `/sim_clock`
- **Action Servers**: `/robot_0/skill_execution`
- **Action Clients**: `/robot_0/compute_path_to_pose`

**设计特点**:
- 完全解耦：Robot 类不包含 ROS 代码
- 独立线程：不阻塞仿真循环
- 统一接口：所有机器人使用相同结构
- CARLA 风格：通过 `.listen()` 连接传感器到 ROS

---

### 7. Sensor ROS Bridge (传感器 ROS 桥接)

**架构设计** (CARLA 风格):
```
[Simulation Layer]
    Sensor Actor (LidarIsaacSensor, RGBCamera)
    ↓ .listen(callback)
[Bridge Layer]
    SensorRosBridge (LidarRosBridge, CameraRosBridge)
    ↓ ROS publish
[ROS Layer]
    ROS Topics (/robot_0/lidar/points, /robot_0/camera/image)
```

**使用示例**:
```python
# 创建传感器 (CARLA style)
lidar_bp = bp_library.find('sensor.lidar.isaac')
lidar = world.spawn_actor(lidar_bp, transform, attach_to=robot_actor)

# 附加到 ROS (通过 RobotRosManager)
robot = robot_actor.robot
if robot.has_ros():
    ros_manager = robot.get_ros_manager()
    ros_manager.attach_sensor_to_ros(lidar, 'lidar', 'front_lidar/points')
    # 发布到: /robot_0/front_lidar/points
```

**支持的传感器类型**:
- **LiDAR**: `sensor_msgs/PointCloud2` → `/robot_0/lidar/points`
- **Camera**: `sensor_msgs/Image` → `/robot_0/camera/image_raw`

**设计优势**:
- ✅ 完全符合 CARLA 架构 (`.listen()` 模式)
- ✅ Sensor 不依赖 ROS (解耦)
- ✅ 可以同时有多个回调
- ✅ 动态添加/移除 ROS 发布

---

## 🔄 第二部分：数据流

### 1. 仿真 Tick 循环

```
主循环
  ↓ world.tick()
Isaac Sim 步进
  ↓ 发布 /sim_clock
MPC.clock_callback() (ROS 线程)
  ↓ control_loop()
  ↓ robot.target_velocity = ... (同步设置)
robot.on_physics_step()
  ↓ publish_robot_state() (更新状态)
  ↓ controller_simplified() (应用命令)
```

### 2. 传感器数据流

**Camera 数据流**:
```
用户代码
  ↓ camera.listen(callback)
RGBCamera.tick()
  ↓ self.sensor.get_rgb()
Camera (Isaac Sim)
  ↓ Isaac Sim API
Isaac Sim 渲染引擎
  ↓ 返回 RGB 数据
RGBCamera 构造 CameraData
  ↓ callback(camera_data)
用户回调函数
```

**LiDAR 数据流**:
```
用户代码
  ↓ lidar.listen(callback)
LidarIsaacSensor.tick()
  ↓ self.sensor.get_current_frame()
LidarIsaac (Isaac Sim)
  ↓ LidarRtx API
Isaac Sim RTX LiDAR
  ↓ 返回点云数据
LidarIsaacSensor 构造 LidarData
  ↓ callback(lidar_data)
用户回调函数
```

**Sensor ROS 发布流程**:
```
Sensor.tick()
  ↓ 获取数据
  ↓ 构造 SensorData
  ↓ callback(sensor_data)
SensorRosBridge.publish()
  ↓ 转换为 ROS 消息
  ↓ publisher.publish(msg)
ROS Topic
```

### 3. 技能执行流程

```
用户发起请求
  ↓ ROS Action: /robot_0/skill_execution
Skill 执行
  ↓ navigate_to() 发送路径规划请求
路径规划 (ROS)
  ↓ NodePlannerOmpl 计算路径
轨迹生成 (ROS)
  ↓ NodeTrajectoryGenerator 生成轨迹
MPC 自动跟踪
  ↓ 直接设置 robot.target_velocity
Robot 应用控制
  ↓ controller_simplified()
```

---

## 📁 第三部分：文件结构

### 核心目录

```
simulation/                     # 仿真层 (CARLA 风格)
├── __init__.py
├── server.py                  # Server 类
├── world.py                   # World 类
├── actor.py                   # Actor 基类
├── robot_actor.py             # RobotActor
├── static_actor.py            # StaticActor
├── sensor.py                  # SensorActor 基类
├── actor_blueprint.py         # ActorBlueprint, BlueprintLibrary
├── transform.py               # Transform, Location, Rotation
├── control.py                 # RobotControl
│
└── sensors/                   # 传感器系统
    ├── __init__.py
    ├── sensor_blueprint_base.py    # SensorBlueprint 基类
    ├── data.py                     # CameraData, LidarData
    ├── camera_actor.py             # RGBCamera Actor
    ├── lidar_actor.py              # LidarSensor Actor
    │
    ├── camera/
    │   ├── camera_blueprint.py    # RGBCameraBlueprint
    │   ├── camera.py              # Camera 实现
    │   └── cfg_camera.py          # CfgCamera
    │
    └── lidar/
        ├── lidar_blueprint.py     # RayCastLidarBlueprint
        ├── lidar_isaac.py         # LidarIsaac 实现
        └── cfg_lidar.py           # CfgLidar

robot/                          # 机器人实现
├── robot.py                   # Robot 基类
├── robot_jetbot.py            # RobotJetbot
├── robot_h1.py                # RobotH1
└── body/                      # Body 实现 (Isaac Sim 层)

application/                    # 应用层
├── skill_manager.py           # SkillManager
└── skills/                    # 技能实现

ros/                           # ROS 集成
├── ros_manager_robot.py       # RobotRosManager
├── node_robot.py              # NodeRobot
└── sensor_ros_bridge.py       # SensorRosBridge, LidarRosBridge, CameraRosBridge
```

---

## 🎨 第四部分：使用示例

### 1. 创建机器人

```python
from simulation import World, Transform, Location

# 创建世界
world = World(simulation_app)

# 获取蓝图库
bp_library = world.get_blueprint_library()

# 获取机器人蓝图
robot_bp = bp_library.find('robot.jetbot')
robot_bp.set_attribute('namespace', 'robot_0')

# 创建机器人
transform = Transform(location=Location(0, 0, 0.5))
robot_actor = world.spawn_actor(robot_bp, transform)

# 获取 Robot 对象
robot = robot_actor.robot
```

### 2. 添加传感器

**添加 Camera**:
```python
# 获取相机蓝图
camera_bp = bp_library.find('sensor.camera.rgb')
camera_bp.set_attribute('image_size_x', 1280)
camera_bp.set_attribute('image_size_y', 720)

# 创建相机（附加到机器人）
camera = world.spawn_actor(
    camera_bp,
    Transform(Location(x=0.2, z=0.1)),
    attach_to=robot_actor
)

# 监听数据
camera.listen(lambda img: img.save_to_disk(f'frame_{img.frame}.png'))
```

**添加 LiDAR**:
```python
# Isaac LiDAR
isaac_lidar_bp = bp_library.find('sensor.lidar.isaac')
isaac_lidar_bp.set_attribute('config_file_name', 'Hesai_XT32_SD10')
isaac_lidar = world.spawn_actor(
    isaac_lidar_bp,
    Transform(Location(x=0.0, z=0.05)),
    attach_to=robot_actor
)

# Omni LiDAR
omni_lidar_bp = bp_library.find('sensor.lidar.omni')
omni_lidar_bp.set_attribute('config_file_name', 'Hesai_XT32_SD10')
omni_lidar_bp.set_attribute('output_size', (352, 120))
omni_lidar_bp.set_attribute('erp_height', 352)
omni_lidar_bp.set_attribute('erp_width', 120)
omni_lidar = world.spawn_actor(
    omni_lidar_bp,
    Transform(Location(x=0.0, z=0.1)),
    attach_to=robot_actor
)

# 监听数据
isaac_lidar.listen(lambda data: print(f"Points: {len(data.points)}"))
```

**发布到 ROS**:
```python
# 获取 ROS manager
robot = robot_actor.robot
ros_manager = robot.get_ros_manager()

# 附加传感器到 ROS (CARLA style)
ros_manager.attach_sensor_to_ros(camera, 'camera', 'front_camera/image')
ros_manager.attach_sensor_to_ros(isaac_lidar, 'lidar', 'isaac_lidar/points')
ros_manager.attach_sensor_to_ros(omni_lidar, 'lidar', 'omni_lidar/points')

# 数据自动发布到:
# - /robot_0/front_camera/image
# - /robot_0/isaac_lidar/points
# - /robot_0/omni_lidar/points
```

### 3. ROS 控制

```python
from ros.ros_manager_robot import RobotRosManager

# 创建 ROS Manager
ros_manager = RobotRosManager(
    robot=robot,
    namespace='robot_0',
    topics={'odom': '/robot_0/odom', 'cmd_vel': '/robot_0/cmd_vel'}
)

# 注入到 Robot
robot.set_ros_manager(ros_manager)

# 启动 ROS
ros_manager.start()

# 通过 ROS 控制
# ros2 topic pub /robot_0/cmd_vel geometry_msgs/msg/Twist ...
```

### 4. 技能执行

```python
from application import SkillManager

# 创建 Skill Manager
skill_manager = SkillManager(robot, auto_register=True)
robot.skill_manager = skill_manager

# 执行技能
result = skill_manager.execute_skill(
    'navigate_to',
    goal_pos=[10, 20, 0],
    timeout=30.0
)
```

---

## 🚀 第五部分：最佳实践

### 1. 命名规范

**Blueprint ID**:
- 机器人: `robot.{type}` (e.g., `robot.jetbot`)
- 静态物体: `static.prop.{type}` (e.g., `static.prop.car`)
- 传感器: `sensor.{category}.{type}` (e.g., `sensor.camera.rgb`)

**类命名**:
- Blueprint: `{Type}Blueprint` (e.g., `RGBCameraBlueprint`)
- Actor: `{Type}Actor` 或 `{Type}` (e.g., `RobotActor`, `RGBCamera`)
- 实现: `{Type}` (e.g., `Camera`, `LidarIsaac`)

### 2. 导入规范

**用户代码**:

```python
# 从 simulation 导入
from simulation import World, Transform, Location
from simulation.sensor import RGBCamera, LidarSensor

# 从子包导入 Blueprint
from simulation.sensor.camera import RGBCameraBlueprint
from simulation.sensor.lidar import RayCastLidarBlueprint
```

**内部代码**:

```python
# 导入实现类
from simulation.sensor.camera.camera import Camera
from simulation.sensor.lidar.lidar_isaac import LidarIsaac
```

### 3. 错误处理

```python
# 传感器回调中的错误处理
def process_image(image):
    try:
        image.save_to_disk(f'frame_{image.frame}.png')
    except Exception as e:
        logger.error(f"Error saving image: {e}")

camera.listen(process_image)
```

### 4. 资源清理

```python
# 清理传感器
camera.stop()
camera.destroy()

# 清理 ROS
ros_manager.stop()

# 清理机器人
robot.cleanup()
```

---

## 📚 第六部分：参考文档

### 核心文档
- **`docs/FRAMEWORK_ARCHITECTURE.md`** (本文档) - 框架架构总览
- **`docs/SENSOR_ACCESS_PATTERN.md`** - 传感器访问模式详解
- **`docs/LIDAR_IMPLEMENTATION.md`** - LiDAR 实现细节
- **`docs/SENSOR_ROS_BRIDGE.md`** - Sensor ROS 桥接详解

### 快速参考
- **`docs/QUICK_REFERENCE.md`** - 传感器系统快速参考
- **`docs/NAMING_CONVENTIONS.md`** - 命名规范

### 迁移指南
- **`docs/MIGRATION_GUIDE.md`** - 从旧配置迁移

### 示例代码
- **`main_example.py`** - 完整使用示例
- **`examples/add_sensor_to_robot_example.py`** - 添加传感器示例

---

## ✅ 验证清单

### 系统验证
- [ ] 所有 Python 文件无语法错误
- [ ] 所有导入路径正确
- [ ] Blueprint 系统正常工作
- [ ] 传感器系统正常工作
- [ ] ROS 集成正常工作

### 代码质量
- [ ] 无重复代码
- [ ] 命名规范统一
- [ ] 文档完整
- [ ] 示例代码可运行

---

**文档版本**: v6.0  
**最后更新**: 2024年11月12日  
**维护者**: Framework Team  
**状态**: ✅ 生产就绪
