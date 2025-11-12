# Isaac Sim 机器人仿真框架 - 完整架构文档

**版本**: v7.0  
**最后更新**: 2024年11月12日  
**状态**: ✅ 生产就绪

---

## 📑 目录

1. [架构概览](#架构概览)
2. [核心设计原则](#核心设计原则)
3. [核心组件](#核心组件)
4. [坐标系统](#坐标系统)
5. [传感器系统](#传感器系统)
6. [ROS 集成](#ros-集成)
7. [技能系统](#技能系统)
8. [性能优化](#性能优化)
9. [CARLA 对齐](#carla-对齐)
10. [使用示例](#使用示例)
11. [故障排查](#故障排查)

---

## 架构概览

### 三层架构设计

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

## 核心设计原则

### 1. CARLA 风格 API
- 统一的 `spawn_actor()`, `apply_control()` 接口
- Blueprint 系统管理所有可创建对象
- Sensor 通过 `.listen()` 模式获取数据

### 2. 分层边界清晰
- Application 层不直接调用 Isaac Sim API
- 通过 Actor 和 Control 对象进行交互
- 数据类型统一转换（torch → Python）

### 3. 状态与命令分离
- Robot 状态变量只读（由 Isaac Sim 更新）
- 命令变量可写（由控制器设置）
- 避免状态覆盖命令的问题

### 4. 同步控制
- MPC 直接设置速度，避免 ROS 延迟
- 控制器与仿真同步运行
- 保证实时性

### 5. 模块化设计
- 每个传感器独立管理
- 技能系统可插拔
- 易于扩展和维护

---

## 核心组件

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

---

### 4. Transform System (坐标变换系统)

**核心类**:
```python
class Location:
    def __init__(self, x=0.0, y=0.0, z=0.0)
    
class Rotation:
    def __init__(self, quaternion=None, order="xyzw")
    def to_quaternion() -> List[float]  # [x, y, z, w]
    
class Transform:
    def __init__(self, location=None, rotation=None)
```

**类型转换**:
- 自动处理 torch tensor、numpy array、Python list
- 统一转换为 Python 原生类型
- 支持 wxyz 和 xyzw 四元数顺序

**设计特点**:
- ✅ 强制显式设置（location/rotation 可以为 None）
- ✅ Fail-fast 原则（使用 None 属性会立即报错）
- ✅ 类型安全（自动转换数据类型）

---

## 坐标系统

### 坐标系约定

**Isaac Sim / ROS 标准**:
- X: 前方
- Y: 左侧
- Z: 上方
- 右手坐标系

### 四元数格式

**Isaac Sim 格式**: `(w, x, y, z)`  
**scipy/ROS 格式**: `(x, y, z, w)`

**转换**:
```python
# Isaac Sim → scipy
quat_scipy = [quat[1], quat[2], quat[3], quat[0]]

# scipy → Isaac Sim
quat_isaac = [quat[3], quat[0], quat[1], quat[2]]
```

### LiDAR 坐标变换

**完整变换链**:
```
1. LiDAR 局部坐标系
   ↓ (应用 LiDAR 的 quat)
2. 父对象（机器人）局部坐标系  
   ↓ (应用父对象的世界位姿)
3. 世界坐标系 (map)
```

**实现位置**:
- `simulation/sensor/lidar/lidar_omni.py` - Omni LiDAR 局部旋转
- `simulation/sensor/lidar_actor.py` - 父对象变换
- `ros/sensor_ros_bridge.py` - ROS 坐标系转换

**关键修复**:
1. ✅ LiDAR 点云应用局部旋转
2. ✅ RobotActor 返回完整 Transform（包含 rotation）
3. ✅ 数据类型保持 float32（避免 scipy 的 float64）

---

## 传感器系统

### 传感器架构

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
│   ├── camera.py              # Camera 实现
│   └── cfg_camera.py          # CfgCamera
│
├── lidar_actor.py              # LidarIsaacSensor, LidarOmniSensor
└── lidar/
    ├── lidar_blueprint.py     # IsaacLidarBlueprint, OmniLidarBlueprint
    ├── lidar_isaac.py         # LidarIsaac 实现
    ├── lidar_omni.py          # LidarOmni 实现
    └── cfg_lidar.py           # CfgLidar
```

### LiDAR 实现

**两种 LiDAR 类型**:

| 类型 | Blueprint ID | 底层 API | 数据格式 |
|------|-------------|---------|---------|
| Isaac LiDAR | `sensor.lidar.isaac` | Isaac Sim LidarRtx | 字典 (distances, emitterIds) |
| Omni LiDAR | `sensor.lidar.omni` | Omni RTX LiDAR | 点云数组 [N, 3] |

**使用示例**:
```python
# Isaac LiDAR
isaac_lidar_bp = bp_library.find('sensor.lidar.isaac')
isaac_lidar_bp.set_attribute('config_file_name', 'Hesai_XT32_SD10')
isaac_lidar_bp.set_attribute('frequency', 10)

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
omni_lidar_bp.set_attribute('frequency', 10)

omni_lidar = world.spawn_actor(
    omni_lidar_bp,
    Transform(Location(x=0.0, z=0.1)),
    attach_to=robot_actor
)
```

### 传感器访问模式

**CARLA 风格设计**:
- 传感器是独立的 Actor
- 不存储在 Robot 类中
- 通过 World 查询获取

**查询方法**:
```python
# 方式 1: 通过 Robot 查询（推荐）
camera = robot.get_sensor_by_type('sensor.camera.rgb')

# 方式 2: 通过 World 查询
sensors = world.find_sensors_by_parent(robot_actor)

# 方式 3: 直接引用（创建时保存）
camera = world.spawn_actor(camera_bp, transform, attach_to=robot_actor)
```

---

## ROS 集成

### RobotRosManager

**职责**: 管理机器人的所有 ROS 通信

**核心组件**:
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

### Sensor ROS Bridge

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
lidar = world.spawn_actor(lidar_bp, transform, attach_to=robot_actor)

# 附加到 ROS
robot = robot_actor.robot
ros_manager = robot.get_ros_manager()
ros_manager.attach_sensor_to_ros(lidar, 'lidar', 'front_lidar/points')
# 发布到: /robot_0/front_lidar/points
```

**支持的传感器类型**:
- **LiDAR**: `sensor_msgs/PointCloud2` → `/robot_0/lidar/points`
- **Camera**: `sensor_msgs/Image` → `/robot_0/camera/image_raw`

---

## 技能系统

### 状态机架构

**基于有限状态机（FSM）的技能管理**:

```
M = (S, Σ, δ, s₀, F)

S: 状态集合
Σ: 输入符号集合（事件/条件）
δ: 状态转换函数
s₀: 初始状态
F: 终止状态集合
```

### Navigate To 技能示例

**状态定义**:
```
S = {INITIALIZING, EXECUTING, COMPLETED, FAILED}
```

**状态转换**:
```
None --[request_received]--> INITIALIZING
INITIALIZING --[planning_succeeded]--> EXECUTING
EXECUTING --[goal_reached]--> COMPLETED
EXECUTING --[execution_timeout]--> FAILED
```

**实现特点**:
1. **异步路径规划**: 使用 ROS Action Future
2. **并行 MPC 控制**: 状态机监控，MPC 执行
3. **事件驱动**: 通过事件信号判断到达

### Take Photo 技能示例

**使用方式**:
```bash
ros2 action send_goal /h1_0/skill_execution \
  plan_msgs/action/SkillExecution \
  '{skill_request: {skill_list: [{
    skill: "take_photo",
    params: [
      {key: "camera_type", value: "sensor.camera.rgb"},
      {key: "save_path", value: "/path/to/photo.jpg"}
    ]
  }]}}' --feedback
```

---

## 性能优化

### LiDAR 频率控制

**问题**: 默认 60Hz 运行导致 CPU 占用过高

**解决方案**: 配置频率参数

```python
lidar_bp.set_attribute('frequency', 5)  # 5Hz - 推荐用于调试
lidar_bp.set_attribute('frequency', 10)  # 10Hz - 默认值
lidar_bp.set_attribute('frequency', 20)  # 20Hz - 高频率
```

**推荐频率**:

| 场景 | 推荐频率 | CPU 占用 |
|------|---------|---------|
| 调试/开发 | 5 Hz | ~8% |
| 一般使用 | 10 Hz | ~17% |
| 高精度需求 | 20 Hz | ~33% |

### 数据类型优化

**问题**: scipy 默认返回 float64，导致数据类型提升

**解决方案**: 强制使用 float32

```python
# 旋转矩阵
rotation_matrix = rotation.as_matrix().astype(np.float32)

# 平移向量
translation = np.array([pos.x, pos.y, pos.z], dtype=np.float32)

# 保持原始数据类型
return points_transformed.astype(original_dtype)
```

---

## CARLA 对齐

### CARLA 风格说明

**我们参考 CARLA 的设计理念，但实现是全新的**:

| CARLA 类 | 我们的类 | 说明 |
|---------|---------|------|
| `carla.World` | `simulation.World` | ✅ 参考 API 设计 |
| `carla.BlueprintLibrary` | `simulation.BlueprintLibrary` | ✅ 参考 API 设计 |
| `carla.Actor` | `simulation.Actor` | ✅ 参考 API 设计 |
| `carla.Image` | `simulation.sensor.CameraData` | ✅ 我们自己创建 |
| `carla.LidarMeasurement` | `simulation.sensor.LidarData` | ✅ 我们自己创建 |

**关键点**:
- ✅ API 风格参考 CARLA
- ✅ 实现完全独立
- ✅ 适配 Isaac Sim
- ✅ 用户体验一致

### 未来优化方向

**高优先级**:
1. **Control 系统重构** - 使用 Control 对象而非直接设置速度
2. **Skill 系统简化** - 提取状态机基类，减少样板代码

**中优先级**:
3. **Vehicle/Robot 类型分离** - 区分 Vehicle, Walker, Drone
4. **传感器数据流优化** - 添加数据缓存和按需获取

---

## 使用示例

### 完整示例：创建机器人和传感器

```python
from simulation import World, Transform, Location, Rotation

# 1. 创建世界
world = World(simulation_app)

# 2. 获取蓝图库
bp_library = world.get_blueprint_library()

# 3. 创建机器人
robot_bp = bp_library.find('robot.jetbot')
robot_bp.set_attribute('namespace', 'robot_0')

robot_actor = world.spawn_actor(
    robot_bp,
    Transform(location=Location(0, 0, 0.5))
)

# 4. 添加相机
camera_bp = bp_library.find('sensor.camera.rgb')
camera_bp.set_attribute('image_size_x', 1280)
camera_bp.set_attribute('image_size_y', 720)

camera = world.spawn_actor(
    camera_bp,
    Transform(Location(x=0.2, z=0.1)),
    attach_to=robot_actor
)

# 5. 添加 LiDAR
lidar_bp = bp_library.find('sensor.lidar.omni')
lidar_bp.set_attribute('config_file_name', 'Hesai_XT32_SD10')
lidar_bp.set_attribute('output_size', (352, 120))
lidar_bp.set_attribute('frequency', 10)

lidar = world.spawn_actor(
    lidar_bp,
    Transform(Location(x=0.0, z=0.05)),
    attach_to=robot_actor
)

# 6. 监听数据
camera.listen(lambda img: img.save_to_disk(f'frame_{img.frame}.png'))
lidar.listen(lambda data: print(f"Points: {len(data.points)}"))

# 7. 附加到 ROS
robot = robot_actor.robot
ros_manager = robot.get_ros_manager()
ros_manager.attach_sensor_to_ros(camera, 'camera', 'camera/image')
ros_manager.attach_sensor_to_ros(lidar, 'lidar', 'lidar/points')

# 8. 运行仿真
while simulation_app.is_running():
    world.tick()
```

---

## 故障排查

### LiDAR 点云方向错误

**症状**: 点云在 RViz 中上下颠倒或旋转

**诊断工具**:
```python
from debug_lidar_coordinate import analyze_lidar_data

lidar.listen(analyze_lidar_data)
```

**解决方案**:
```python
# 调整坐标转换参数
ros_manager.attach_sensor_to_ros(
    lidar, 'lidar', 'lidar/points',
    flip_z=True,      # 翻转 Z 轴
    rotate_z_deg=-90  # 旋转 -90 度
)
```

### 传感器未找到

**症状**: `Camera 'sensor.camera.rgb' not found`

**检查方法**:
```python
# 列出所有传感器
sensors = robot.get_sensors()
for sensor in sensors:
    print(f"Sensor: {sensor.get_type_id()}")
```

**解决方案**: 确保传感器已创建并附加到机器人

### 仿真卡顿

**原因**: LiDAR 频率过高

**解决方案**:
```python
# 降低 LiDAR 频率
lidar_bp.set_attribute('frequency', 5)  # 从 60Hz 降到 5Hz
```

---

## 文件结构

```
simulation/                     # 仿真层
├── world.py                   # World 类
├── actor.py                   # Actor 基类
├── robot_actor.py             # RobotActor
├── static_actor.py            # StaticActor
├── actor_blueprint.py         # Blueprint 系统
├── transform.py               # Transform, Location, Rotation
├── control.py                 # RobotControl
│
└── sensors/                   # 传感器系统
    ├── sensor.py              # SensorActor 基类
    ├── data.py                # CameraData, LidarData
    ├── camera_actor.py        # RGBCamera
    ├── lidar_actor.py         # LidarSensor
    │
    ├── camera/
    │   ├── camera_blueprint.py
    │   ├── camera.py
    │   └── cfg_camera.py
    │
    └── lidar/
        ├── lidar_blueprint.py
        ├── lidar_isaac.py
        ├── lidar_omni.py
        └── cfg_lidar.py

robot/                          # 机器人实现
├── robot.py                   # Robot 基类
├── robot_jetbot.py
├── robot_h1.py
└── body/

application/                    # 应用层
├── skill_manager.py
└── skills/

ros/                           # ROS 集成
├── ros_manager_robot.py
├── node_robot.py
└── sensor_ros_bridge.py

docs/                          # 文档
├── FRAMEWORK_ARCHITECTURE.md  # 本文档
├── README.md                  # 文档导航
└── ...
```

---

## 参考文档

### 核心文档
- **`docs/FRAMEWORK_ARCHITECTURE.md`** (本文档) - 完整架构
- **`docs/README.md`** - 文档导航

### 详细文档
- **`docs/SENSOR_ACCESS_PATTERN.md`** - 传感器访问模式
- **`docs/LIDAR_IMPLEMENTATION.md`** - LiDAR 实现细节
- **`docs/SENSOR_ROS_BRIDGE.md`** - Sensor ROS 桥接
- **`docs/LIDAR_COORDINATE_TRANSFORM.md`** - LiDAR 坐标转换
- **`docs/LIDAR_LOCAL_ROTATION_FIX.md`** - LiDAR 局部旋转修复
- **`docs/PERFORMANCE_OPTIMIZATION.md`** - 性能优化指南
- **`docs/TAKE_PHOTO_SKILL_GUIDE.md`** - Take Photo 技能指南
- **`docs/navigate_to_skill_architecture.md`** - Navigate To 技能架构
- **`docs/state_machine_design_for_paper.md`** - 状态机设计（学术）
- **`docs/CARLA_STYLE_EXPLANATION.md`** - CARLA 风格说明
- **`docs/CARLA_ALIGNMENT_ROADMAP.md`** - CARLA 对齐路线图

### 示例代码
- **`main_example.py`** - 完整使用示例
- **`debug_lidar_coordinate.py`** - LiDAR 坐标调试工具

---

## 版本历史

### v7.0 (2024-11-12)
- ✅ 整合所有文档到单一架构文档
- ✅ 添加完整的坐标系统说明
- ✅ 添加性能优化指南
- ✅ 添加故障排查章节

### v6.0 (2024-11-12)
- ✅ 完善 Transform 系统（支持 None 值）
- ✅ 修复 LiDAR 坐标转换
- ✅ 优化数据类型处理（float32）

### v5.0 (2024-11-11)
- ✅ 添加 Sensor ROS Bridge
- ✅ 完善 LiDAR 实现
- ✅ 添加频率控制

---

**文档版本**: v7.0  
**最后更新**: 2024年11月12日  
**维护者**: Framework Team  
**状态**: ✅ 生产就绪
