# 文档导航

## 📖 核心文档

### 🏗️ [FRAMEWORK_ARCHITECTURE.md](FRAMEWORK_ARCHITECTURE.md)
**框架完整架构文档** - 唯一必读文档

包含内容：
- 三层架构设计
- 核心组件说明 (World, Actor, Sensor, ROS)
- 坐标系统和变换
- 传感器系统（Camera, LiDAR）
- ROS 集成和 Sensor Bridge
- 技能系统和状态机
- 性能优化指南
- CARLA 对齐说明
- 完整使用示例
- 故障排查指南

**适合**: 所有开发者

---

## 🚀 快速开始

1. **阅读** [FRAMEWORK_ARCHITECTURE.md](FRAMEWORK_ARCHITECTURE.md) 了解整体架构
2. **查看** `main_example.py` 运行完整示例
3. **使用** `debug_lidar_coordinate.py` 调试 LiDAR 坐标系

---

## 📊 文档结构

```
docs/
├── README.md                          # 本文档 - 导航
└── FRAMEWORK_ARCHITECTURE.md          # ⭐ 完整架构文档（唯一核心文档）
```

---

## 🔧 工具脚本

### debug_lidar_coordinate.py
LiDAR 坐标系调试工具

**使用方法**:
```python
from debug_lidar_coordinate import analyze_lidar_data

lidar.listen(analyze_lidar_data)
```

**功能**:
- 分析点云坐标范围
- 诊断坐标系问题
- 检测 Z 轴倒置
- 检测旋转问题

---

## 📝 示例代码

### main_example.py
完整的框架使用示例

**包含**:
- 创建世界和机器人
- 添加传感器（Camera, LiDAR）
- ROS 集成
- 技能执行
- 仿真循环

---

## 🎯 常见任务快速参考

### 创建机器人
```python
from simulation import World, Transform, Location

world = World(simulation_app)
bp_library = world.get_blueprint_library()

robot_bp = bp_library.find('robot.jetbot')
robot_actor = world.spawn_actor(robot_bp, Transform(location=Location(0, 0, 0.5)))
```

### 添加传感器
```python
# Camera
camera_bp = bp_library.find('sensor.camera.rgb')
camera = world.spawn_actor(camera_bp, Transform(Location(x=0.2, z=0.1)), attach_to=robot_actor)

# LiDAR
lidar_bp = bp_library.find('sensor.lidar.omni')
lidar_bp.set_attribute('frequency', 10)
lidar = world.spawn_actor(lidar_bp, Transform(Location(x=0.0, z=0.05)), attach_to=robot_actor)
```

### 发布到 ROS
```python
robot = robot_actor.robot
ros_manager = robot.get_ros_manager()
ros_manager.attach_sensor_to_ros(camera, 'camera', 'camera/image')
ros_manager.attach_sensor_to_ros(lidar, 'lidar', 'lidar/points')
```

### 执行技能
```bash
# Navigate To
ros2 action send_goal /robot_0/skill_execution plan_msgs/action/SkillExecution \
  '{skill_request: {skill_list: [{skill: "navigate_to", params: [{key: "goal_pos", value: "[10, 20, 0]"}]}]}}' --feedback

# Take Photo
ros2 action send_goal /robot_0/skill_execution plan_msgs/action/SkillExecution \
  '{skill_request: {skill_list: [{skill: "take_photo", params: [{key: "save_path", value: "/path/to/photo.jpg"}]}]}}' --feedback
```

---

## 🐛 故障排查

### LiDAR 点云方向错误
```python
# 使用调试工具
from debug_lidar_coordinate import analyze_lidar_data
lidar.listen(analyze_lidar_data)

# 调整坐标转换
ros_manager.attach_sensor_to_ros(lidar, 'lidar', 'lidar/points', flip_z=True, rotate_z_deg=-90)
```

### 传感器未找到
```python
# 列出所有传感器
sensors = robot.get_sensors()
for sensor in sensors:
    print(f"Sensor: {sensor.get_type_id()}")
```

### 仿真卡顿
```python
# 降低 LiDAR 频率
lidar_bp.set_attribute('frequency', 5)  # 从 60Hz 降到 5Hz
```

---

## 📚 相关资源

### 外部文档
- [Isaac Sim 官方文档](https://docs.omniverse.nvidia.com/isaacsim/latest/index.html)
- [CARLA 文档](https://carla.readthedocs.io/)
- [ROS 2 文档](https://docs.ros.org/)

### 代码仓库
- 主仓库: `simulation/` - 仿真层
- 机器人: `robot/` - 机器人实现
- 应用层: `application/` - 技能系统
- ROS 集成: `ros/` - ROS 桥接

---

**维护者**: Framework Team  
**最后更新**: 2024年11月12日  
**版本**: v7.0
