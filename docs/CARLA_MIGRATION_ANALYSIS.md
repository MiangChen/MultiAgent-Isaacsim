# CARLA 架构迁移分析

## 📊 当前进度总览

### ✅ 已完成的核心组件

| CARLA 组件 | 我们的实现 | 完成度 | 说明 |
|-----------|-----------|--------|------|
| `carla.Client` | `simulation.Server` | ✅ 100% | 管理 simulation_app |
| `carla.World` | `simulation.World` | ✅ 95% | 核心功能完成 |
| `carla.Actor` | `simulation.Actor` | ✅ 90% | 基础功能完成 |
| `carla.Vehicle` | `simulation.RobotActor` | ✅ 90% | 机器人封装 |
| `carla.Transform` | `simulation.Transform` | ✅ 100% | 完整实现 |
| `carla.Location` | `simulation.Location` | ✅ 100% | 完整实现 |
| `carla.Rotation` | `simulation.Rotation` | ✅ 100% | 完整实现 |
| `carla.Blueprint` | `simulation.Blueprint` | ✅ 95% | 核心功能完成 |
| `carla.BlueprintLibrary` | `simulation.BlueprintLibrary` | ✅ 95% | 核心功能完成 |
| `carla.VehicleControl` | `simulation.RobotControl` | ✅ 100% | 完整实现 |

### ⚠️ 部分完成的组件

| CARLA 组件 | 我们的实现 | 完成度 | 缺失功能 |
|-----------|-----------|--------|---------|
| `carla.Sensor` | - | ❌ 0% | 传感器基类 |
| `carla.SensorData` | - | ❌ 0% | 传感器数据类 |
| `carla.Image` | - | ❌ 0% | 图像数据 |
| `carla.LidarMeasurement` | - | ❌ 0% | 点云数据 |
| `carla.World.on_tick()` | - | ❌ 0% | 回调机制 |
| `carla.Actor.destroy()` | - | ⚠️ 30% | 生命周期管理 |
| `carla.World.get_actors()` | `world.get_actors()` | ⚠️ 50% | 需要完善 |

---

## 🎯 核心差异分析

### 1. Actor 生命周期管理

**CARLA 的方式：**
```python
# 创建
vehicle = world.spawn_actor(blueprint, transform)

# 查询
actors = world.get_actors()
vehicles = world.get_actors().filter('vehicle.*')

# 销毁
vehicle.destroy()

# 检查存活
if vehicle.is_alive:
    pass
```

**我们当前的实现：**
```python
# 创建 ✅
robot = world.spawn_actor(blueprint, transform)

# 查询 ⚠️ 需要实现
# actors = world.get_actors()  # 未实现
# robots = world.get_actors().filter('robot.*')  # 未实现

# 销毁 ❌ 未实现
# robot.destroy()  # 未实现

# 检查存活 ❌ 未实现
# if robot.is_alive:  # 未实现
```

**需要实现：**
- [ ] `world.get_actors()` - 返回 ActorList
- [ ] `ActorList.filter(wildcard)` - 过滤 actors
- [ ] `actor.destroy()` - 销毁 actor
- [ ] `actor.is_alive` - 检查存活状态
- [ ] `actor.id` - 唯一标识符

---

### 2. 传感器系统

**CARLA 的传感器架构：**
```python
# 1. 获取传感器 blueprint
camera_bp = blueprint_library.find('sensor.camera.rgb')
camera_bp.set_attribute('image_size_x', '800')
camera_bp.set_attribute('image_size_y', '600')

# 2. 创建传感器（附加到 vehicle）
camera = world.spawn_actor(camera_bp, transform, attach_to=vehicle)

# 3. 监听数据
def process_image(image):
    array = np.frombuffer(image.raw_data, dtype=np.uint8)
    array = array.reshape((image.height, image.width, 4))
    # 处理图像

camera.listen(process_image)

# 4. 停止监听
camera.stop()

# 5. 销毁传感器
camera.destroy()
```

**我们当前的实现：**
```python
# ❌ 没有统一的传感器接口
# 相机和 lidar 是直接在机器人类中创建的
# 没有 listen() 回调机制
# 没有统一的数据格式
```

**需要实现：**
- [ ] `simulation.Sensor` - 传感器基类
- [ ] `simulation.Camera` - 相机传感器
- [ ] `simulation.Lidar` - 激光雷达传感器
- [ ] `sensor.listen(callback)` - 数据回调
- [ ] `sensor.stop()` - 停止监听
- [ ] `SensorData` - 传感器数据基类
- [ ] `ImageData` - 图像数据
- [ ] `PointCloudData` - 点云数据

---

### 3. World 查询和管理

**CARLA 的 World API：**
```python
# 获取所有 actors
actors = world.get_actors()

# 过滤特定类型
vehicles = world.get_actors().filter('vehicle.*')
pedestrians = world.get_actors().filter('walker.*')

# 通过 ID 查找
actor = world.get_actor(actor_id)

# 获取观察者（相机）
spectator = world.get_spectator()

# 设置天气
world.set_weather(carla.WeatherParameters.ClearNoon)

# 获取地图
map = world.get_map()

# Tick 回调
def on_world_tick(snapshot):
    print(f"Frame: {snapshot.frame}")

world.on_tick(on_world_tick)
```

**我们当前的实现：**
```python
# ✅ world.spawn_actor() - 已实现
# ✅ world.tick() - 已实现
# ❌ world.get_actors() - 未实现
# ❌ world.get_actor(id) - 未实现
# ❌ world.on_tick(callback) - 未实现
# ⚠️ world.get_blueprint_library() - 已实现
```

**需要实现：**
- [ ] `world.get_actors()` - 返回所有 actors
- [ ] `world.get_actor(actor_id)` - 通过 ID 查找
- [ ] `world.on_tick(callback)` - Tick 回调
- [ ] `ActorList` - Actor 列表类
- [ ] `ActorList.filter(wildcard)` - 过滤功能

---

### 4. Actor 属性和方法

**CARLA 的 Actor API：**
```python
# 基础属性
actor.id                    # 唯一 ID
actor.type_id              # 类型 ID（如 'vehicle.tesla.model3'）
actor.is_alive             # 是否存活
actor.attributes           # 属性字典

# Transform 相关
transform = actor.get_transform()
actor.set_transform(transform)
location = actor.get_location()
actor.set_location(location)

# 速度相关
velocity = actor.get_velocity()
angular_velocity = actor.get_angular_velocity()
acceleration = actor.get_acceleration()

# 控制
actor.set_simulate_physics(True)
actor.set_enable_gravity(True)

# 附加
actor.attach_to(parent_actor)

# 销毁
actor.destroy()
```

**我们当前的实现：**
```python
# ⚠️ robot.namespace - 类似 ID
# ❌ robot.id - 未实现
# ❌ robot.type_id - 未实现
# ❌ robot.is_alive - 未实现
# ⚠️ robot.get_transform() - 部分实现
# ❌ robot.set_transform() - 未实现
# ⚠️ robot.get_velocity() - 部分实现
# ❌ robot.destroy() - 未实现
```

**需要实现：**
- [ ] `actor.id` - 唯一标识符
- [ ] `actor.type_id` - 类型标识
- [ ] `actor.is_alive` - 存活状态
- [ ] `actor.get_transform()` - 获取变换
- [ ] `actor.set_transform()` - 设置变换
- [ ] `actor.get_location()` - 获取位置
- [ ] `actor.set_location()` - 设置位置
- [ ] `actor.get_velocity()` - 获取速度
- [ ] `actor.destroy()` - 销毁 actor

---

## 📋 优先级规划

### 🔥 高优先级（核心功能）

#### 1. Actor 生命周期管理（2-3 天）
```python
# 目标 API
actors = world.get_actors()
robot = world.get_actor(robot_id)
robot.destroy()
if robot.is_alive:
    pass
```

**实现步骤：**
1. 在 `World` 中维护 actor 列表
2. 实现 `get_actors()` 返回 `ActorList`
3. 实现 `ActorList.filter(wildcard)`
4. 实现 `actor.destroy()`
5. 实现 `actor.is_alive` 属性

**文件修改：**
- `simulation/world.py` - 添加 actor 管理
- `simulation/actor.py` - 添加生命周期方法
- 新建 `simulation/actor_list.py` - ActorList 类

#### 2. Actor 属性和方法（2-3 天）
```python
# 目标 API
transform = actor.get_transform()
actor.set_transform(transform)
location = actor.get_location()
velocity = actor.get_velocity()
```

**实现步骤：**
1. 实现 `actor.get_transform()`
2. 实现 `actor.set_transform()`
3. 实现 `actor.get_location()` / `set_location()`
4. 实现 `actor.get_velocity()`
5. 添加 `actor.id` 和 `actor.type_id`

**文件修改：**
- `simulation/actor.py` - 添加方法
- `simulation/robot_actor.py` - 实现具体逻辑

---

### 🔴 中优先级（重要功能）

#### 3. 传感器系统（1-2 周）
```python
# 目标 API
camera_bp = blueprint_library.find('sensor.camera.rgb')
camera = world.spawn_actor(camera_bp, transform, attach_to=robot)
camera.listen(lambda image: process(image))
camera.stop()
```

**实现步骤：**
1. 设计传感器基类 `Sensor`
2. 实现 `Camera` 传感器
3. 实现 `Lidar` 传感器
4. 实现 `listen()` 回调机制
5. 实现传感器数据类

**文件创建：**
- `simulation/sensor.py` - 传感器基类
- `simulation/sensor_data.py` - 数据类
- `simulation/camera.py` - 相机传感器
- `simulation/lidar.py` - 激光雷达传感器

#### 4. World 回调机制（3-5 天）
```python
# 目标 API
def on_tick(snapshot):
    print(f"Frame: {snapshot.frame}")

world.on_tick(on_tick)
```

**实现步骤：**
1. 实现 `WorldSnapshot` 类
2. 在 `world.tick()` 中触发回调
3. 实现回调注册和管理

**文件修改：**
- `simulation/world.py` - 添加回调机制
- 新建 `simulation/snapshot.py` - WorldSnapshot 类

---

### 🟡 低优先级（增强功能）

#### 5. 碰撞检测（1 周）
```python
# 目标 API
collision_sensor = world.spawn_actor(collision_bp, attach_to=robot)
collision_sensor.listen(lambda event: on_collision(event))
```

#### 6. 录制和回放（2 周）
```python
# 目标 API
recorder = world.get_recorder()
recorder.start("recording.log")
recorder.stop()
recorder.replay("recording.log")
```

#### 7. 天气和环境（1 周）
```python
# 目标 API
world.set_weather(WeatherParameters.ClearNoon)
weather = world.get_weather()
```

---

## 🏗️ 实现路线图

### Phase 1: 核心 Actor 管理（1 周）
- [x] Blueprint 系统
- [x] spawn_actor()
- [ ] get_actors()
- [ ] ActorList.filter()
- [ ] actor.destroy()
- [ ] actor.is_alive

### Phase 2: Actor 属性完善（1 周）
- [ ] actor.id
- [ ] actor.type_id
- [ ] get_transform() / set_transform()
- [ ] get_location() / set_location()
- [ ] get_velocity()

### Phase 3: 传感器系统（2 周）
- [ ] Sensor 基类
- [ ] Camera 传感器
- [ ] Lidar 传感器
- [ ] listen() 回调
- [ ] SensorData 类

### Phase 4: World 增强（1 周）
- [ ] on_tick() 回调
- [ ] WorldSnapshot
- [ ] get_actor(id)

### Phase 5: 高级功能（持续）
- [ ] 碰撞检测
- [ ] 录制回放
- [ ] 天气系统

---

## 📊 完成度评估

### 整体进度
- **基础架构**: 90% ✅
- **Actor 系统**: 60% ⚠️
- **传感器系统**: 10% ❌
- **World 管理**: 70% ⚠️
- **高级功能**: 5% ❌

### 核心 CARLA 兼容性
- **API 设计**: 85% ✅
- **功能完整性**: 55% ⚠️
- **使用体验**: 75% ✅

---

## 🎯 近期目标（2-3 周）

### Week 1: Actor 生命周期
- [ ] 实现 `world.get_actors()`
- [ ] 实现 `ActorList` 和 `filter()`
- [ ] 实现 `actor.destroy()`
- [ ] 实现 `actor.is_alive`
- [ ] 添加 `actor.id` 和 `actor.type_id`

### Week 2: Actor 属性和方法
- [ ] 实现 `get_transform()` / `set_transform()`
- [ ] 实现 `get_location()` / `set_location()`
- [ ] 实现 `get_velocity()`
- [ ] 完善 Transform 相关方法

### Week 3: 传感器基础
- [ ] 设计传感器架构
- [ ] 实现 Sensor 基类
- [ ] 实现 Camera 传感器（基础版）
- [ ] 实现 listen() 回调机制

---

## 💡 设计建议

### 1. 保持 CARLA 兼容性
- API 命名和参数尽量与 CARLA 一致
- 但可以添加我们特有的功能（如技能系统）

### 2. 渐进式实现
- 先实现核心功能，再添加高级功能
- 每个功能都要有测试和文档

### 3. 保持简洁
- 不要过度设计
- 只实现真正需要的功能

### 4. 文档先行
- 每个新功能都要更新文档
- 提供清晰的使用示例

---

## 📚 参考资源

- [CARLA Documentation](https://carla.readthedocs.io/)
- [CARLA Python API](https://carla.readthedocs.io/en/latest/python_api/)
- [CARLA GitHub](https://github.com/carla-simulator/carla)

---

## 🎓 总结

### 已完成的核心工作
1. ✅ 基础架构（Server, World, Actor）
2. ✅ Blueprint 系统
3. ✅ Transform 数据类
4. ✅ RobotControl 控制系统
5. ✅ 基本的 spawn_actor()

### 最关键的缺失功能
1. ❌ Actor 生命周期管理（get_actors, destroy）
2. ❌ 传感器系统（Sensor, listen）
3. ❌ Actor 属性方法（get_transform, set_transform）
4. ❌ World 回调机制（on_tick）

### 建议的实施顺序
1. **先做 Actor 管理** - 这是最基础的功能
2. **再做 Actor 属性** - 完善 Actor 的可用性
3. **然后做传感器** - 这是重要但独立的系统
4. **最后做高级功能** - 根据实际需求决定

当前架构已经非常接近 CARLA 了，主要是需要补充一些管理和查询功能！
