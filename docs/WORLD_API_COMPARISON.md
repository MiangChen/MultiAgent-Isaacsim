# World API 对比分析

## 📊 当前实现 vs CARLA World API

### ✅ 已实现的功能

| CARLA API | 我们的实现 | 状态 | 说明 |
|-----------|-----------|------|------|
| `world.tick()` | `world.tick()` | ✅ | 推进仿真一步 |
| `world.reset()` | `world.reset()` | ✅ | 重置世界 |
| `world.spawn_actor()` | `world.spawn_actor()` | ✅ | 创建 actor |
| `world.get_blueprint_library()` | `world.get_blueprint_library()` | ✅ | 获取 blueprint 库 |
| `world.get_actors()` | `world.get_actors()` | ✅ | 获取所有 actors |
| `world.get_actor(id)` | `world.get_actor(id)` | ✅ | 通过 ID 获取 actor |

### ⚠️ 部分实现的功能

| CARLA API | 我们的实现 | 完成度 | 缺失内容 |
|-----------|-----------|--------|---------|
| `world.get_actors().filter()` | - | ❌ 0% | ActorList 类和 filter 方法 |
| `world.on_tick(callback)` | - | ❌ 0% | Tick 回调机制 |
| `world.wait_for_tick()` | - | ❌ 0% | 同步等待 |
| `world.get_snapshot()` | - | ❌ 0% | WorldSnapshot 类 |

### ❌ 未实现的功能

| CARLA API | 说明 | 优先级 |
|-----------|------|--------|
| `world.get_spectator()` | 获取观察者相机 | 🟡 低 |
| `world.get_settings()` | 获取世界设置 | 🟡 低 |
| `world.apply_settings()` | 应用世界设置 | 🟡 低 |
| `world.get_weather()` | 获取天气 | 🟡 低 |
| `world.set_weather()` | 设置天气 | 🟡 低 |
| `world.get_map()` | 获取地图 | 🔴 中 |
| `world.get_lightstate()` | 获取灯光状态 | 🟡 低 |
| `world.freeze_all_traffic_lights()` | 冻结交通灯 | 🟢 不需要 |
| `world.get_random_location_from_navigation()` | 随机导航点 | 🟡 低 |

---

## 🎯 需要实现的核心功能

### 1. ActorList 和过滤功能 🔥

**CARLA 的实现：**
```python
# 获取所有 actors
actors = world.get_actors()  # 返回 ActorList

# 过滤特定类型
vehicles = world.get_actors().filter('vehicle.*')
pedestrians = world.get_actors().filter('walker.pedestrian.*')
traffic_lights = world.get_actors().filter('traffic.traffic_light')

# 通过 ID 查找
actor = world.get_actors().find(actor_id)

# 迭代
for actor in world.get_actors():
    print(actor.type_id)
```

**我们当前的实现：**
```python
# ✅ 基础功能
actors = world.get_actors()  # 返回 List[Actor]

# ❌ 缺失功能
# actors.filter('robot.*')  # 未实现
# actors.find(actor_id)      # 未实现
```

**需要实现：**

```python
# simulation/actor_list.py
class ActorList:
    """
    Actor 列表类（参考 CARLA）
    
    提供过滤、查找等功能
    """
    def __init__(self, actors: List['Actor']):
        self._actors = actors
    
    def filter(self, wildcard: str) -> 'ActorList':
        """
        使用通配符过滤 actors
        
        Examples:
            robots = actors.filter('robot.*')
            drones = actors.filter('robot.cf2x')
        """
        import fnmatch
        filtered = [
            actor for actor in self._actors 
            if fnmatch.fnmatch(actor.type_id, wildcard)
        ]
        return ActorList(filtered)
    
    def find(self, actor_id: int) -> Optional['Actor']:
        """通过 ID 查找 actor"""
        for actor in self._actors:
            if actor.id == actor_id:
                return actor
        return None
    
    def __iter__(self):
        return iter(self._actors)
    
    def __len__(self):
        return len(self._actors)
    
    def __getitem__(self, index):
        return self._actors[index]
```

**修改 World：**
```python
# simulation/world.py
from simulation.actor_list import ActorList

def get_actors(self) -> ActorList:
    """返回 ActorList 而不是 List"""
    return ActorList(list(self._actors.values()))
```

---

### 2. Tick 回调机制 🔥

**CARLA 的实现：**
```python
# 注册回调
def on_world_tick(world_snapshot):
    print(f"Frame: {world_snapshot.frame}")
    print(f"Timestamp: {world_snapshot.timestamp}")
    for actor_snapshot in world_snapshot:
        print(f"Actor {actor_snapshot.id}: {actor_snapshot.get_transform()}")

callback_id = world.on_tick(on_world_tick)

# 移除回调
world.remove_on_tick(callback_id)

# 等待下一个 tick
world_snapshot = world.wait_for_tick()
```

**我们当前的实现：**
```python
# ❌ 完全未实现
```

**需要实现：**

```python
# simulation/snapshot.py
class ActorSnapshot:
    """Actor 快照"""
    def __init__(self, actor_id: int, transform, velocity):
        self.id = actor_id
        self._transform = transform
        self._velocity = velocity
    
    def get_transform(self):
        return self._transform
    
    def get_velocity(self):
        return self._velocity


class WorldSnapshot:
    """
    World 快照（参考 CARLA）
    
    包含某一帧的所有 actor 状态
    """
    def __init__(self, frame: int, timestamp: float, actor_snapshots: List[ActorSnapshot]):
        self.frame = frame
        self.timestamp = timestamp
        self._actor_snapshots = {snap.id: snap for snap in actor_snapshots}
    
    def find(self, actor_id: int) -> Optional[ActorSnapshot]:
        return self._actor_snapshots.get(actor_id)
    
    def has_actor(self, actor_id: int) -> bool:
        return actor_id in self._actor_snapshots
    
    def __iter__(self):
        return iter(self._actor_snapshots.values())
    
    def __len__(self):
        return len(self._actor_snapshots)
```

**修改 World：**
```python
# simulation/world.py
from simulation.snapshot import WorldSnapshot, ActorSnapshot

class World:
    def __init__(self, ...):
        # ...
        self._tick_callbacks = {}
        self._next_callback_id = 1
        self._frame_count = 0
        self._start_time = 0
    
    def on_tick(self, callback) -> int:
        """
        注册 tick 回调
        
        Args:
            callback: 回调函数，接收 WorldSnapshot 参数
        
        Returns:
            callback_id: 回调 ID，用于移除
        """
        callback_id = self._next_callback_id
        self._next_callback_id += 1
        self._tick_callbacks[callback_id] = callback
        return callback_id
    
    def remove_on_tick(self, callback_id: int):
        """移除 tick 回调"""
        if callback_id in self._tick_callbacks:
            del self._tick_callbacks[callback_id]
    
    def tick(self):
        """推进仿真并触发回调"""
        # 执行物理步进
        self._isaac_world.step(render=True)
        self._frame_count += 1
        
        # 创建快照
        snapshot = self._create_snapshot()
        
        # 触发所有回调
        for callback in self._tick_callbacks.values():
            try:
                callback(snapshot)
            except Exception as e:
                print(f"Error in tick callback: {e}")
        
        return snapshot
    
    def _create_snapshot(self) -> WorldSnapshot:
        """创建当前帧的快照"""
        import time
        timestamp = time.time() - self._start_time
        
        actor_snapshots = []
        for actor in self.get_actors():
            if hasattr(actor, 'get_transform') and hasattr(actor, 'get_velocity'):
                snapshot = ActorSnapshot(
                    actor_id=actor.id,
                    transform=actor.get_transform(),
                    velocity=actor.get_velocity()
                )
                actor_snapshots.append(snapshot)
        
        return WorldSnapshot(self._frame_count, timestamp, actor_snapshots)
    
    def wait_for_tick(self, timeout: float = 10.0) -> WorldSnapshot:
        """
        等待下一个 tick（同步模式）
        
        Args:
            timeout: 超时时间（秒）
        
        Returns:
            WorldSnapshot: 下一帧的快照
        """
        import time
        start_time = time.time()
        current_frame = self._frame_count
        
        while time.time() - start_time < timeout:
            if self._frame_count > current_frame:
                return self._create_snapshot()
            time.sleep(0.001)
        
        raise TimeoutError("wait_for_tick timeout")
```

---

### 3. World 设置和配置 🔴

**CARLA 的实现：**
```python
# 获取设置
settings = world.get_settings()
print(f"Synchronous mode: {settings.synchronous_mode}")
print(f"Fixed delta seconds: {settings.fixed_delta_seconds}")

# 修改设置
settings.synchronous_mode = True
settings.fixed_delta_seconds = 0.05
world.apply_settings(settings)
```

**需要实现：**

```python
# simulation/world_settings.py
from dataclasses import dataclass

@dataclass
class WorldSettings:
    """
    World 设置（参考 CARLA）
    """
    synchronous_mode: bool = False
    fixed_delta_seconds: float = 0.0
    no_rendering_mode: bool = False
    substepping: bool = True
    max_substep_delta_time: float = 0.01
    max_substeps: int = 10
    
    def __eq__(self, other):
        if not isinstance(other, WorldSettings):
            return False
        return (
            self.synchronous_mode == other.synchronous_mode and
            self.fixed_delta_seconds == other.fixed_delta_seconds
        )
```

**修改 World：**
```python
# simulation/world.py
from simulation.world_settings import WorldSettings

class World:
    def __init__(self, ...):
        # ...
        self._settings = WorldSettings()
    
    def get_settings(self) -> WorldSettings:
        """获取世界设置"""
        return self._settings
    
    def apply_settings(self, settings: WorldSettings):
        """
        应用世界设置
        
        Args:
            settings: WorldSettings 对象
        """
        self._settings = settings
        
        # 应用到 Isaac Sim
        if settings.fixed_delta_seconds > 0:
            # 设置固定时间步长
            pass
        
        if settings.no_rendering_mode:
            # 禁用渲染
            pass
```

---

### 4. 地图访问 🔴

**CARLA 的实现：**
```python
# 获取地图
map = world.get_map()

# 地图信息
print(f"Map name: {map.name}")
waypoints = map.generate_waypoints(2.0)
spawn_points = map.get_spawn_points()

# 查询
location = carla.Location(x=10, y=20, z=0)
waypoint = map.get_waypoint(location)
```

**我们当前的实现：**
```python
# ⚠️ 有 grid_map 和 semantic_map，但不是 CARLA 风格
grid_map = world.get_grid_map()
semantic_map = world.get_semantic_map()
```

**需要实现：**

```python
# simulation/map.py
class Map:
    """
    地图类（简化版 CARLA Map）
    
    封装 grid_map 和 semantic_map
    """
    def __init__(self, name: str, grid_map, semantic_map):
        self.name = name
        self._grid_map = grid_map
        self._semantic_map = semantic_map
    
    def get_spawn_points(self) -> List['Transform']:
        """获取可用的生成点"""
        # 从 grid_map 获取空闲位置
        pass
    
    def get_waypoint(self, location) -> Optional['Waypoint']:
        """获取最近的路径点"""
        pass
    
    def generate_waypoints(self, distance: float) -> List['Waypoint']:
        """生成路径点"""
        pass
```

**修改 World：**
```python
# simulation/world.py
from simulation.map import Map

class World:
    def get_map(self) -> Map:
        """获取地图"""
        if self._grid_map and self._semantic_map:
            return Map("default", self._grid_map, self._semantic_map)
        return None
```

---

## 📋 实现优先级

### 🔥 高优先级（1-2 周）

1. **ActorList 和过滤** (2-3 天)
   - [ ] 创建 `ActorList` 类
   - [ ] 实现 `filter(wildcard)` 方法
   - [ ] 实现 `find(actor_id)` 方法
   - [ ] 修改 `world.get_actors()` 返回 `ActorList`

2. **Tick 回调机制** (3-5 天)
   - [ ] 创建 `WorldSnapshot` 类
   - [ ] 创建 `ActorSnapshot` 类
   - [ ] 实现 `world.on_tick(callback)`
   - [ ] 实现 `world.remove_on_tick(callback_id)`
   - [ ] 实现 `world.wait_for_tick()`

### 🔴 中优先级（1 周）

3. **World 设置** (2-3 天)
   - [ ] 创建 `WorldSettings` 类
   - [ ] 实现 `world.get_settings()`
   - [ ] 实现 `world.apply_settings()`

4. **地图访问** (3-4 天)
   - [ ] 创建 `Map` 类
   - [ ] 实现 `world.get_map()`
   - [ ] 封装 grid_map 和 semantic_map

### 🟡 低优先级（按需实现）

5. **观察者相机**
   - [ ] `world.get_spectator()`

6. **天气系统**
   - [ ] `world.get_weather()`
   - [ ] `world.set_weather()`

---

## 🎯 推荐实施顺序

### Week 1: ActorList
```python
# 目标：完成 actor 过滤和查找
actors = world.get_actors()
robots = actors.filter('robot.*')
drones = actors.filter('robot.cf2x')
actor = actors.find(actor_id)
```

**文件：**
- 新建 `simulation/actor_list.py`
- 修改 `simulation/world.py`

### Week 2: Tick 回调
```python
# 目标：完成 tick 回调机制
def on_tick(snapshot):
    print(f"Frame: {snapshot.frame}")

world.on_tick(on_tick)
snapshot = world.wait_for_tick()
```

**文件：**
- 新建 `simulation/snapshot.py`
- 修改 `simulation/world.py`

### Week 3: 设置和地图
```python
# 目标：完成设置和地图访问
settings = world.get_settings()
settings.synchronous_mode = True
world.apply_settings(settings)

map = world.get_map()
spawn_points = map.get_spawn_points()
```

**文件：**
- 新建 `simulation/world_settings.py`
- 新建 `simulation/map.py`
- 修改 `simulation/world.py`

---

## 📊 完成度评估

### 当前 World API 完成度

| 功能类别 | 完成度 | 说明 |
|---------|--------|------|
| **基础操作** | 90% | tick, reset, spawn_actor |
| **Actor 管理** | 60% | get_actors 有，但缺 filter |
| **回调机制** | 0% | 完全未实现 |
| **设置管理** | 0% | 完全未实现 |
| **地图访问** | 30% | 有 grid_map，但不是 CARLA 风格 |
| **高级功能** | 5% | 天气、观察者等未实现 |

### 整体评估
- **核心功能**: 70% ✅
- **CARLA 兼容性**: 50% ⚠️
- **易用性**: 65% ⚠️

---

## 💡 设计建议

### 1. 保持向后兼容
```python
# 旧代码仍然可以工作
actors = world.get_actors()  # 返回 ActorList，但可以当 List 用
for actor in actors:
    pass

# 新功能
robots = actors.filter('robot.*')
```

### 2. 渐进式实现
- 先实现 ActorList（最常用）
- 再实现 Tick 回调（重要但独立）
- 最后实现设置和地图（锦上添花）

### 3. 文档和测试
- 每个新功能都要有文档
- 提供清晰的使用示例
- 添加单元测试

---

## 🎓 总结

### World 管理中缺少的核心功能

1. **ActorList 和过滤** 🔥
   - 最常用的功能
   - 实现简单，影响大

2. **Tick 回调机制** 🔥
   - 重要的事件机制
   - 用于监控和调试

3. **World 设置** 🔴
   - 控制仿真行为
   - 同步模式等

4. **地图访问** 🔴
   - 统一的地图接口
   - 封装现有功能

### 建议的实施计划

**第 1 周**: ActorList + filter  
**第 2 周**: Tick 回调 + WorldSnapshot  
**第 3 周**: WorldSettings + Map  

完成这些后，World API 将达到 85%+ 的 CARLA 兼容性！
