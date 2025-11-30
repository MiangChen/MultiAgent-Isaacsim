# MultiAgent-Unreal 迁移方案

**版本**: v1.1  
**日期**: 2024年11月  
**状态**: 📋 设计阶段

---

## 1. 项目目标

将 MultiAgent-IsaacSim 的三层架构迁移到 Unreal Engine 5.5，实现：

1. **连接本地 UE5.5**: 直接连接本地运行的 UE5.5 编辑器（UnrealCV 插件已安装）
2. **中间件架构**: multiagent-unreal 作为中间件，接收 ROS2 技能控制信号
3. **快速创建实体**: 通过中间件 API 快速创建机器人/物体
4. **技能系统复用**: 复用现有的 skill 系统（navigate_to, track, take_photo 等）

---

## 2. 可行性评估

### 2.1 架构对比

| 层级 | MultiAgent-IsaacSim | MultiAgent-Unreal | 迁移难度 |
|------|---------------------|-------------------|----------|
| **应用层** | SkillManager + ROS2 Action | 完全复用 | ⭐ 低 |
| **仿真层** | World/Actor/Blueprint | 重新实现 (UnrealCV) | ⭐⭐⭐ 中等 |
| **引擎层** | Isaac Sim | Unreal Engine 5.5 | N/A |

### 2.2 通信架构

```
┌─────────────────────────────────────────────────────────────────┐
│                    MultiAgent-Unreal (Python)                   │
├─────────────────────────────────────────────────────────────────┤
│  应用层: SkillManager + ROS2 (复用)                              │
│  仿真层: UnrealWorld + UnrealActor (新实现)                      │
│                         │                                        │
│                         ▼                                        │
│              UnrealCV Client (TCP:9000)                         │
└─────────────────────────┬───────────────────────────────────────┘
                          │
                          ▼
┌─────────────────────────────────────────────────────────────────┐
│                 Unreal Engine 5.5 编辑器                         │
│                 + UnrealCV Plugin (已安装)                       │
└─────────────────────────────────────────────────────────────────┘
```

### 2.3 借鉴 UnrealZoo 的能力

| 特性 | 借鉴方式 |
|------|----------|
| **Tracking 算法** | 借鉴视觉跟踪逻辑，不依赖其架构 |
| **UnrealCV 命令** | 直接使用 vbp 命令 (carry_body, nav_to_goal 等) |
| **物体交互** | 复用 pick/drop 命令 |

---

## 3. 架构设计

### 3.1 整体架构

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         Application Layer (应用层) - 复用               │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────────────┐  │
│  │  SkillManager   │  │ SkillROSInterface│  │  ROS2 Action Server    │  │
│  └────────┬────────┘  └────────┬────────┘  └───────────┬─────────────┘  │
│           └────────────────────┴───────────────────────┘                 │
└────────────────────────────────┬─────────────────────────────────────────┘
                                 │
┌────────────────────────────────┼─────────────────────────────────────────┐
│                    Simulation Layer (仿真层) - 新实现                    │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────────────┐  │
│  │  UnrealWorld    │  │  UnrealActor    │  │  UnrealSensor           │  │
│  │  (CARLA 风格)   │  │  (Robot/Static) │  │  (Camera/Depth)         │  │
│  └────────┬────────┘  └────────┬────────┘  └───────────┬─────────────┘  │
│           └────────────────────┴───────────────────────┘                 │
│                                │                                         │
│                    ┌───────────▼───────────┐                             │
│                    │  UnrealCV Client      │                             │
│                    │  (TCP:9000)           │                             │
│                    └───────────────────────┘                             │
└──────────────────────────────────────────────────────────────────────────┘
                                 │
                    ┌────────────▼────────────┐
                    │   Unreal Engine 5.5     │
                    │   + UnrealCV Plugin     │
                    └─────────────────────────┘
```

### 3.2 核心组件

#### 3.2.1 UnrealWorld

```python
# simulation_unreal/world.py
from unrealcv import Client

class UnrealWorld:
    """Unreal 世界管理器 (CARLA 风格 API)"""
    
    def __init__(self, port: int = 9000):
        self._client = Client(('127.0.0.1', port))
        self._client.connect()
        self._actors = {}
        
    def spawn_actor(self, blueprint, transform=None):
        """创建 Actor"""
        name = f"{blueprint.type}_{len(self._actors)}"
        loc = transform.location.to_list() if transform else [0, 0, 100]
        
        cmd = f"vset /objects/spawn {blueprint.class_name} {name}"
        self._client.request(cmd)
        self._client.request(f"vset /object/{name}/location {loc[0]} {loc[1]} {loc[2]}")
        
        actor = UnrealActor(name, self._client)
        self._actors[name] = actor
        return actor
    
    def get_objects(self):
        """获取所有对象"""
        return self._client.request("vget /objects").split()
```

#### 3.2.2 UnrealActor

```python
# simulation_unreal/actor.py
class UnrealActor:
    """Unreal Actor 基类"""
    
    def __init__(self, name: str, client):
        self.name = name
        self._client = client
        
    def get_location(self):
        res = self._client.request(f"vget /object/{self.name}/location")
        return [float(x) for x in res.split()]
    
    def set_location(self, loc):
        self._client.request(f"vset /object/{self.name}/location {loc[0]} {loc[1]} {loc[2]}")
    
    def apply_control(self, velocity):
        """应用控制 [角速度, 线速度]"""
        self._client.request(f"vbp {self.name} set_move {velocity[0]} {velocity[1]}")
```

#### 3.2.3 ROS2 桥接

```python
# ros_unreal/ros2_bridge.py
class UnrealROS2Bridge:
    """ROS2 桥接 - 发布传感器数据，接收控制命令"""
    
    def __init__(self, namespace: str, client):
        self.client = client
        self.node = Node(f"unreal_bridge_{namespace}")
        
        # 发布者
        self.pub_image = self.node.create_publisher(Image, f"{namespace}/camera/image_raw", 10)
        self.pub_odom = self.node.create_publisher(Odometry, f"{namespace}/odom", 10)
        
        # 定时发布 (30Hz)
        self.timer = self.node.create_timer(1/30, self._publish_sensors)
        
    def _publish_sensors(self):
        # 获取图像
        img_data = self.client.request(f"vget /camera/{self.cam_id}/lit bmp")
        # 转换并发布...
```

---

## 4. 目录结构

```
multiagent-unreal/
├── config/
│   ├── unreal_config.yaml        # Unreal 配置
│   └── skill_config.yaml         # 技能配置 (复用)
│
├── simulation_unreal/            # 仿真层 (新实现)
│   ├── __init__.py
│   ├── world.py                  # UnrealWorld
│   ├── actor.py                  # UnrealActor
│   ├── robot_actor.py            # 机器人 Actor
│   ├── transform.py              # Transform (复用)
│   └── sensor/
│       ├── camera.py             # 相机
│       └── depth_camera.py       # 深度相机
│
├── application/                  # 应用层 (完全复用)
│   ├── skill_manager.py
│   ├── skill_ros_interface.py
│   └── skills/
│       ├── base/
│       │   ├── navigate_to.py    # 适配 UnrealCV nav_to_goal
│       │   ├── track.py          # 借鉴 UnrealZoo 逻辑
│       │   ├── take_photo.py     # 适配 UnrealCV get_image
│       │   └── pick_up.py        # 使用 carry_body 命令
│       └── drone/
│           └── take_off.py
│
├── ros_unreal/                   # ROS2 集成
│   ├── ros2_bridge.py
│   └── sensor_bridge.py
│
└── examples/
    ├── basic_demo.py
    ├── tracking_demo.py
    └── pick_drop_demo.py
```

---

## 5. 本地 UE5.5 编辑器集成

### 5.1 UnrealCV 插件安装 ✅ (已完成)

### 5.2 使用流程

```bash
# 1. 启动 UE5.5 编辑器，打开你的项目✅ (已完成)
ue5  # alias ue5='/home/ubuntu/Linux_Unreal_Engine_5.5.4/Engine/Binaries/Linux/UnrealEditor' ✅ (已完成)

# 2. 在编辑器中点击 Play (UnrealCV Server 自动启动在 9000 端口)

# 3. 运行 Python 代码
python examples/basic_demo.py
```

### 5.3 配置文件

```yaml
# config/unreal_config.yaml
unreal:
  host: "127.0.0.1"
  port: 9000
  resolution: [1280, 720]

ros2:
  enabled: true
  namespace_prefix: "robot"
  publish_rate: 30  # Hz
```

---

## 6. API 映射

### 6.1 核心 API

| Isaac Sim | UnrealCV | 命令示例 |
|-----------|----------|----------|
| `spawn_actor()` | `vset /objects/spawn` | `vset /objects/spawn bp_character_C player_0` |
| `get_transform()` | `vget /object/.../location` | `vget /object/player_0/location` |
| `set_transform()` | `vset /object/.../location` | `vset /object/player_0/location 100 200 50` |
| `apply_control()` | `vbp ... set_move` | `vbp player_0 set_move 30 100` |
| `nav_to()` | `vbp ... nav_to_goal` | `vbp player_0 nav_to_goal 500 300 0` |

### 6.2 传感器 API

| Isaac Sim | UnrealCV | 命令示例 |
|-----------|----------|----------|
| `camera.get_image()` | `vget /camera/.../lit` | `vget /camera/0/lit bmp` |
| `camera.get_depth()` | `vget /camera/.../depth` | `vget /camera/0/depth npy` |
| `get_mask()` | `vget /camera/.../object_mask` | `vget /camera/0/object_mask png` |

### 6.3 交互 API

| 功能 | UnrealCV 命令 |
|------|---------------|
| 拿取物品 | `vbp player_0 carry_body` |
| 放下物品 | `vbp player_0 drop_body` |
| 检查是否携带 | `vbp player_0 is_carrying` |
| 导航到目标 | `vbp player_0 nav_to_goal x y z` |
| 随机导航 | `vbp player_0 nav_random radius loop` |

---

## 7. 技能系统迁移

### 7.1 技能适配表

| 技能 | 适配方式 | UnrealCV 命令 |
|------|----------|---------------|
| `take_photo` | 替换图像获取 | `vget /camera/0/lit bmp` |
| `track` | 借鉴 UnrealZoo 逻辑 | `vbp ... set_move` + `check_visibility` |
| `navigate_to` | 使用 NavMesh | `vbp ... nav_to_goal x y z` |
| `pick_up` | 新增 | `vbp ... carry_body` |
| `put_down` | 新增 | `vbp ... drop_body` |
| `detect` | 使用 mask 检测 | `vget /camera/0/object_mask` |

### 7.2 Track 技能实现

```python
# application/skills/base/track.py
@SkillManager.register("track")
def track(robot, skill_manager, **kwargs):
    target_name = kwargs.get("target_name")
    
    # 获取 UnrealCV client
    client = robot.world._client
    
    # 检查目标可见性 (借鉴 UnrealZoo)
    mask = client.request(f"vget /camera/{robot.cam_id}/object_mask bmp")
    visibility = check_target_in_mask(mask, target_name)
    
    if visibility <= 0:
        lost_count = skill_manager.get_skill_data("track", "lost_count", 0) + 1
        if lost_count > 20:
            return skill_manager.form_feedback("failed", "Target lost", 100)
        skill_manager.set_skill_data("track", "lost_count", lost_count)
    else:
        skill_manager.set_skill_data("track", "lost_count", 0)
    
    # 计算相对位置
    robot_pos = robot.get_location()
    target_pos = [float(x) for x in client.request(f"vget /object/{target_name}/location").split()]
    
    # 生成跟踪控制
    angle, distance = compute_relative(robot_pos, target_pos)
    control = [angle * 0.5, min(distance * 0.1, 100)]  # [角速度, 线速度]
    
    robot.apply_control(control)
    return skill_manager.form_feedback("processing", f"Tracking d={distance:.0f}", 50)
```

---

## 8. 实现路线图

### Phase 1: 基础架构 (1-2 周)

- [ ] `simulation_unreal/world.py` - UnrealWorld
- [ ] `simulation_unreal/actor.py` - UnrealActor
- [ ] `simulation_unreal/robot_actor.py` - 机器人封装
- [ ] 基础连接测试

### Phase 2: 技能迁移 (1-2 周)

- [ ] `take_photo` 适配
- [ ] `track` 实现 (借鉴 UnrealZoo)
- [ ] `navigate_to` 适配
- [ ] `pick_up` / `put_down` 新增

### Phase 3: ROS2 集成 (1 周)

- [ ] `ros_unreal/ros2_bridge.py`
- [ ] 传感器数据发布
- [ ] SkillROSInterface 集成

### Phase 4: 示例与文档 (1 周)

- [ ] 基础示例
- [ ] 跟踪示例
- [ ] 物品交互示例

---

## 9. 使用示例

### 9.1 基础使用

```python
from simulation_unreal import UnrealWorld, Transform, Location

# 连接本地 UE5.5
world = UnrealWorld(port=9000)

# 创建机器人
robot = world.spawn_actor(
    blueprint=PlayerBlueprint(),
    transform=Transform(Location(0, 0, 100))
)

# 移动
robot.apply_control([0, 100])  # 前进

# 导航到目标
robot.nav_to_goal([500, 300, 0])

# 拿取物品
robot.carry_body()
```

### 9.2 ROS2 技能调用

```bash
# 跟踪目标
ros2 action send_goal /robot_0/skill_execution plan_msgs/action/SkillExecution \
  '{skill: "track", params: [{key: "target_name", value: "target_0"}]}' --feedback

# 导航
ros2 action send_goal /robot_0/skill_execution plan_msgs/action/SkillExecution \
  '{skill: "navigate_to", params: [{key: "goal_pos", value: "[500, 300, 0]"}]}' --feedback

# 拿取物品
ros2 action send_goal /robot_0/skill_execution plan_msgs/action/SkillExecution \
  '{skill: "pick_up", params: []}' --feedback
```

---

## 10. 总结

### 精简后的优势

| 方面 | 说明 |
|------|------|
| **架构简单** | 直连本地 UE5.5，无需管理 Binary |
| **代码复用** | 应用层 100% 复用 |
| **开发效率** | 纯 Python 开发，快速迭代 |
| **灵活性** | 可在 UE 编辑器中实时调试场景 |

### 预计工作量

| 阶段 | 时间 | 产出 |
|------|------|------|
| Phase 1 | 1-2 周 | 基础架构可用 |
| Phase 2 | 1-2 周 | 核心技能可用 |
| Phase 3 | 1 周 | ROS2 集成完成 |
| Phase 4 | 1 周 | 示例和文档 |
| **总计** | **4-6 周** | 完整可用版本 |

---

**文档版本**: v1.1  
**最后更新**: 2024年11月  
**状态**: 📋 设计阶段
