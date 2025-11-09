# Application Layer Skills

应用层技能实现，基于统一的状态机模式。

## 已实现的技能

### 1. navigate_to - 导航到目标点

**功能**: 使用 ROS 路径规划和 MPC 控制导航到目标位置

**使用示例**:
```python
skill_manager.execute_skill(
    'navigate_to',
    goal_pos=[10.0, 20.0, 0.0],
    goal_quat_wxyz=[1.0, 0.0, 0.0, 0.0]
)
```

**参数**:
- `goal_pos`: 目标位置 [x, y, z]
- `goal_quat_wxyz`: 目标姿态四元数 [w, x, y, z] (可选)
- `start_pos`: 起始位置 (可选，默认当前位置)
- `start_quat`: 起始姿态 (可选，默认当前姿态)

**状态流程**:
```
INITIALIZING -> EXECUTING -> COMPLETED/FAILED
```

**依赖**:
- ROS action: `ComputePathToPose`
- MPC 控制器: `robot.node_controller_mpc`

---

### 2. explore - 区域覆盖探索

**功能**: 规划覆盖路径并探索指定区域

**使用示例**:
```python
boundary = [
    [-5.0, -5.0, 1.0],
    [15.0, -5.0, 1.0],
    [15.0, 15.0, 1.0],
    [-5.0, 15.0, 1.0],
]

skill_manager.execute_skill(
    'explore',
    boundary=boundary,
    holes=[],
    target_prim="car"
)
```

**参数**:
- `boundary`: 探索区域边界多边形 [[x1,y1,z1], [x2,y2,z2], ...]
- `holes`: 内部障碍物孔洞列表 (可选)
- `target_prim`: 检测目标类型 (可选)
- `interpolation_distance`: 路径点间距 (默认 0.05m)
- `interpolation_method`: 插值方法 'linear'/'spline'/'adaptive' (默认 'linear')

**状态流程**:
```
INITIALIZING -> NAVIGATING_TO_START -> EXECUTING -> COMPLETED/FAILED
```

**依赖**:
- `navigate_to` 技能 (导航到起点)
- `plan_exploration_waypoints` (路径规划)
- MPC 控制器: `robot.node_controller_mpc`
- OMPL 规划器: `robot.node_planner_ompl`

**特性**:
- 自动规划覆盖路径（蛇形扫描）
- 先导航到起点，再开始探索
- 支持检测模式 (`robot.is_detecting`)
- 支持目标检测 (`robot.target_prim`)

---

## 技能开发指南

### 技能实现模式

所有技能遵循统一的状态机模式：

```python
def my_skill(**kwargs):
    robot = kwargs.get("robot")
    skill_manager = kwargs.get("skill_manager")
    skill_name = "my_skill"
    
    current_state = skill_manager.get_skill_state(skill_name)
    
    if current_state in [None, "INITIALIZING"]:
        return _init_my_skill(robot, skill_manager, skill_name, kwargs)
    elif current_state == "EXECUTING":
        return _handle_executing(robot, skill_manager, skill_name)
    elif current_state == "COMPLETED":
        return _handle_completed(robot, skill_manager, skill_name)
    elif current_state == "FAILED":
        return _handle_failed(robot, skill_manager, skill_name)
```

### 关键原则

1. **非阻塞**: 技能函数立即返回，不阻塞仿真循环
2. **状态驱动**: 使用状态机管理执行流程
3. **数据隔离**: 使用 `skill_manager.set_skill_data()` 存储私有数据
4. **错误处理**: 设置超时和错误状态
5. **Control 对象**: 通过 `robot.apply_control()` 输出控制命令

### 状态管理 API

```python
# 获取/设置状态
state = skill_manager.get_skill_state(skill_name)
skill_manager.set_skill_state(skill_name, "EXECUTING")

# 存储/获取数据
skill_manager.set_skill_data(skill_name, "key", value)
value = skill_manager.get_skill_data(skill_name, "key", default)

# 错误处理
skill_manager.skill_errors[skill_name] = "Error message"

# 反馈
return skill_manager.form_feedback("processing", "Message", progress)
```

### 控制输出

技能通过 Control 对象输出控制命令：

```python
from simulation import RobotControl

control = RobotControl()
control.linear_velocity = [1.0, 0.0, 0.0]
control.angular_velocity = [0.0, 0.0, 0.0]
robot.apply_control(control)
```

### 技能组合

技能可以调用其他技能：

```python
# 在 explore 中调用 navigate_to
from application.skills.navigate_to import navigate_to

skill_manager.register_skill("navigate_to", navigate_to)
skill_manager.execute_skill("navigate_to", goal_pos=[10, 20, 0])

# 检查子技能状态
nav_state = skill_manager.get_skill_state("navigate_to")
if nav_state == "COMPLETED":
    # 继续下一步
    pass
```

---

### 3. take_off - 无人机起飞

**功能**: 无人机垂直起飞到指定高度

**使用示例**:
```python
skill_manager.execute_skill(
    'take_off',
    altitude=5.0
)
```

**参数**:
- `altitude`: 目标高度（米），必需参数，必须 > 0

**状态流程**:
```
INITIALIZING -> NAVIGATING_TO_ALTITUDE -> COMPLETED/FAILED
```

**依赖**:
- `navigate_to` 技能（导航到目标高度）
- 机器人 body 组件

**特性**:
- 保持当前 XY 位置
- 保持当前朝向
- 垂直上升到目标高度
- 实时高度反馈

**内部流程**:
1. 获取当前位置 (x, y, z_current)
2. 计算目标位置 (x, y, altitude)
3. 进入 NAVIGATING_TO_ALTITUDE 状态
4. 调用 navigate_to 导航到目标位置
5. 监控导航进度
6. 到达目标高度后完成

---

### 4. pick_up - 拾取物体

**功能**: 创建物理关节将物体附着到机器人手部

**使用示例**:
```python
skill_manager.execute_skill(
    'pick_up',
    robot_hand_prim_path="/World/robot/hand",
    object_prim_path="/World/objects/box",
    distance_threshold=2.0
)
```

**参数**:
- `robot_hand_prim_path`: 机器人手部 prim 路径（必需）
- `object_prim_path`: 目标物体 prim 路径（必需）
- `distance_threshold`: 距离阈值（米），默认 2.0
- `axis`: 关节轴向，默认 [0, 0, 1]
- `local_pos_hand`: 手部局部位置，默认 [0, 0, 1]
- `local_pos_object`: 物体局部位置，默认 [0, 0, 0]

**状态流程**:
```
INITIALIZING -> EXECUTING -> COMPLETED/FAILED
```

**依赖**:
- `scene_manager` - 场景管理器
- Isaac Sim 物理引擎

**特性**:
- 距离检查（防止远距离抓取）
- 物理关节创建
- 碰撞禁用
- 速度清零

---

### 5. put_down - 放下物体

**功能**: 禁用物理关节，恢复物体物理属性

**使用示例**:
```python
skill_manager.execute_skill(
    'put_down',
    robot_hand_prim_path="/World/robot/hand",
    object_prim_path="/World/objects/box"
)
```

**参数**:
- `robot_hand_prim_path`: 机器人手部 prim 路径（必需）
- `object_prim_path`: 目标物体 prim 路径（必需）

**状态流程**:
```
INITIALIZING -> EXECUTING -> COMPLETED/FAILED
```

**依赖**:
- `scene_manager` - 场景管理器
- Isaac Sim 物理引擎
- 需要先执行 `pick_up`

**特性**:
- 关节禁用
- 碰撞恢复
- 动量传递（惯性模拟）

---

### 6. take_photo - 拍照

**功能**: 使用机器人相机捕获图像

**使用示例**:
```python
skill_manager.execute_skill(
    'take_photo',
    camera_name="front",
    save_to_file="/tmp/photo.png"
)
```

**参数**:
- `camera_name`: 相机名称，默认 "default"
- `save_to_file`: 保存路径（可选），为空则不保存

**状态流程**:
```
INITIALIZING -> EXECUTING -> COMPLETED/FAILED
```

**依赖**:
- `robot.camera_dict` - 相机字典

**特性**:
- 相机可用性检查
- RGB 图像捕获
- 可选文件保存
- 图像数据存储

**返回数据**:
```python
photo_data = skill_manager.get_skill_data('take_photo', 'photo_data')
# {
#     'rgb_image': numpy.ndarray,
#     'camera_name': str
# }
```

---

### 7. detect - 重叠检测

**功能**: 检测机器人是否与目标物体重叠

**使用示例**:
```python
skill_manager.execute_skill(
    'detect',
    target_prim="car"
)
```

**参数**:
- `target_prim`: 目标 prim 名称（必需）

**状态流程**:
```
INITIALIZING -> EXECUTING -> COMPLETED/FAILED
```

**依赖**:
- `robot.body` - 机器人身体组件
- `robot.scene_manager` - 场景管理器

**特性**:
- 重叠检测（overlap detection）
- 位置获取
- 结果存储

**返回数据**:
```python
result = skill_manager.get_skill_data('detect', 'detection_result')
# {
#     'success': bool,
#     'message': str,
#     'data': detection_data
# }
```

---

### 8. object_detection - 语义检测

**功能**: 使用语义相机检测目标物体

**使用示例**:
```python
skill_manager.execute_skill(
    'object_detection',
    camera_name="front",
    target_class="car"
)
```

**参数**:
- `camera_name`: 相机名称，默认 "default"
- `target_class`: 目标类别，默认 "car"

**状态流程**:
```
INITIALIZING -> EXECUTING -> COMPLETED/FAILED
```

**依赖**:
- `robot.camera_dict` - 相机字典
- 语义相机

**特性**:
- 语义分割
- 目标识别
- 边界框检测
- 结果存储

**返回数据**:
```python
result = skill_manager.get_skill_data('object_detection', 'detection_result')
# {
#     'success': bool,
#     'message': str,
#     'data': semantic_detection_data
# }
```

---

## 待实现的技能

### 短期
- `land` - 无人机降落
- `follow` - 跟随目标
- `patrol` - 巡逻路径

### 中期
- `track` - 目标跟踪
- `record_video` - 录制视频
- `measure_distance` - 距离测量

### 长期
- `manipulate` - 精细操作
- `assemble` - 组装任务

---

## 参考

- 技能管理器: `application/skill_manager.py`
- 示例代码: `main_example.py`
- 架构文档: `docs/PROJECT_SUMMARY.md`
