# Robot Py-Trees Tasks

这个模块包含了基于py_trees的机器人任务构建器，通过entry points机制实现插件化的任务管理。

## 系统架构

```
BehaviorTreeManager (管理器)
    ↓ (通过entry points加载)
Task Builders (任务构建器)
    ↓ (创建并组合)
Behavior Classes (行为类)
    ↓ (调用)
Robot Skills (机器人技能)
```

## Entry Points配置

在`pyproject.toml`中配置的entry points：

```toml
[project.entry-points."robot.tasks"]
# Navigation tasks
"navigate_and_return" = "robot.skill.py_trees_tasks.navigation_tasks:build_navigate_and_return_tree"
"simple_navigation" = "robot.skill.py_trees_tasks.navigation_tasks:build_simple_navigation_tree"

# Manipulation tasks  
"pickup_task" = "robot.skill.py_trees_tasks.manipulation_tasks:build_pickup_task_tree"
"manipulation_sequence" = "robot.skill.py_trees_tasks.manipulation_tasks:build_manipulation_sequence_tree"

# Exploration tasks
"exploration" = "robot.skill.py_trees_tasks.exploration_tasks:build_exploration_tree"
"search_and_explore" = "robot.skill.py_trees_tasks.exploration_tasks:build_search_and_explore_tree"

# Composite tasks
"patrol_task" = "robot.skill.py_trees_tasks.composite_tasks:build_patrol_task_tree"
"delivery_task" = "robot.skill.py_trees_tasks.composite_tasks:build_delivery_task_tree"
```

## 可用任务类型

### 导航任务 (Navigation Tasks)

#### 1. simple_navigation
简单的点到点导航
```python
params = {
    "goal_pos": [x, y, z],
    "goal_quat_wxyz": [w, x, y, z]  # 可选
}
```

#### 2. navigate_and_return  
导航到目标位置并返回
```python
params = {
    "goal_pos": [x, y, z],
    "goal_quat_wxyz": [w, x, y, z],  # 可选
    "take_photo": True,              # 可选
    "broadcast_messages": True,      # 可选
    "photo_path": "/path/to/photo.jpg"  # 可选
}
```

#### 3. waypoint_navigation
多点导航任务
```python
params = {
    "waypoints": [[x1,y1,z1], [x2,y2,z2], ...],
    "loop": False  # 可选，是否循环
}
```

### 操作任务 (Manipulation Tasks)

#### 1. pickup_task
抓取任务
```python
params = {
    "object_prim_path": "/World/target_object",
    "robot_hand_prim_path": "/robot/hand",
    "target_location": [x, y, z],  # 可选，放置位置
    "distance_threshold": 2.0,     # 可选
    "use_detection": True          # 可选
}
```

#### 2. manipulation_sequence
操作序列任务
```python
params = {
    "manipulation_sequence": [
        {
            "object_prim_path": "/World/object1",
            "pickup_location": [x1, y1, z1],
            "target_location": [x2, y2, z2]
        },
        # 更多操作...
    ],
    "robot_hand_prim_path": "/robot/hand"
}
```

#### 3. sorting_task
分拣任务
```python
params = {
    "objects_to_sort": [...],
    "sorting_criteria": "color",  # 或其他标准
    "target_locations": {"red": [x1,y1,z1], "blue": [x2,y2,z2]},
    "robot_hand_prim_path": "/robot/hand"
}
```

### 探索任务 (Exploration Tasks)

#### 1. exploration
区域探索
```python
params = {
    "boundary": [[x1,y1], [x2,y2], [x3,y3], [x4,y4]],
    "holes": [[[hx1,hy1], [hx2,hy2], ...]],  # 可选
    "target_prim": "/World/search_target",   # 可选
    "take_photos": True,                     # 可选
    "lane_width": 2.0,                       # 可选
    "robot_radius": 0.5                      # 可选
}
```

#### 2. search_and_explore
搜索并探索
```python
params = {
    "boundary": [[x1,y1], [x2,y2], [x3,y3], [x4,y4]],
    "target_prim": "/World/search_target",
    "max_search_attempts": 3,  # 可选
    "search_areas": [          # 可选，重点搜索区域
        {"boundary": [[...]]},
        {"boundary": [[...]]}
    ]
}
```

#### 3. patrol_exploration
巡逻探索
```python
params = {
    "patrol_areas": [
        {"boundary": [[...]], "entry_point": [x,y,z]},
        # 更多区域...
    ],
    "patrol_cycles": -1,  # -1为无限循环
    "rest_points": [[x1,y1,z1], [x2,y2,z2]],  # 可选
    "detection_interval": 10  # 可选
}
```

### 复合任务 (Composite Tasks)

#### 1. patrol_task
巡逻任务
```python
params = {
    "patrol_points": [[x1,y1,z1], [x2,y2,z2], ...],
    "patrol_cycles": 1,        # -1为无限循环
    "detection_at_points": True,  # 可选
    "photo_at_points": True,      # 可选
    "target_prim": "/World/intruder"  # 可选
}
```

#### 2. delivery_task
配送任务
```python
params = {
    "pickup_location": [x1, y1, z1],
    "delivery_location": [x2, y2, z2],
    "object_prim_path": "/World/package",
    "robot_hand_prim_path": "/robot/hand",
    "verify_delivery": True,  # 可选
    "return_to_base": True    # 可选
}
```

#### 3. search_and_rescue
搜救任务
```python
params = {
    "search_area": [[x1,y1], [x2,y2], [x3,y3], [x4,y4]],
    "target_prim": "/World/victim",
    "rescue_location": [x, y, z],
    "max_search_time": 300,      # 可选
    "emergency_protocols": True  # 可选
}
```

#### 4. inspection_task
检查任务
```python
params = {
    "inspection_points": [[x1,y1,z1], [x2,y2,z2], ...],
    "inspection_criteria": {...},     # 可选
    "photo_documentation": True,      # 可选
    "detailed_detection": True        # 可选
}
```

## 使用方法

### 1. 基本使用

```python
from robot.skill.behavior_tree_manager import BehaviorTreeManager

# 创建管理器
bt_manager = BehaviorTreeManager(robot_instance)

# 查看可用任务
print(bt_manager.get_available_tasks())

# 执行任务
params = {"goal_pos": [10.0, 5.0, 1.0]}
status = bt_manager.execute_task("simple_navigation", params)
```

### 2. 创建并检查行为树

```python
# 创建行为树但不执行
tree = bt_manager.create_tree_for_task("pickup_task", params)

# 显示行为树结构
print(bt_manager.display_current_tree())

# 手动执行
while tree.root.status == py_trees.common.Status.RUNNING:
    tree.tick()
```

### 3. 监控任务状态

```python
# 获取当前任务状态
status = bt_manager.get_current_tree_status()

# 停止当前任务
bt_manager.stop_current_task()

# 重新加载任务插件
bt_manager.reload_tasks()
```

## 扩展新任务

要添加新的任务类型：

1. 在相应的任务模块中创建构建函数
2. 在`pyproject.toml`中添加entry point
3. 重新安装包或重新加载任务

```python
def build_my_custom_task(robot_instance, params: dict):
    # 创建行为树根节点
    root = py_trees.composites.Sequence(name="我的自定义任务", memory=False)
    
    # 设置黑板数据
    _setup_custom_blackboard(params)
    
    # 添加行为节点
    behavior1 = SomeBehaviour("行为1", robot_instance)
    behavior2 = SomeBehaviour("行为2", robot_instance)
    
    root.add_children([behavior1, behavior2])
    return root
```

然后在`pyproject.toml`中添加：
```toml
"my_custom_task" = "robot.skill.py_trees_tasks.custom_tasks:build_my_custom_task"
```

## 调试和日志

系统包含完整的日志记录：

```python
import logging
logging.basicConfig(level=logging.DEBUG)

# 现在所有操作都会有详细日志
bt_manager = BehaviorTreeManager(robot_instance)
```

## 注意事项

1. 确保所有必需的参数都在params字典中提供
2. 黑板键名必须与行为类中定义的一致
3. 任务构建函数必须返回py_trees.composites.Composite类型的根节点
4. 使用`memory=False`的组合节点以确保每次执行时重新评估
5. 长时间运行的任务应该正确处理生成器和异步状态