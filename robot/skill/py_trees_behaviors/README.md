# Robot Skills Py-Trees Behaviors

这个模块为机器人技能系统提供了基于py_trees的行为树实现。每个机器人技能都被封装为一个py_trees Behaviour类，可以轻松地组合成复杂的行为树。

## 功能特性

- **模块化设计**: 每个技能都是独立的行为节点
- **黑板通信**: 使用py_trees黑板机制进行数据共享
- **异步支持**: 支持长时间运行的技能（如导航、探索）
- **错误处理**: 完善的异常处理和状态管理
- **易于扩展**: 可以轻松添加新的技能行为

## 可用的行为类

### 导航相关
- `NavigateToBehaviour`: 导航到指定位置
- `ReturnHomeBehaviour`: 返回起始位置

### 操作相关
- `PickupObjectBehaviour`: 抓取物体
- `PutDownBehaviour`: 放下物体

### 检测相关
- `DetectBehaviour`: 目标检测
- `ObjectDetectionBehaviour`: 基于语义相机的物体检测

### 探索相关
- `ExploreBehaviour`: 区域探索
- `PlanExplorationWaypointsBehaviour`: 规划探索路径点

### 通信相关
- `BroadcastBehaviour`: 广播消息

### 相机相关
- `TakePhotoBehaviour`: 拍照

## 使用方法

### 1. 导入所需的行为类

```python
import py_trees
from robot.skill.py_trees_behaviors import (
    NavigateToBehaviour, 
    PickupObjectBehaviour,
    DetectBehaviour
)
```

### 2. 创建行为树

```python
def create_simple_task_tree(robot_instance):
    # 创建根节点（序列执行）
    root = py_trees.composites.Sequence(name="简单任务", memory=False)
    
    # 创建行为节点
    navigate = NavigateToBehaviour("导航到目标", robot_instance)
    detect = DetectBehaviour("检测目标", robot_instance)
    pickup = PickupObjectBehaviour("抓取物体", robot_instance)
    
    # 添加到行为树
    root.add_children([navigate, detect, pickup])
    
    return root
```

### 3. 设置黑板数据

```python
def setup_blackboard():
    blackboard = py_trees.blackboard.Client()
    
    # 注册并设置导航目标
    blackboard.register_key("goal_pos", access=py_trees.common.Access.WRITE)
    blackboard.goal_pos = [10.0, 5.0, 1.0]
    
    # 注册并设置检测目标
    blackboard.register_key("target_prim", access=py_trees.common.Access.WRITE)
    blackboard.target_prim = "/World/target_object"
    
    # 注册并设置抓取参数
    blackboard.register_key("robot_hand_prim_path", access=py_trees.common.Access.WRITE)
    blackboard.register_key("object_prim_path", access=py_trees.common.Access.WRITE)
    blackboard.robot_hand_prim_path = "/robot/hand"
    blackboard.object_prim_path = "/World/target_object"
```

### 4. 运行行为树

```python
def run_behavior_tree(robot_instance):
    # 设置黑板数据
    setup_blackboard()
    
    # 创建行为树
    root = create_simple_task_tree(robot_instance)
    
    # 初始化行为树
    root.setup_with_descendants()
    
    # 执行行为树
    while root.status == py_trees.common.Status.RUNNING:
        root.tick_once()
        
        # 可选：显示当前状态
        print(py_trees.display.unicode_tree(root, show_status=True))
        
        # 添加适当的延迟
        time.sleep(0.1)
    
    print(f"任务完成，状态: {root.status}")
```

## 黑板键值说明

每个行为类都使用特定的黑板键来获取输入参数和存储输出结果：

### NavigateToBehaviour
- **输入**: `goal_pos` (目标位置), `goal_quat_wxyz` (目标姿态)

### PickupObjectBehaviour
- **输入**: `robot_hand_prim_path` (机械手路径), `object_prim_path` (物体路径), `distance_threshold` (距离阈值)

### DetectBehaviour
- **输入**: `target_prim` (检测目标)
- **输出**: `detection_result` (检测结果)

### ObjectDetectionBehaviour
- **输入**: `semantic_camera` (语义相机), `map_semantic` (语义地图), `target_class` (目标类别)
- **输出**: `detected_object` (检测到的物体)

### ExploreBehaviour
- **输入**: `boundary` (边界), `holes` (洞), `target_prim` (目标)

### BroadcastBehaviour
- **输入**: `broadcast_content` (广播内容)

### TakePhotoBehaviour
- **输入**: `photo_file_path` (保存路径，可选)
- **输出**: `photo_data` (照片数据)

## 示例

查看 `example_usage.py` 文件获取完整的使用示例，包括：

1. 简单导航任务
2. 复杂抓取任务
3. 区域探索任务

## 扩展新的行为

要添加新的技能行为，请遵循以下步骤：

1. 在相应的模块文件中创建新的行为类
2. 继承 `py_trees.behaviour.Behaviour`
3. 实现必要的方法：`setup()`, `initialise()`, `update()`, `terminate()`
4. 注册所需的黑板键
5. 在 `__init__.py` 中导出新的行为类

## 注意事项

- 确保在使用行为类之前正确设置黑板数据
- 对于长时间运行的技能（如导航、探索），行为类会正确处理生成器和异步状态
- 所有行为类都包含完善的错误处理和日志记录
- 使用 `memory=False` 的组合节点以确保每次执行时重新评估子节点状态