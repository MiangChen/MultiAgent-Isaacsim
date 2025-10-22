"""
Py-trees behaviors usage example.
演示如何使用机器人技能的py_trees行为类。
"""

import py_trees
from robot.skill.py_trees_behaviors import (
    NavigateToBehaviour, ReturnHomeBehaviour,
    PickupObjectBehaviour, PutDownBehaviour,
    DetectBehaviour, ObjectDetectionBehaviour,
    ExploreBehaviour, PlanExplorationWaypointsBehaviour,
    BroadcastBehaviour, TakePhotoBehaviour
)


def create_simple_navigation_tree(robot_instance):
    """创建一个简单的导航行为树"""
    
    # 根节点
    root = py_trees.composites.Sequence(name="导航任务", memory=False)
    
    # 子节点
    broadcast_start = BroadcastBehaviour("广播开始", robot_instance)
    navigate = NavigateToBehaviour("导航到目标", robot_instance)
    take_photo = TakePhotoBehaviour("拍照记录", robot_instance)
    return_home = ReturnHomeBehaviour("返回起点", robot_instance)
    broadcast_end = BroadcastBehaviour("广播结束", robot_instance)
    
    # 构建树结构
    root.add_children([broadcast_start, navigate, take_photo, return_home, broadcast_end])
    
    return root


def create_pickup_task_tree(robot_instance):
    """创建一个抓取任务的行为树"""
    
    # 根节点
    root = py_trees.composites.Sequence(name="抓取任务", memory=False)
    
    # 检测和抓取的选择器
    detection_selector = py_trees.composites.Selector(name="检测物体", memory=False)
    
    # 检测行为
    detect_object = ObjectDetectionBehaviour("物体检测", robot_instance)
    detect_fallback = DetectBehaviour("备用检测", robot_instance)
    detection_selector.add_children([detect_object, detect_fallback])
    
    # 导航到物体
    navigate_to_object = NavigateToBehaviour("导航到物体", robot_instance)
    
    # 抓取物体
    pickup_object = PickupObjectBehaviour("抓取物体", robot_instance)
    
    # 导航到目标位置
    navigate_to_target = NavigateToBehaviour("导航到目标位置", robot_instance)
    
    # 放下物体
    put_down_object = PutDownBehaviour("放下物体", robot_instance)
    
    # 构建树结构
    root.add_children([
        detection_selector,
        navigate_to_object,
        pickup_object,
        navigate_to_target,
        put_down_object
    ])
    
    return root


def create_exploration_tree(robot_instance):
    """创建一个探索任务的行为树"""
    
    # 根节点
    root = py_trees.composites.Sequence(name="探索任务", memory=False)
    
    # 规划探索路径
    plan_waypoints = PlanExplorationWaypointsBehaviour("规划探索路径", robot_instance)
    
    # 执行探索
    explore_area = ExploreBehaviour("执行探索", robot_instance)
    
    # 拍照记录
    take_photo = TakePhotoBehaviour("拍照记录", robot_instance)
    
    # 广播结果
    broadcast_result = BroadcastBehaviour("广播探索结果", robot_instance)
    
    # 构建树结构
    root.add_children([plan_waypoints, explore_area, take_photo, broadcast_result])
    
    return root


def setup_blackboard_for_navigation(goal_position, goal_orientation=None):
    """为导航任务设置黑板数据"""
    blackboard = py_trees.blackboard.Client()
    blackboard.register_key("goal_pos", access=py_trees.common.Access.WRITE)
    blackboard.register_key("goal_quat_wxyz", access=py_trees.common.Access.WRITE)
    blackboard.register_key("broadcast_content", access=py_trees.common.Access.WRITE)
    
    blackboard.goal_pos = goal_position
    blackboard.goal_quat_wxyz = goal_orientation or [1.0, 0.0, 0.0, 0.0]
    blackboard.broadcast_content = "开始执行导航任务"


def setup_blackboard_for_pickup(hand_path, object_path, distance_threshold=2.0):
    """为抓取任务设置黑板数据"""
    blackboard = py_trees.blackboard.Client()
    blackboard.register_key("robot_hand_prim_path", access=py_trees.common.Access.WRITE)
    blackboard.register_key("object_prim_path", access=py_trees.common.Access.WRITE)
    blackboard.register_key("distance_threshold", access=py_trees.common.Access.WRITE)
    blackboard.register_key("target_class", access=py_trees.common.Access.WRITE)
    
    blackboard.robot_hand_prim_path = hand_path
    blackboard.object_prim_path = object_path
    blackboard.distance_threshold = distance_threshold
    blackboard.target_class = "car"


def setup_blackboard_for_exploration(boundary_coords, holes=None):
    """为探索任务设置黑板数据"""
    blackboard = py_trees.blackboard.Client()
    blackboard.register_key("polygon_coords", access=py_trees.common.Access.WRITE)
    blackboard.register_key("holes", access=py_trees.common.Access.WRITE)
    blackboard.register_key("boundary", access=py_trees.common.Access.WRITE)
    blackboard.register_key("lane_width", access=py_trees.common.Access.WRITE)
    blackboard.register_key("robot_radius", access=py_trees.common.Access.WRITE)
    blackboard.register_key("broadcast_content", access=py_trees.common.Access.WRITE)
    
    blackboard.polygon_coords = boundary_coords
    blackboard.holes = holes
    blackboard.boundary = boundary_coords
    blackboard.lane_width = 1.0
    blackboard.robot_radius = 0.2
    blackboard.broadcast_content = "探索任务完成"


def run_behavior_tree_example(robot_instance, task_type="navigation"):
    """运行行为树示例"""
    
    # 根据任务类型创建不同的行为树
    if task_type == "navigation":
        # 设置导航任务的黑板数据
        setup_blackboard_for_navigation([10.0, 5.0, 1.0])
        root = create_simple_navigation_tree(robot_instance)
        
    elif task_type == "pickup":
        # 设置抓取任务的黑板数据
        setup_blackboard_for_pickup("/robot/hand", "/object/target")
        root = create_pickup_task_tree(robot_instance)
        
    elif task_type == "exploration":
        # 设置探索任务的黑板数据
        boundary = [[0, 0], [10, 0], [10, 10], [0, 10]]
        setup_blackboard_for_exploration(boundary)
        root = create_exploration_tree(robot_instance)
        
    else:
        raise ValueError(f"未知的任务类型: {task_type}")
    
    # 设置行为树
    root.setup_with_descendants()
    
    # 运行行为树
    print(f"开始执行{task_type}任务...")
    print(py_trees.display.unicode_tree(root, show_status=True))
    
    # 模拟行为树执行循环
    for i in range(10):  # 最多执行10次
        root.tick_once()
        print(f"\n第{i+1}次tick后的状态:")
        print(py_trees.display.unicode_tree(root, show_status=True))
        
        # 检查是否完成
        if root.status != py_trees.common.Status.RUNNING:
            break
    
    print(f"\n任务完成，最终状态: {root.status}")
    return root.status


if __name__ == "__main__":
    # 这里需要传入实际的robot实例
    # robot = YourRobotInstance()
    
    # 示例用法（需要实际的robot实例）
    print("这是py_trees行为类的使用示例")
    print("请在实际项目中传入robot实例来运行这些示例")
    
    # 示例调用（注释掉，因为需要实际的robot实例）
    # run_behavior_tree_example(robot, "navigation")
    # run_behavior_tree_example(robot, "pickup") 
    # run_behavior_tree_example(robot, "exploration")