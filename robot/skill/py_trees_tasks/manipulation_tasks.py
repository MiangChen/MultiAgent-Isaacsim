"""
Manipulation-related task builders for py-trees behavior trees.
"""

import py_trees
from robot.skill.py_trees_behaviors import (
    NavigateToBehaviour,
    PickupObjectBehaviour,
    PutDownBehaviour,
    DetectBehaviour,
    ObjectDetectionBehaviour,
    BroadcastBehaviour
)


def build_pickup_task_tree(robot_instance, params: dict):
    """
    构建抓取任务的行为树
    
    Args:
        robot_instance: 机器人实例
        params: 参数字典，包含:
            - object_prim_path: 目标物体路径
            - robot_hand_prim_path: 机械手路径
            - target_location: 目标放置位置 [x, y, z] (可选)
            - distance_threshold: 抓取距离阈值 (可选)
            - use_detection: 是否使用检测 (可选)
    
    Returns:
        py_trees.composites.Composite: 行为树根节点
    """
    
    # 根节点 - 序列执行
    root = py_trees.composites.Sequence(
        name="抓取任务",
        memory=False
    )
    
    # 设置黑板数据
    _setup_manipulation_blackboard(params)
    
    children = []
    
    # 可选：使用检测来定位物体
    if params.get("use_detection", False):
        detection_selector = py_trees.composites.Selector(
            name="物体检测",
            memory=False
        )
        
        # 尝试语义检测
        if params.get("semantic_camera") and params.get("map_semantic"):
            object_detection = ObjectDetectionBehaviour("语义物体检测", robot_instance)
            detection_selector.add_child(object_detection)
        
        # 备用基础检测
        basic_detection = DetectBehaviour("基础物体检测", robot_instance)
        detection_selector.add_child(basic_detection)
        
        children.append(detection_selector)
    
    # 导航到物体位置
    if params.get("object_location"):
        navigate_to_object = NavigateToBehaviour("导航到物体", robot_instance)
        children.append(navigate_to_object)
    
    # 抓取物体
    pickup_object = PickupObjectBehaviour("抓取物体", robot_instance)
    children.append(pickup_object)
    
    # 可选：导航到目标位置并放下
    if params.get("target_location"):
        navigate_to_target = NavigateToBehaviour("导航到目标位置", robot_instance)
        put_down_object = PutDownBehaviour("放下物体", robot_instance)
        children.extend([navigate_to_target, put_down_object])
    
    # 广播完成消息
    broadcast_complete = BroadcastBehaviour("广播抓取完成", robot_instance)
    children.append(broadcast_complete)
    
    # 添加所有子节点
    root.add_children(children)
    
    return root


def build_manipulation_sequence_tree(robot_instance, params: dict):
    """
    构建操作序列任务的行为树（多个抓取-放置操作）
    
    Args:
        robot_instance: 机器人实例
        params: 参数字典，包含:
            - manipulation_sequence: 操作序列列表，每个元素包含:
                - object_prim_path: 物体路径
                - pickup_location: 抓取位置 [x, y, z] (可选)
                - target_location: 放置位置 [x, y, z]
            - robot_hand_prim_path: 机械手路径
    
    Returns:
        py_trees.composites.Composite: 行为树根节点
    """
    
    sequence_list = params.get("manipulation_sequence", [])
    if not sequence_list:
        raise ValueError("manipulation_sequence参数不能为空")
    
    # 根节点 - 序列执行
    root = py_trees.composites.Sequence(
        name="操作序列任务",
        memory=False
    )
    
    # 设置基础黑板数据
    _setup_manipulation_blackboard(params)
    
    # 为每个操作创建子序列
    for i, operation in enumerate(sequence_list):
        operation_sequence = py_trees.composites.Sequence(
            name=f"操作{i+1}",
            memory=False
        )
        
        # 更新黑板数据为当前操作
        _update_blackboard_for_operation(operation, i)
        
        operation_children = []
        
        # 导航到抓取位置
        if operation.get("pickup_location"):
            navigate_to_pickup = NavigateToBehaviour(f"导航到抓取位置{i+1}", robot_instance)
            operation_children.append(navigate_to_pickup)
        
        # 抓取物体
        pickup = PickupObjectBehaviour(f"抓取物体{i+1}", robot_instance)
        operation_children.append(pickup)
        
        # 导航到放置位置
        navigate_to_target = NavigateToBehaviour(f"导航到放置位置{i+1}", robot_instance)
        operation_children.append(navigate_to_target)
        
        # 放下物体
        put_down = PutDownBehaviour(f"放下物体{i+1}", robot_instance)
        operation_children.append(put_down)
        
        operation_sequence.add_children(operation_children)
        root.add_child(operation_sequence)
    
    return root


def build_sorting_task_tree(robot_instance, params: dict):
    """
    构建分拣任务的行为树
    
    Args:
        robot_instance: 机器人实例
        params: 参数字典，包含:
            - objects_to_sort: 待分拣物体列表
            - sorting_criteria: 分拣标准 (如颜色、大小等)
            - target_locations: 目标位置字典 {criteria: location}
            - robot_hand_prim_path: 机械手路径
    
    Returns:
        py_trees.composites.Composite: 行为树根节点
    """
    
    # 根节点 - 序列执行
    root = py_trees.composites.Sequence(
        name="分拣任务",
        memory=False
    )
    
    objects_to_sort = params.get("objects_to_sort", [])
    target_locations = params.get("target_locations", {})
    
    if not objects_to_sort or not target_locations:
        raise ValueError("objects_to_sort和target_locations参数不能为空")
    
    # 设置黑板数据
    _setup_manipulation_blackboard(params)
    
    # 为每个物体创建分拣子任务
    for i, obj_info in enumerate(objects_to_sort):
        sort_sequence = py_trees.composites.Sequence(
            name=f"分拣物体{i+1}",
            memory=False
        )
        
        # 检测物体属性
        detect_object = DetectBehaviour(f"检测物体{i+1}属性", robot_instance)
        
        # 根据检测结果选择目标位置的选择器
        location_selector = py_trees.composites.Selector(
            name=f"选择目标位置{i+1}",
            memory=False
        )
        
        # 为每个可能的分拣标准创建分支
        for criteria, location in target_locations.items():
            criteria_sequence = py_trees.composites.Sequence(
                name=f"分拣到{criteria}区域",
                memory=False
            )
            
            # 这里可以添加条件检查行为
            # condition_check = ConditionCheckBehaviour(f"检查是否为{criteria}")
            
            # 导航并放置
            navigate_to_sort = NavigateToBehaviour(f"导航到{criteria}区域", robot_instance)
            put_down_sorted = PutDownBehaviour(f"放置到{criteria}区域", robot_instance)
            
            criteria_sequence.add_children([navigate_to_sort, put_down_sorted])
            location_selector.add_child(criteria_sequence)
        
        # 抓取物体
        pickup_for_sort = PickupObjectBehaviour(f"抓取待分拣物体{i+1}", robot_instance)
        
        sort_sequence.add_children([detect_object, pickup_for_sort, location_selector])
        root.add_child(sort_sequence)
    
    return root


def _setup_manipulation_blackboard(params: dict):
    """设置操作任务的黑板数据"""
    blackboard = py_trees.blackboard.Client()
    
    # 机械手路径
    blackboard.register_key("robot_hand_prim_path", access=py_trees.common.Access.WRITE)
    blackboard.robot_hand_prim_path = params.get("robot_hand_prim_path", "/robot/hand")
    
    # 物体路径
    blackboard.register_key("object_prim_path", access=py_trees.common.Access.WRITE)
    blackboard.object_prim_path = params.get("object_prim_path", "/World/target_object")
    
    # 距离阈值
    blackboard.register_key("distance_threshold", access=py_trees.common.Access.WRITE)
    blackboard.distance_threshold = params.get("distance_threshold", 2.0)
    
    # 目标位置
    if "target_location" in params:
        blackboard.register_key("goal_pos", access=py_trees.common.Access.WRITE)
        blackboard.goal_pos = params["target_location"]
    
    # 物体位置
    if "object_location" in params:
        blackboard.register_key("goal_pos", access=py_trees.common.Access.WRITE)
        blackboard.goal_pos = params["object_location"]
    
    # 语义检测参数
    if "semantic_camera" in params:
        blackboard.register_key("semantic_camera", access=py_trees.common.Access.WRITE)
        blackboard.semantic_camera = params["semantic_camera"]
    
    if "map_semantic" in params:
        blackboard.register_key("map_semantic", access=py_trees.common.Access.WRITE)
        blackboard.map_semantic = params["map_semantic"]
    
    if "target_class" in params:
        blackboard.register_key("target_class", access=py_trees.common.Access.WRITE)
        blackboard.target_class = params["target_class"]
    
    # 广播内容
    blackboard.register_key("broadcast_content", access=py_trees.common.Access.WRITE)
    blackboard.broadcast_content = params.get("broadcast_content", "执行操作任务")


def _update_blackboard_for_operation(operation: dict, index: int):
    """为特定操作更新黑板数据"""
    blackboard = py_trees.blackboard.Client()
    
    # 更新物体路径
    if "object_prim_path" in operation:
        blackboard.object_prim_path = operation["object_prim_path"]
    
    # 更新目标位置
    if "target_location" in operation:
        blackboard.goal_pos = operation["target_location"]
    
    # 更新抓取位置
    if "pickup_location" in operation:
        blackboard.register_key(f"pickup_pos_{index}", access=py_trees.common.Access.WRITE)
        blackboard.__setattr__(f"pickup_pos_{index}", operation["pickup_location"])