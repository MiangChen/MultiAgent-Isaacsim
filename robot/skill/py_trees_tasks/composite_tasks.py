"""
Composite task builders that combine multiple skill types.
"""

import py_trees
from robot.skill.py_trees_behaviors import (
    NavigateToBehaviour,
    ReturnHomeBehaviour,
    PickupObjectBehaviour,
    PutDownBehaviour,
    DetectBehaviour,
    ExploreBehaviour,
    TakePhotoBehaviour,
    BroadcastBehaviour
)


def build_patrol_task_tree(robot_instance, params: dict):
    """
    构建巡逻任务的行为树
    
    Args:
        robot_instance: 机器人实例
        params: 参数字典，包含:
            - patrol_points: 巡逻点列表 [[x1,y1,z1], [x2,y2,z2], ...]
            - patrol_cycles: 巡逻循环次数 (-1为无限循环)
            - detection_at_points: 是否在巡逻点进行检测 (可选)
            - photo_at_points: 是否在巡逻点拍照 (可选)
            - target_prim: 检测目标 (可选)
    
    Returns:
        py_trees.composites.Composite: 行为树根节点
    """
    
    patrol_points = params.get("patrol_points", [])
    patrol_cycles = params.get("patrol_cycles", 1)
    
    if not patrol_points:
        raise ValueError("patrol_points参数不能为空")
    
    # 创建巡逻序列
    patrol_sequence = py_trees.composites.Sequence(
        name="巡逻序列",
        memory=False
    )
    
    # 设置黑板数据
    _setup_patrol_blackboard(params)
    
    # 为每个巡逻点创建任务
    for i, point in enumerate(patrol_points):
        point_sequence = py_trees.composites.Sequence(
            name=f"巡逻点{i+1}",
            memory=False
        )
        
        # 导航到巡逻点
        navigate_to_point = NavigateToBehaviour(f"导航到巡逻点{i+1}", robot_instance)
        point_sequence.add_child(navigate_to_point)
        
        # 可选：在巡逻点进行检测
        if params.get("detection_at_points", False):
            detect_at_point = DetectBehaviour(f"在巡逻点{i+1}检测", robot_instance)
            point_sequence.add_child(detect_at_point)
        
        # 可选：在巡逻点拍照
        if params.get("photo_at_points", False):
            photo_at_point = TakePhotoBehaviour(f"在巡逻点{i+1}拍照", robot_instance)
            point_sequence.add_child(photo_at_point)
        
        patrol_sequence.add_child(point_sequence)
    
    # 根据循环次数创建根节点
    if patrol_cycles == -1:
        # 无限循环
        root = py_trees.decorators.Repeat(
            name="无限巡逻任务",
            child=patrol_sequence,
            num_success=-1
        )
    elif patrol_cycles > 1:
        # 有限循环
        root = py_trees.decorators.Repeat(
            name=f"巡逻任务({patrol_cycles}次)",
            child=patrol_sequence,
            num_success=patrol_cycles
        )
    else:
        # 单次执行
        root = py_trees.composites.Sequence(name="单次巡逻任务", memory=False)
        root.add_child(patrol_sequence)
    
    return root


def build_delivery_task_tree(robot_instance, params: dict):
    """
    构建配送任务的行为树
    
    Args:
        robot_instance: 机器人实例
        params: 参数字典，包含:
            - pickup_location: 取货位置 [x, y, z]
            - delivery_location: 配送位置 [x, y, z]
            - object_prim_path: 货物路径
            - robot_hand_prim_path: 机械手路径
            - verify_delivery: 是否验证配送 (可选)
            - return_to_base: 是否返回基地 (可选)
    
    Returns:
        py_trees.composites.Composite: 行为树根节点
    """
    
    # 根节点 - 序列执行
    root = py_trees.composites.Sequence(
        name="配送任务",
        memory=False
    )
    
    # 设置黑板数据
    _setup_delivery_blackboard(params)
    
    children = []
    
    # 广播开始配送
    broadcast_start = BroadcastBehaviour("广播开始配送", robot_instance)
    children.append(broadcast_start)
    
    # 导航到取货位置
    navigate_to_pickup = NavigateToBehaviour("导航到取货位置", robot_instance)
    children.append(navigate_to_pickup)
    
    # 抓取货物
    pickup_item = PickupObjectBehaviour("抓取货物", robot_instance)
    children.append(pickup_item)
    
    # 导航到配送位置
    navigate_to_delivery = NavigateToBehaviour("导航到配送位置", robot_instance)
    children.append(navigate_to_delivery)
    
    # 放下货物
    deliver_item = PutDownBehaviour("配送货物", robot_instance)
    children.append(deliver_item)
    
    # 可选：验证配送
    if params.get("verify_delivery", False):
        verify_delivery = DetectBehaviour("验证配送完成", robot_instance)
        children.append(verify_delivery)
    
    # 可选：返回基地
    if params.get("return_to_base", True):
        return_to_base = ReturnHomeBehaviour("返回基地", robot_instance)
        children.append(return_to_base)
    
    # 广播配送完成
    broadcast_complete = BroadcastBehaviour("广播配送完成", robot_instance)
    children.append(broadcast_complete)
    
    # 添加所有子节点
    root.add_children(children)
    
    return root


def build_search_and_rescue_tree(robot_instance, params: dict):
    """
    构建搜救任务的行为树
    
    Args:
        robot_instance: 机器人实例
        params: 参数字典，包含:
            - search_area: 搜索区域边界
            - target_prim: 搜救目标
            - rescue_location: 救援集合点 [x, y, z]
            - max_search_time: 最大搜索时间 (可选)
            - emergency_protocols: 紧急协议 (可选)
    
    Returns:
        py_trees.composites.Composite: 行为树根节点
    """
    
    # 根节点 - 序列执行
    root = py_trees.composites.Sequence(
        name="搜救任务",
        memory=False
    )
    
    # 设置黑板数据
    _setup_search_rescue_blackboard(params)
    
    children = []
    
    # 广播开始搜救
    broadcast_start = BroadcastBehaviour("广播开始搜救", robot_instance)
    children.append(broadcast_start)
    
    # 搜索阶段 - 使用选择器，成功找到目标就停止
    search_phase = py_trees.composites.Selector(
        name="搜索阶段",
        memory=False
    )
    
    # 快速检测
    quick_detection = DetectBehaviour("快速目标检测", robot_instance)
    search_phase.add_child(quick_detection)
    
    # 区域探索搜索
    area_exploration = ExploreBehaviour("区域探索搜索", robot_instance)
    search_phase.add_child(area_exploration)
    
    children.append(search_phase)
    
    # 救援阶段 - 序列执行
    rescue_phase = py_trees.composites.Sequence(
        name="救援阶段",
        memory=False
    )
    
    # 拍照记录现场
    document_scene = TakePhotoBehaviour("记录救援现场", robot_instance)
    rescue_phase.add_child(document_scene)
    
    # 导航到救援集合点
    navigate_to_rescue = NavigateToBehaviour("导航到救援点", robot_instance)
    rescue_phase.add_child(navigate_to_rescue)
    
    # 广播发现目标
    broadcast_found = BroadcastBehaviour("广播发现目标", robot_instance)
    rescue_phase.add_child(broadcast_found)
    
    children.append(rescue_phase)
    
    # 添加所有子节点
    root.add_children(children)
    
    return root


def build_inspection_task_tree(robot_instance, params: dict):
    """
    构建检查任务的行为树
    
    Args:
        robot_instance: 机器人实例
        params: 参数字典，包含:
            - inspection_points: 检查点列表
            - inspection_criteria: 检查标准
            - photo_documentation: 是否拍照记录 (可选)
            - detailed_detection: 是否详细检测 (可选)
    
    Returns:
        py_trees.composites.Composite: 行为树根节点
    """
    
    # 根节点 - 序列执行
    root = py_trees.composites.Sequence(
        name="检查任务",
        memory=False
    )
    
    inspection_points = params.get("inspection_points", [])
    if not inspection_points:
        raise ValueError("inspection_points参数不能为空")
    
    # 设置黑板数据
    _setup_inspection_blackboard(params)
    
    children = []
    
    # 广播开始检查
    broadcast_start = BroadcastBehaviour("广播开始检查", robot_instance)
    children.append(broadcast_start)
    
    # 为每个检查点创建检查序列
    for i, point in enumerate(inspection_points):
        inspection_sequence = py_trees.composites.Sequence(
            name=f"检查点{i+1}",
            memory=False
        )
        
        # 导航到检查点
        navigate_to_point = NavigateToBehaviour(f"导航到检查点{i+1}", robot_instance)
        inspection_sequence.add_child(navigate_to_point)
        
        # 执行检测
        if params.get("detailed_detection", True):
            detect_at_point = DetectBehaviour(f"在检查点{i+1}检测", robot_instance)
            inspection_sequence.add_child(detect_at_point)
        
        # 可选：拍照记录
        if params.get("photo_documentation", True):
            photo_at_point = TakePhotoBehaviour(f"在检查点{i+1}拍照", robot_instance)
            inspection_sequence.add_child(photo_at_point)
        
        children.append(inspection_sequence)
    
    # 生成检查报告
    generate_report = BroadcastBehaviour("生成检查报告", robot_instance)
    children.append(generate_report)
    
    # 添加所有子节点
    root.add_children(children)
    
    return root


def _setup_patrol_blackboard(params: dict):
    """设置巡逻任务的黑板数据"""
    blackboard = py_trees.blackboard.Client()
    
    # 巡逻点
    patrol_points = params.get("patrol_points", [])
    for i, point in enumerate(patrol_points):
        blackboard.register_key(f"patrol_point_{i}", access=py_trees.common.Access.WRITE)
        blackboard.__setattr__(f"patrol_point_{i}", point)
    
    # 当前目标位置（会动态更新）
    blackboard.register_key("goal_pos", access=py_trees.common.Access.WRITE)
    if patrol_points:
        blackboard.goal_pos = patrol_points[0]
    
    # 检测目标
    if "target_prim" in params:
        blackboard.register_key("target_prim", access=py_trees.common.Access.WRITE)
        blackboard.target_prim = params["target_prim"]
    
    # 广播内容
    blackboard.register_key("broadcast_content", access=py_trees.common.Access.WRITE)
    blackboard.broadcast_content = "执行巡逻任务"


def _setup_delivery_blackboard(params: dict):
    """设置配送任务的黑板数据"""
    blackboard = py_trees.blackboard.Client()
    
    # 取货位置
    blackboard.register_key("pickup_pos", access=py_trees.common.Access.WRITE)
    blackboard.pickup_pos = params.get("pickup_location", [0, 0, 1])
    
    # 配送位置
    blackboard.register_key("delivery_pos", access=py_trees.common.Access.WRITE)
    blackboard.delivery_pos = params.get("delivery_location", [0, 0, 1])
    
    # 当前目标位置（会动态更新）
    blackboard.register_key("goal_pos", access=py_trees.common.Access.WRITE)
    blackboard.goal_pos = params.get("pickup_location", [0, 0, 1])
    
    # 货物和机械手路径
    blackboard.register_key("object_prim_path", access=py_trees.common.Access.WRITE)
    blackboard.object_prim_path = params.get("object_prim_path", "/World/package")
    
    blackboard.register_key("robot_hand_prim_path", access=py_trees.common.Access.WRITE)
    blackboard.robot_hand_prim_path = params.get("robot_hand_prim_path", "/robot/hand")
    
    # 广播内容
    blackboard.register_key("broadcast_content", access=py_trees.common.Access.WRITE)
    blackboard.broadcast_content = "执行配送任务"


def _setup_search_rescue_blackboard(params: dict):
    """设置搜救任务的黑板数据"""
    blackboard = py_trees.blackboard.Client()
    
    # 搜索区域
    blackboard.register_key("boundary", access=py_trees.common.Access.WRITE)
    blackboard.boundary = params.get("search_area", [])
    
    # 搜救目标
    blackboard.register_key("target_prim", access=py_trees.common.Access.WRITE)
    blackboard.target_prim = params.get("target_prim", "/rescue_target")
    
    # 救援集合点
    blackboard.register_key("rescue_pos", access=py_trees.common.Access.WRITE)
    blackboard.rescue_pos = params.get("rescue_location", [0, 0, 1])
    
    blackboard.register_key("goal_pos", access=py_trees.common.Access.WRITE)
    blackboard.goal_pos = params.get("rescue_location", [0, 0, 1])
    
    # 广播内容
    blackboard.register_key("broadcast_content", access=py_trees.common.Access.WRITE)
    blackboard.broadcast_content = "执行搜救任务"


def _setup_inspection_blackboard(params: dict):
    """设置检查任务的黑板数据"""
    blackboard = py_trees.blackboard.Client()
    
    # 检查点
    inspection_points = params.get("inspection_points", [])
    for i, point in enumerate(inspection_points):
        blackboard.register_key(f"inspection_point_{i}", access=py_trees.common.Access.WRITE)
        blackboard.__setattr__(f"inspection_point_{i}", point)
    
    # 当前目标位置
    blackboard.register_key("goal_pos", access=py_trees.common.Access.WRITE)
    if inspection_points:
        blackboard.goal_pos = inspection_points[0]
    
    # 检查标准
    if "inspection_criteria" in params:
        blackboard.register_key("inspection_criteria", access=py_trees.common.Access.WRITE)
        blackboard.inspection_criteria = params["inspection_criteria"]
    
    # 广播内容
    blackboard.register_key("broadcast_content", access=py_trees.common.Access.WRITE)
    blackboard.broadcast_content = "执行检查任务"