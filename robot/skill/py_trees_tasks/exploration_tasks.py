"""
Exploration-related task builders for py-trees behavior trees.
"""

import py_trees
from robot.skill.py_trees_behaviors import (
    ExploreBehaviour,
    PlanExplorationWaypointsBehaviour,
    DetectBehaviour,
    TakePhotoBehaviour,
    BroadcastBehaviour,
    NavigateToBehaviour
)


def build_exploration_tree(robot_instance, params: dict):
    """
    构建区域探索任务的行为树
    
    Args:
        robot_instance: 机器人实例
        params: 参数字典，包含:
            - boundary: 探索边界坐标
            - holes: 洞/障碍物坐标 (可选)
            - target_prim: 搜索目标 (可选)
            - take_photos: 是否拍照记录 (可选)
            - lane_width: 扫描宽度 (可选)
            - robot_radius: 机器人半径 (可选)
    
    Returns:
        py_trees.composites.Composite: 行为树根节点
    """
    
    # 根节点 - 序列执行
    root = py_trees.composites.Sequence(
        name="区域探索任务",
        memory=False
    )
    
    # 设置黑板数据
    _setup_exploration_blackboard(params)
    
    children = []
    
    # 广播开始探索
    broadcast_start = BroadcastBehaviour("广播开始探索", robot_instance)
    children.append(broadcast_start)
    
    # 规划探索路径点
    plan_waypoints = PlanExplorationWaypointsBehaviour("规划探索路径", robot_instance)
    children.append(plan_waypoints)
    
    # 执行探索
    explore_area = ExploreBehaviour("执行区域探索", robot_instance)
    children.append(explore_area)
    
    # 可选：拍照记录
    if params.get("take_photos", True):
        take_photo = TakePhotoBehaviour("拍照记录探索结果", robot_instance)
        children.append(take_photo)
    
    # 广播探索完成
    broadcast_complete = BroadcastBehaviour("广播探索完成", robot_instance)
    children.append(broadcast_complete)
    
    # 添加所有子节点
    root.add_children(children)
    
    return root


def build_search_and_explore_tree(robot_instance, params: dict):
    """
    构建搜索并探索任务的行为树（带目标搜索的探索）
    
    Args:
        robot_instance: 机器人实例
        params: 参数字典，包含:
            - boundary: 探索边界坐标
            - target_prim: 搜索目标
            - max_search_attempts: 最大搜索尝试次数 (可选)
            - search_areas: 重点搜索区域列表 (可选)
    
    Returns:
        py_trees.composites.Composite: 行为树根节点
    """
    
    # 根节点 - 序列执行
    root = py_trees.composites.Sequence(
        name="搜索探索任务",
        memory=False
    )
    
    # 设置黑板数据
    _setup_exploration_blackboard(params)
    
    children = []
    
    # 广播开始搜索
    broadcast_start = BroadcastBehaviour("广播开始搜索", robot_instance)
    children.append(broadcast_start)
    
    # 搜索策略选择器
    search_strategy = py_trees.composites.Selector(
        name="搜索策略",
        memory=False
    )
    
    # 策略1: 重点区域搜索
    if params.get("search_areas"):
        focused_search = _build_focused_search_sequence(robot_instance, params)
        search_strategy.add_child(focused_search)
    
    # 策略2: 全区域探索
    full_exploration = py_trees.composites.Sequence(
        name="全区域探索",
        memory=False
    )
    
    plan_full_exploration = PlanExplorationWaypointsBehaviour("规划全区域探索", robot_instance)
    execute_exploration = ExploreBehaviour("执行全区域探索", robot_instance)
    full_exploration.add_children([plan_full_exploration, execute_exploration])
    
    search_strategy.add_child(full_exploration)
    children.append(search_strategy)
    
    # 目标检测验证
    target_detection = DetectBehaviour("验证目标发现", robot_instance)
    children.append(target_detection)
    
    # 拍照记录发现
    take_photo = TakePhotoBehaviour("拍照记录发现", robot_instance)
    children.append(take_photo)
    
    # 广播搜索结果
    broadcast_result = BroadcastBehaviour("广播搜索结果", robot_instance)
    children.append(broadcast_result)
    
    # 添加所有子节点
    root.add_children(children)
    
    return root


def build_patrol_exploration_tree(robot_instance, params: dict):
    """
    构建巡逻探索任务的行为树（循环探索）
    
    Args:
        robot_instance: 机器人实例
        params: 参数字典，包含:
            - patrol_areas: 巡逻区域列表
            - patrol_cycles: 巡逻循环次数 (-1为无限循环)
            - rest_points: 休息点位置列表 (可选)
            - detection_interval: 检测间隔 (可选)
    
    Returns:
        py_trees.composites.Composite: 行为树根节点
    """
    
    patrol_cycles = params.get("patrol_cycles", -1)
    
    # 根节点 - 重复执行
    if patrol_cycles == -1:
        root = py_trees.decorators.Repeat(
            name="无限巡逻探索",
            child=py_trees.composites.Sequence(name="巡逻循环", memory=False),
            num_success=-1  # 无限循环
        )
        patrol_sequence = root.decorated
    else:
        root = py_trees.decorators.Repeat(
            name=f"巡逻探索({patrol_cycles}次)",
            child=py_trees.composites.Sequence(name="巡逻循环", memory=False),
            num_success=patrol_cycles
        )
        patrol_sequence = root.decorated
    
    # 设置黑板数据
    _setup_exploration_blackboard(params)
    
    patrol_areas = params.get("patrol_areas", [])
    if not patrol_areas:
        raise ValueError("patrol_areas参数不能为空")
    
    # 为每个巡逻区域创建探索任务
    for i, area in enumerate(patrol_areas):
        area_sequence = py_trees.composites.Sequence(
            name=f"巡逻区域{i+1}",
            memory=False
        )
        
        # 更新当前区域的黑板数据
        _update_blackboard_for_area(area, i)
        
        # 导航到区域
        if area.get("entry_point"):
            navigate_to_area = NavigateToBehaviour(f"导航到区域{i+1}", robot_instance)
            area_sequence.add_child(navigate_to_area)
        
        # 探索区域
        explore_area = ExploreBehaviour(f"探索区域{i+1}", robot_instance)
        area_sequence.add_child(explore_area)
        
        # 可选：在休息点休息
        if params.get("rest_points") and i < len(params["rest_points"]):
            navigate_to_rest = NavigateToBehaviour(f"导航到休息点{i+1}", robot_instance)
            area_sequence.add_child(navigate_to_rest)
        
        patrol_sequence.add_child(area_sequence)
    
    return root


def _build_focused_search_sequence(robot_instance, params: dict):
    """构建重点区域搜索序列"""
    
    focused_search = py_trees.composites.Sequence(
        name="重点区域搜索",
        memory=False
    )
    
    search_areas = params.get("search_areas", [])
    
    for i, area in enumerate(search_areas):
        area_search = py_trees.composites.Sequence(
            name=f"搜索区域{i+1}",
            memory=False
        )
        
        # 导航到搜索区域
        navigate_to_search = NavigateToBehaviour(f"导航到搜索区域{i+1}", robot_instance)
        
        # 在该区域进行检测
        detect_in_area = DetectBehaviour(f"在区域{i+1}检测目标", robot_instance)
        
        # 如果检测成功，则成功退出；否则继续下一个区域
        area_search.add_children([navigate_to_search, detect_in_area])
        focused_search.add_child(area_search)
    
    return focused_search


def _setup_exploration_blackboard(params: dict):
    """设置探索任务的黑板数据"""
    blackboard = py_trees.blackboard.Client()
    
    # 探索边界
    blackboard.register_key("boundary", access=py_trees.common.Access.WRITE)
    blackboard.boundary = params.get("boundary", [])
    
    blackboard.register_key("polygon_coords", access=py_trees.common.Access.WRITE)
    blackboard.polygon_coords = params.get("boundary", [])
    
    # 洞/障碍物
    blackboard.register_key("holes", access=py_trees.common.Access.WRITE)
    blackboard.holes = params.get("holes", None)
    
    # 搜索目标
    blackboard.register_key("target_prim", access=py_trees.common.Access.WRITE)
    blackboard.target_prim = params.get("target_prim", "/TARGET_PRIM_NOT_SPECIFIED")
    
    # 探索参数
    blackboard.register_key("lane_width", access=py_trees.common.Access.WRITE)
    blackboard.lane_width = params.get("lane_width", 1.0)
    
    blackboard.register_key("robot_radius", access=py_trees.common.Access.WRITE)
    blackboard.robot_radius = params.get("robot_radius", 0.2)
    
    # 拍照路径
    if "photo_path" in params:
        blackboard.register_key("photo_file_path", access=py_trees.common.Access.WRITE)
        blackboard.photo_file_path = params["photo_path"]
    
    # 广播内容
    blackboard.register_key("broadcast_content", access=py_trees.common.Access.WRITE)
    blackboard.broadcast_content = params.get("broadcast_content", "执行探索任务")


def _update_blackboard_for_area(area: dict, index: int):
    """为特定区域更新黑板数据"""
    blackboard = py_trees.blackboard.Client()
    
    # 更新当前区域边界
    if "boundary" in area:
        blackboard.boundary = area["boundary"]
        blackboard.polygon_coords = area["boundary"]
    
    # 更新入口点
    if "entry_point" in area:
        blackboard.register_key(f"area_{index}_entry", access=py_trees.common.Access.WRITE)
        blackboard.__setattr__(f"area_{index}_entry", area["entry_point"])
        blackboard.goal_pos = area["entry_point"]
    
    # 更新区域特定的目标
    if "target_prim" in area:
        blackboard.target_prim = area["target_prim"]