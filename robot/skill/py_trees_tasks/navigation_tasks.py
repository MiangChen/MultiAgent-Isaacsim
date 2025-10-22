"""
Navigation-related task builders for py-trees behavior trees.
"""

import py_trees
from robot.skill.py_trees_behaviors import (
    NavigateToBehaviour, 
    ReturnHomeBehaviour,
    BroadcastBehaviour,
    TakePhotoBehaviour
)


def build_navigate_and_return_tree(robot_instance, params: dict):
    """
    构建导航并返回的行为树
    
    Args:
        robot_instance: 机器人实例
        params: 参数字典，包含:
            - goal_pos: 目标位置 [x, y, z]
            - goal_quat_wxyz: 目标姿态 [w, x, y, z] (可选)
            - take_photo: 是否在目标位置拍照 (可选)
            - broadcast_messages: 是否广播消息 (可选)
    
    Returns:
        py_trees.composites.Composite: 行为树根节点
    """
    
    # 根节点 - 序列执行
    root = py_trees.composites.Sequence(
        name="导航并返回任务",
        memory=False
    )
    
    # 设置黑板数据
    _setup_navigation_blackboard(params)
    
    children = []
    
    # 可选：开始广播
    if params.get("broadcast_messages", True):
        broadcast_start = BroadcastBehaviour("广播任务开始", robot_instance)
        children.append(broadcast_start)
    
    # 导航到目标位置
    navigate_to_goal = NavigateToBehaviour("导航到目标位置", robot_instance)
    children.append(navigate_to_goal)
    
    # 可选：在目标位置拍照
    if params.get("take_photo", False):
        take_photo = TakePhotoBehaviour("目标位置拍照", robot_instance)
        children.append(take_photo)
    
    # 返回起始位置
    return_home = ReturnHomeBehaviour("返回起始位置", robot_instance)
    children.append(return_home)
    
    # 可选：结束广播
    if params.get("broadcast_messages", True):
        broadcast_end = BroadcastBehaviour("广播任务完成", robot_instance)
        children.append(broadcast_end)
    
    # 添加所有子节点
    root.add_children(children)
    
    return root


def build_simple_navigation_tree(robot_instance, params: dict):
    """
    构建简单导航任务的行为树
    
    Args:
        robot_instance: 机器人实例
        params: 参数字典，包含:
            - goal_pos: 目标位置 [x, y, z]
            - goal_quat_wxyz: 目标姿态 [w, x, y, z] (可选)
    
    Returns:
        py_trees.composites.Composite: 行为树根节点
    """
    
    # 根节点 - 序列执行
    root = py_trees.composites.Sequence(
        name="简单导航任务",
        memory=False
    )
    
    # 设置黑板数据
    _setup_navigation_blackboard(params)
    
    # 导航到目标位置
    navigate_to_goal = NavigateToBehaviour("导航到目标", robot_instance)
    root.add_child(navigate_to_goal)
    
    return root


def build_waypoint_navigation_tree(robot_instance, params: dict):
    """
    构建多点导航任务的行为树
    
    Args:
        robot_instance: 机器人实例
        params: 参数字典，包含:
            - waypoints: 路径点列表 [[x1,y1,z1], [x2,y2,z2], ...]
            - loop: 是否循环执行 (可选)
    
    Returns:
        py_trees.composites.Composite: 行为树根节点
    """
    
    waypoints = params.get("waypoints", [])
    if not waypoints:
        raise ValueError("waypoints参数不能为空")
    
    # 根节点
    if params.get("loop", False):
        root = py_trees.decorators.Repeat(
            name="循环多点导航",
            child=py_trees.composites.Sequence(name="路径点序列", memory=False),
            num_success=-1  # 无限循环
        )
        sequence = root.decorated
    else:
        root = py_trees.composites.Sequence(name="多点导航任务", memory=False)
        sequence = root
    
    # 为每个路径点创建导航行为
    for i, waypoint in enumerate(waypoints):
        # 设置当前路径点的黑板数据
        blackboard = py_trees.blackboard.Client()
        blackboard.register_key(f"waypoint_{i}_pos", access=py_trees.common.Access.WRITE)
        blackboard.__setattr__(f"waypoint_{i}_pos", waypoint)
        
        # 创建导航行为
        navigate_behavior = NavigateToBehaviour(f"导航到路径点{i+1}", robot_instance)
        
        # 创建一个装饰器来设置特定路径点的目标
        waypoint_setter = _create_waypoint_setter(waypoint, navigate_behavior)
        sequence.add_child(waypoint_setter)
    
    return root


def _setup_navigation_blackboard(params: dict):
    """设置导航任务的黑板数据"""
    blackboard = py_trees.blackboard.Client()
    
    # 注册并设置目标位置
    blackboard.register_key("goal_pos", access=py_trees.common.Access.WRITE)
    blackboard.goal_pos = params.get("goal_pos", [0.0, 0.0, 1.0])
    
    # 注册并设置目标姿态
    blackboard.register_key("goal_quat_wxyz", access=py_trees.common.Access.WRITE)
    blackboard.goal_quat_wxyz = params.get("goal_quat_wxyz", [1.0, 0.0, 0.0, 0.0])
    
    # 注册广播内容
    blackboard.register_key("broadcast_content", access=py_trees.common.Access.WRITE)
    blackboard.broadcast_content = params.get("broadcast_content", "执行导航任务")
    
    # 注册拍照路径
    if "photo_path" in params:
        blackboard.register_key("photo_file_path", access=py_trees.common.Access.WRITE)
        blackboard.photo_file_path = params["photo_path"]


def _create_waypoint_setter(waypoint, navigation_behavior):
    """创建路径点设置装饰器"""
    
    class WaypointSetter(py_trees.behaviour.Behaviour):
        def __init__(self, name, waypoint_pos, child_behavior):
            super().__init__(name)
            self.waypoint_pos = waypoint_pos
            self.child_behavior = child_behavior
            self.blackboard = self.attach_blackboard_client()
            self.blackboard.register_key("goal_pos", access=py_trees.common.Access.WRITE)
        
        def update(self):
            # 设置当前路径点为目标
            self.blackboard.goal_pos = self.waypoint_pos
            # 执行导航行为
            return self.child_behavior.update()
        
        def setup(self):
            return self.child_behavior.setup()
        
        def initialise(self):
            self.child_behavior.initialise()
        
        def terminate(self, new_status):
            self.child_behavior.terminate(new_status)
    
    return WaypointSetter(f"设置路径点{waypoint}", waypoint, navigation_behavior)