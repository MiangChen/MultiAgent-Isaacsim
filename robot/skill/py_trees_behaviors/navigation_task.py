import py_trees
from robot.skill.py_trees_behaviors.navigation_behaviors import NavigateToBehaviour, ReturnHomeBehaviour

def build_navigate_and_return_tree(robot_instance, params: dict):
    """
    构建一个“导航到目标点然后返回”的任务树。
    这个函数被 pyproject.toml 注册为了一个插件。

    Args:
        robot_instance: 机器人实例，用于传递给行为节点。
        params: 从 Action Goal 传入的参数字典。
                预期包含 'goal_pos' 和 'goal_quat_wxyz'。

    Returns:
        py_trees.behaviour.Behaviour: 构建好的行为树的根节点。
    """
    # 1. 初始化黑板 (Blackboard)
    #    黑板是行为树内部的数据共享中心。
    #    我们将外部传入的参数写入黑板，供树中的节点读取。
    blackboard = py_trees.blackboard.Client()
    blackboard.register_key("goal_pos", access=py_trees.common.Access.WRITE)
    blackboard.register_key("goal_quat_wxyz", access=py_trees.common.Access.WRITE)

    # 从参数中获取值，如果不存在则提供默认值
    blackboard.goal_pos = params.get('goal_pos')
    blackboard.goal_quat_wxyz = params.get('goal_quat_wxyz', [1.0, 0.0, 0.0, 0.0])

    # 2. 创建行为节点 (Behaviours) 的实例
    #    这些是我们任务流程中的具体步骤。
    navigate_node = NavigateToBehaviour(name="前往目标点", robot_instance=robot_instance)
    return_home_node = ReturnHomeBehaviour(name="返回起始点", robot_instance=robot_instance)

    # 3. 组装行为树 (Compose the Tree)
    #    我们使用一个 Sequence 复合节点，它会按顺序执行子节点。
    #    只有当前一个成功后，才会执行下一个。
    root = py_trees.composites.Sequence(name="NavigateAndReturnTask", memory=True)
    root.add_children([navigate_node, return_home_node])

    return root