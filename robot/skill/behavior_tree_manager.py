import py_trees
from importlib.metadata import entry_points


class BehaviorTreeManager:
    def __init__(self, robot_instance):
        self.robot = robot_instance
        self._task_builders = {}
        self._load_tasks_from_entry_points()

    def _load_tasks_from_entry_points(self):
        print("BT Manager: 正在从 entry_points 加载任务插件...")
        discovered_tasks = entry_points(group='robot.tasks')
        for entry in discovered_tasks:
            print(f"  - 发现任务插件: '{entry.name}'")
            self._task_builders[entry.name] = entry.load()

    def create_tree_for_task(self, task_name: str, params: dict):
        builder = self._task_builders.get(task_name)
        if not builder:
            raise ValueError(f"未知任务'{task_name}'。已加载: {list(self._task_builders.keys())}")

        root_node = builder(self.robot, params)
        tree = py_trees.trees.BehaviourTree(root=root_node)
        tree.setup_with_descendants()
        return tree
