import py_trees
from importlib.metadata import entry_points
from typing import Dict, Any, Optional
import logging


class BehaviorTreeManager:
    """
    行为树管理器，负责加载和管理机器人任务的行为树
    """
    
    def __init__(self, robot_instance):
        self.robot = robot_instance
        self._task_builders = {}
        self._current_tree = None
        self._logger = logging.getLogger(__name__)
        self._load_tasks_from_entry_points()

    def _load_tasks_from_entry_points(self):
        """从entry points加载任务构建器"""
        self._logger.info("BT Manager: 正在从 entry_points 加载任务插件...")
        
        try:
            discovered_tasks = entry_points(group='robot.tasks')
            
            if not discovered_tasks:
                self._logger.warning("未发现任何任务插件，尝试手动加载...")
                self._load_tasks_manually()
                return
            
            for entry in discovered_tasks:
                try:
                    self._logger.info(f"  - 发现任务插件: '{entry.name}'")
                    builder_func = entry.load()
                    self._task_builders[entry.name] = builder_func
                    self._logger.debug(f"    成功加载: {entry.value}")
                except Exception as e:
                    self._logger.error(f"    加载失败 '{entry.name}': {e}")
            
            self._logger.info(f"BT Manager: 成功加载 {len(self._task_builders)} 个任务插件")
            
        except Exception as e:
            self._logger.error(f"加载entry points时发生错误: {e}")
            self._logger.info("尝试手动加载任务...")
            self._load_tasks_manually()

    def _load_tasks_manually(self):
        """手动加载任务构建器（当entry points不可用时的备选方案）"""
        self._logger.info("BT Manager: 手动加载任务插件...")
        
        # 手动导入任务构建器
        task_mappings = {
            # Navigation tasks
            "navigate_and_return": ("robot.skill.py_trees_tasks.navigation_tasks", "build_navigate_and_return_tree"),
            "simple_navigation": ("robot.skill.py_trees_tasks.navigation_tasks", "build_simple_navigation_tree"),
            "waypoint_navigation": ("robot.skill.py_trees_tasks.navigation_tasks", "build_waypoint_navigation_tree"),
            
            # Manipulation tasks
            "pickup_task": ("robot.skill.py_trees_tasks.manipulation_tasks", "build_pickup_task_tree"),
            "manipulation_sequence": ("robot.skill.py_trees_tasks.manipulation_tasks", "build_manipulation_sequence_tree"),
            "sorting_task": ("robot.skill.py_trees_tasks.manipulation_tasks", "build_sorting_task_tree"),
            
            # Exploration tasks
            "exploration": ("robot.skill.py_trees_tasks.exploration_tasks", "build_exploration_tree"),
            "search_and_explore": ("robot.skill.py_trees_tasks.exploration_tasks", "build_search_and_explore_tree"),
            "patrol_exploration": ("robot.skill.py_trees_tasks.exploration_tasks", "build_patrol_exploration_tree"),
            
            # Composite tasks
            "patrol_task": ("robot.skill.py_trees_tasks.composite_tasks", "build_patrol_task_tree"),
            "delivery_task": ("robot.skill.py_trees_tasks.composite_tasks", "build_delivery_task_tree"),
            "search_and_rescue": ("robot.skill.py_trees_tasks.composite_tasks", "build_search_and_rescue_tree"),
            "inspection_task": ("robot.skill.py_trees_tasks.composite_tasks", "build_inspection_task_tree"),
        }
        
        for task_name, (module_path, function_name) in task_mappings.items():
            try:
                # 动态导入模块
                import importlib
                module = importlib.import_module(module_path)
                builder_func = getattr(module, function_name)
                self._task_builders[task_name] = builder_func
                self._logger.info(f"  - 手动加载任务: '{task_name}'")
            except Exception as e:
                self._logger.error(f"  - 手动加载失败 '{task_name}': {e}")
        
        if self._task_builders:
            self._logger.info(f"BT Manager: 手动加载完成，共 {len(self._task_builders)} 个任务插件")
        else:
            self._logger.error("BT Manager: 没有成功加载任何任务插件")

    def get_available_tasks(self) -> list:
        """获取所有可用的任务名称"""
        return list(self._task_builders.keys())

    def create_tree_for_task(self, task_name: str, params: dict) -> py_trees.trees.BehaviourTree:
        """
        为指定任务创建行为树
        
        Args:
            task_name: 任务名称
            params: 任务参数
            
        Returns:
            py_trees.trees.BehaviourTree: 创建的行为树
            
        Raises:
            ValueError: 当任务名称不存在时
        """
        builder = self._task_builders.get(task_name)
        if not builder:
            available_tasks = list(self._task_builders.keys())
            raise ValueError(
                f"未知任务 '{task_name}'。\n"
                f"可用任务: {available_tasks}"
            )

        try:
            self._logger.info(f"创建任务 '{task_name}' 的行为树...")
            root_node = builder(self.robot, params)
            tree = py_trees.trees.BehaviourTree(root=root_node)
            tree.setup(timeout=20)
            
            self._current_tree = tree
            self._logger.info(f"成功创建任务 '{task_name}' 的行为树")
            return tree
            
        except Exception as e:
            self._logger.error(f"创建任务 '{task_name}' 的行为树时发生错误: {e}")
            raise

    def execute_task(self, task_name: str, params: dict, max_ticks: int = 1000) -> py_trees.common.Status:
        """
        执行指定任务
        
        Args:
            task_name: 任务名称
            params: 任务参数
            max_ticks: 最大tick次数
            
        Returns:
            py_trees.common.Status: 任务执行结果
        """
        tree = self.create_tree_for_task(task_name, params)
        
        self._logger.info(f"开始执行任务 '{task_name}'...")
        self._logger.debug(f"行为树结构:\n{py_trees.display.unicode_tree(tree.root, show_status=True)}")
        
        tick_count = 0
        while tree.root.status == py_trees.common.Status.RUNNING and tick_count < max_ticks:
            tree.tick()
            tick_count += 1
            
            if tick_count % 10 == 0:  # 每10次tick记录一次状态
                self._logger.debug(f"任务 '{task_name}' 执行中... (tick: {tick_count})")
        
        final_status = tree.root.status
        self._logger.info(f"任务 '{task_name}' 执行完成，状态: {final_status}, 总tick数: {tick_count}")
        
        if tick_count >= max_ticks:
            self._logger.warning(f"任务 '{task_name}' 达到最大tick次数限制 ({max_ticks})")
        
        return final_status

    def get_current_tree_status(self) -> Optional[py_trees.common.Status]:
        """获取当前行为树的状态"""
        if self._current_tree:
            return self._current_tree.root.status
        return None

    def display_current_tree(self) -> str:
        """显示当前行为树的结构"""
        if self._current_tree:
            return py_trees.display.unicode_tree(self._current_tree.root, show_status=True)
        return "没有活动的行为树"

    def stop_current_task(self):
        """停止当前任务"""
        if self._current_tree:
            self._logger.info("停止当前任务...")
            # 这里可以添加停止逻辑，比如发送停止信号等
            self._current_tree = None

    def reload_tasks(self):
        """重新加载任务插件"""
        self._logger.info("重新加载任务插件...")
        self._task_builders.clear()
        self._load_tasks_from_entry_points()
