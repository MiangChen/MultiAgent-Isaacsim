"""
Navigation-related py-trees behaviors.
"""

import py_trees
from typing import List, Optional


class NavigateToBehaviour(py_trees.behaviour.Behaviour):
    """导航到指定位置的行为节点"""
    
    def __init__(self, name: str, robot_instance):
        super().__init__(name)
        self.robot = robot_instance
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key("goal_pos", access=py_trees.common.Access.READ)
        self.blackboard.register_key("goal_quat_wxyz", access=py_trees.common.Access.READ)
        
        # 导航状态跟踪
        self._navigation_generator = None
        self._is_running = False

    def setup(self):
        """初始化设置"""
        self.logger.debug(f"  行为节点 ({self.name}): 设置完成")
        return True

    def initialise(self):
        """行为开始时的初始化"""
        self.logger.debug(f"  行为节点 ({self.name}): 开始导航")
        self._is_running = True
        
        # 获取目标位置和姿态
        goal_pos = getattr(self.blackboard, 'goal_pos', None)
        goal_quat_wxyz = getattr(self.blackboard, 'goal_quat_wxyz', [1.0, 0.0, 0.0, 0.0])
        
        if goal_pos is None:
            self.logger.error(f"  行为节点 ({self.name}): 未找到目标位置")
            return
            
        # 启动导航技能
        from robot.skill.base.navigation.navigate_to import navigate_to_skill
        self._navigation_generator = navigate_to_skill(
            robot=self.robot,
            goal_pos=goal_pos,
            goal_quat_wxyz=goal_quat_wxyz
        )

    def update(self):
        """更新行为状态"""
        if not self._is_running:
            return py_trees.common.Status.FAILURE
            
        if self._navigation_generator is None:
            return py_trees.common.Status.FAILURE
            
        try:
            # 获取导航进度
            feedback = next(self._navigation_generator)
            
            if feedback["status"] == "processing":
                self.logger.info(f"  行为节点 ({self.name}): 导航进行中... {feedback.get('message', '')}")
                return py_trees.common.Status.RUNNING
            elif feedback["status"] == "finished":
                self.logger.info(f"  行为节点 ({self.name}): 导航完成")
                return py_trees.common.Status.SUCCESS
            else:
                self.logger.error(f"  行为节点 ({self.name}): 导航失败 - {feedback.get('message', '')}")
                return py_trees.common.Status.FAILURE
                
        except StopIteration as e:
            # 生成器结束，检查返回值
            if hasattr(e, 'value') and e.value:
                feedback = e.value
                if feedback["status"] == "finished":
                    return py_trees.common.Status.SUCCESS
                else:
                    return py_trees.common.Status.FAILURE
            return py_trees.common.Status.FAILURE
        except Exception as e:
            self.logger.error(f"  行为节点 ({self.name}): 导航异常 - {str(e)}")
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        """行为结束时的清理"""
        self._is_running = False
        self._navigation_generator = None
        self.logger.debug(f"  行为节点 ({self.name}): 导航结束，状态: {new_status}")


class ReturnHomeBehaviour(py_trees.behaviour.Behaviour):
    """返回起始位置的行为节点"""
    
    def __init__(self, name: str, robot_instance):
        super().__init__(name)
        self.robot = robot_instance
        self.blackboard = self.attach_blackboard_client()

    def setup(self):
        """初始化设置"""
        self.logger.debug(f"  行为节点 ({self.name}): 设置完成")
        return True

    def initialise(self):
        """行为开始时的初始化"""
        self.logger.debug(f"  行为节点 ({self.name}): 开始返回起始位置")

    def update(self):
        """更新行为状态"""
        try:
            from robot.skill.base.navigation.return_home import return_home_skill
            result = return_home_skill(robot=self.robot)
            
            if result.get("success", False):
                self.logger.info(f"  行为节点 ({self.name}): 成功返回起始位置")
                return py_trees.common.Status.SUCCESS
            else:
                self.logger.error(f"  行为节点 ({self.name}): 返回起始位置失败")
                return py_trees.common.Status.FAILURE
                
        except Exception as e:
            self.logger.error(f"  行为节点 ({self.name}): 返回起始位置异常 - {str(e)}")
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        """行为结束时的清理"""
        self.logger.debug(f"  行为节点 ({self.name}): 返回起始位置结束，状态: {new_status}")