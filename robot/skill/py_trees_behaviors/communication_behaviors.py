"""
Communication-related py-trees behaviors.
"""

import py_trees


class BroadcastBehaviour(py_trees.behaviour.Behaviour):
    """广播消息的行为节点"""
    
    def __init__(self, name: str, robot_instance):
        super().__init__(name)
        self.robot = robot_instance
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key("broadcast_content", access=py_trees.common.Access.READ)

    def setup(self):
        """初始化设置"""
        self.logger.debug(f"  行为节点 ({self.name}): 设置完成")
        return True

    def initialise(self):
        """行为开始时的初始化"""
        self.logger.debug(f"  行为节点 ({self.name}): 开始广播消息")

    def update(self):
        """更新行为状态"""
        try:
            # 获取广播内容
            content = getattr(self.blackboard, 'broadcast_content', "默认广播消息")
            
            # 执行广播
            from robot.skill.base.broadcast.broadcast import broadcast
            result = broadcast(self.robot, {"content": content})
            
            if result:
                self.logger.info(f"  行为节点 ({self.name}): 广播成功，内容: {content}")
                return py_trees.common.Status.SUCCESS
            else:
                self.logger.error(f"  行为节点 ({self.name}): 广播失败")
                return py_trees.common.Status.FAILURE
                
        except Exception as e:
            self.logger.error(f"  行为节点 ({self.name}): 广播异常 - {str(e)}")
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        """行为结束时的清理"""
        self.logger.debug(f"  行为节点 ({self.name}): 广播结束，状态: {new_status}")