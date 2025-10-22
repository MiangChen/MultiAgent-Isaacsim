"""
Manipulation-related py-trees behaviors.
"""

import py_trees


class PickupObjectBehaviour(py_trees.behaviour.Behaviour):
    """抓取物体的行为节点"""
    
    def __init__(self, name: str, robot_instance):
        super().__init__(name)
        self.robot = robot_instance
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key("robot_hand_prim_path", access=py_trees.common.Access.READ)
        self.blackboard.register_key("object_prim_path", access=py_trees.common.Access.READ)
        self.blackboard.register_key("distance_threshold", access=py_trees.common.Access.READ)
        
        # 抓取状态跟踪
        self._pickup_generator = None
        self._is_running = False

    def setup(self):
        """初始化设置"""
        self.logger.debug(f"  行为节点 ({self.name}): 设置完成")
        return True

    def initialise(self):
        """行为开始时的初始化"""
        self.logger.debug(f"  行为节点 ({self.name}): 开始抓取物体")
        self._is_running = True
        
        # 获取抓取参数
        robot_hand_prim_path = getattr(self.blackboard, 'robot_hand_prim_path', None)
        object_prim_path = getattr(self.blackboard, 'object_prim_path', None)
        distance_threshold = getattr(self.blackboard, 'distance_threshold', 2.0)
        
        if not robot_hand_prim_path or not object_prim_path:
            self.logger.error(f"  行为节点 ({self.name}): 缺少必要的抓取参数")
            return
            
        # 启动抓取技能
        from robot.skill.base.manipulation.pickup_object import pickup_object_skill
        self._pickup_generator = pickup_object_skill(
            robot=self.robot,
            robot_hand_prim_path=robot_hand_prim_path,
            object_prim_path=object_prim_path,
            distance_threshold=distance_threshold
        )

    def update(self):
        """更新行为状态"""
        if not self._is_running:
            return py_trees.common.Status.FAILURE
            
        if self._pickup_generator is None:
            return py_trees.common.Status.FAILURE
            
        try:
            # 获取抓取进度
            feedback = next(self._pickup_generator)
            
            if feedback["status"] == "processing":
                self.logger.info(f"  行为节点 ({self.name}): 抓取进行中... {feedback.get('message', '')}")
                return py_trees.common.Status.RUNNING
            elif feedback["status"] == "success":
                self.logger.info(f"  行为节点 ({self.name}): 抓取成功")
                return py_trees.common.Status.SUCCESS
            else:
                self.logger.error(f"  行为节点 ({self.name}): 抓取失败 - {feedback.get('message', '')}")
                return py_trees.common.Status.FAILURE
                
        except StopIteration as e:
            # 生成器结束，检查返回值
            if hasattr(e, 'value') and e.value:
                feedback = e.value
                if feedback["status"] == "success":
                    return py_trees.common.Status.SUCCESS
                else:
                    return py_trees.common.Status.FAILURE
            return py_trees.common.Status.FAILURE
        except Exception as e:
            self.logger.error(f"  行为节点 ({self.name}): 抓取异常 - {str(e)}")
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        """行为结束时的清理"""
        self._is_running = False
        self._pickup_generator = None
        self.logger.debug(f"  行为节点 ({self.name}): 抓取结束，状态: {new_status}")


class PutDownBehaviour(py_trees.behaviour.Behaviour):
    """放下物体的行为节点"""
    
    def __init__(self, name: str, robot_instance):
        super().__init__(name)
        self.robot = robot_instance
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key("robot_hand_prim_path", access=py_trees.common.Access.READ)
        self.blackboard.register_key("object_prim_path", access=py_trees.common.Access.READ)
        
        # 放下状态跟踪
        self._putdown_generator = None
        self._is_running = False

    def setup(self):
        """初始化设置"""
        self.logger.debug(f"  行为节点 ({self.name}): 设置完成")
        return True

    def initialise(self):
        """行为开始时的初始化"""
        self.logger.debug(f"  行为节点 ({self.name}): 开始放下物体")
        self._is_running = True
        
        # 获取放下参数
        robot_hand_prim_path = getattr(self.blackboard, 'robot_hand_prim_path', None)
        object_prim_path = getattr(self.blackboard, 'object_prim_path', None)
        
        if not robot_hand_prim_path or not object_prim_path:
            self.logger.error(f"  行为节点 ({self.name}): 缺少必要的放下参数")
            return
            
        # 启动放下技能
        from robot.skill.base.manipulation.put_down import put_down_skill
        self._putdown_generator = put_down_skill(
            robot=self.robot,
            robot_hand_prim_path=robot_hand_prim_path,
            object_prim_path=object_prim_path
        )

    def update(self):
        """更新行为状态"""
        if not self._is_running:
            return py_trees.common.Status.FAILURE
            
        if self._putdown_generator is None:
            return py_trees.common.Status.FAILURE
            
        try:
            # 获取放下进度
            feedback = next(self._putdown_generator)
            
            if feedback["status"] == "processing":
                self.logger.info(f"  行为节点 ({self.name}): 放下进行中... {feedback.get('message', '')}")
                return py_trees.common.Status.RUNNING
            elif feedback["status"] == "success":
                self.logger.info(f"  行为节点 ({self.name}): 放下成功")
                return py_trees.common.Status.SUCCESS
            else:
                self.logger.error(f"  行为节点 ({self.name}): 放下失败 - {feedback.get('message', '')}")
                return py_trees.common.Status.FAILURE
                
        except StopIteration as e:
            # 生成器结束，检查返回值
            if hasattr(e, 'value') and e.value:
                feedback = e.value
                if feedback["status"] == "success":
                    return py_trees.common.Status.SUCCESS
                else:
                    return py_trees.common.Status.FAILURE
            return py_trees.common.Status.FAILURE
        except Exception as e:
            self.logger.error(f"  行为节点 ({self.name}): 放下异常 - {str(e)}")
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        """行为结束时的清理"""
        self._is_running = False
        self._putdown_generator = None
        self.logger.debug(f"  行为节点 ({self.name}): 放下结束，状态: {new_status}")