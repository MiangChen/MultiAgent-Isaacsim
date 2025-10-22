"""
Detection-related py-trees behaviors.
"""

import py_trees


class DetectBehaviour(py_trees.behaviour.Behaviour):
    """目标检测的行为节点"""
    
    def __init__(self, name: str, robot_instance):
        super().__init__(name)
        self.robot = robot_instance
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key("target_prim", access=py_trees.common.Access.READ)
        self.blackboard.register_key("detection_result", access=py_trees.common.Access.WRITE)

    def setup(self):
        """初始化设置"""
        self.logger.debug(f"  行为节点 ({self.name}): 设置完成")
        return True

    def initialise(self):
        """行为开始时的初始化"""
        self.logger.debug(f"  行为节点 ({self.name}): 开始目标检测")

    def update(self):
        """更新行为状态"""
        try:
            # 获取检测目标
            target_prim = getattr(self.blackboard, 'target_prim', None)
            
            if target_prim is None:
                self.logger.error(f"  行为节点 ({self.name}): 未指定检测目标")
                return py_trees.common.Status.FAILURE
            
            from robot.skill.base.detection.detect import detect_skill
            result = detect_skill(robot=self.robot, target_prim=target_prim)
            
            # 将检测结果写入黑板
            self.blackboard.detection_result = result.get("data", False)
            
            if result.get("success", False):
                self.logger.info(f"  行为节点 ({self.name}): 检测完成，结果: {result.get('data', False)}")
                return py_trees.common.Status.SUCCESS
            else:
                self.logger.error(f"  行为节点 ({self.name}): 检测失败 - {result.get('message', '')}")
                return py_trees.common.Status.FAILURE
                
        except Exception as e:
            self.logger.error(f"  行为节点 ({self.name}): 检测异常 - {str(e)}")
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        """行为结束时的清理"""
        self.logger.debug(f"  行为节点 ({self.name}): 检测结束，状态: {new_status}")


class ObjectDetectionBehaviour(py_trees.behaviour.Behaviour):
    """基于语义相机的物体检测行为节点"""
    
    def __init__(self, name: str, robot_instance):
        super().__init__(name)
        self.robot = robot_instance
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key("semantic_camera", access=py_trees.common.Access.READ)
        self.blackboard.register_key("map_semantic", access=py_trees.common.Access.READ)
        self.blackboard.register_key("target_class", access=py_trees.common.Access.READ)
        self.blackboard.register_key("detected_object", access=py_trees.common.Access.WRITE)

    def setup(self):
        """初始化设置"""
        self.logger.debug(f"  行为节点 ({self.name}): 设置完成")
        return True

    def initialise(self):
        """行为开始时的初始化"""
        self.logger.debug(f"  行为节点 ({self.name}): 开始物体检测")

    def update(self):
        """更新行为状态"""
        try:
            # 获取检测参数
            semantic_camera = getattr(self.blackboard, 'semantic_camera', None)
            map_semantic = getattr(self.blackboard, 'map_semantic', None)
            target_class = getattr(self.blackboard, 'target_class', 'car')
            
            if semantic_camera is None or map_semantic is None:
                self.logger.error(f"  行为节点 ({self.name}): 缺少语义相机或语义地图")
                return py_trees.common.Status.FAILURE
            
            from robot.skill.base.object_detection.object_detection import object_detection_skill
            result = object_detection_skill(
                semantic_camera=semantic_camera,
                map_semantic=map_semantic,
                target_class=target_class
            )
            
            # 将检测结果写入黑板
            self.blackboard.detected_object = result.get("data", None)
            
            if result.get("success", False):
                self.logger.info(f"  行为节点 ({self.name}): 物体检测成功，发现: {target_class}")
                return py_trees.common.Status.SUCCESS
            else:
                self.logger.info(f"  行为节点 ({self.name}): 未检测到目标物体: {target_class}")
                return py_trees.common.Status.FAILURE
                
        except Exception as e:
            self.logger.error(f"  行为节点 ({self.name}): 物体检测异常 - {str(e)}")
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        """行为结束时的清理"""
        self.logger.debug(f"  行为节点 ({self.name}): 物体检测结束，状态: {new_status}")