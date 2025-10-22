"""
Camera-related py-trees behaviors.
"""

import py_trees


class TakePhotoBehaviour(py_trees.behaviour.Behaviour):
    """拍照的行为节点"""
    
    def __init__(self, name: str, robot_instance):
        super().__init__(name)
        self.robot = robot_instance
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key("photo_file_path", access=py_trees.common.Access.READ)
        self.blackboard.register_key("photo_data", access=py_trees.common.Access.WRITE)

    def setup(self):
        """初始化设置"""
        self.logger.debug(f"  行为节点 ({self.name}): 设置完成")
        return True

    def initialise(self):
        """行为开始时的初始化"""
        self.logger.debug(f"  行为节点 ({self.name}): 开始拍照")

    def update(self):
        """更新行为状态"""
        try:
            # 获取拍照参数
            file_path = getattr(self.blackboard, 'photo_file_path', None)
            
            # 执行拍照
            from robot.skill.base.take_photo.take_photo import take_photo
            photo_data = take_photo(self.robot, file_path)
            
            if photo_data is not None:
                # 将照片数据写入黑板
                self.blackboard.photo_data = photo_data
                
                if file_path:
                    self.logger.info(f"  行为节点 ({self.name}): 拍照成功，已保存到: {file_path}")
                else:
                    self.logger.info(f"  行为节点 ({self.name}): 拍照成功，数据已存储到黑板")
                return py_trees.common.Status.SUCCESS
            else:
                self.logger.error(f"  行为节点 ({self.name}): 拍照失败，相机不可用")
                return py_trees.common.Status.FAILURE
                
        except Exception as e:
            self.logger.error(f"  行为节点 ({self.name}): 拍照异常 - {str(e)}")
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        """行为结束时的清理"""
        self.logger.debug(f"  行为节点 ({self.name}): 拍照结束，状态: {new_status}")