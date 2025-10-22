"""
Exploration-related py-trees behaviors.
"""

import py_trees


class ExploreBehaviour(py_trees.behaviour.Behaviour):
    """区域探索的行为节点"""
    
    def __init__(self, name: str, robot_instance):
        super().__init__(name)
        self.robot = robot_instance
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key("boundary", access=py_trees.common.Access.READ)
        self.blackboard.register_key("holes", access=py_trees.common.Access.READ)
        self.blackboard.register_key("target_prim", access=py_trees.common.Access.READ)
        
        # 探索状态跟踪
        self._exploration_generator = None
        self._is_running = False

    def setup(self):
        """初始化设置"""
        self.logger.debug(f"  行为节点 ({self.name}): 设置完成")
        return True

    def initialise(self):
        """行为开始时的初始化"""
        self.logger.debug(f"  行为节点 ({self.name}): 开始区域探索")
        self._is_running = True
        
        # 获取探索参数
        boundary = getattr(self.blackboard, 'boundary', None)
        holes = getattr(self.blackboard, 'holes', None)
        target_prim = getattr(self.blackboard, 'target_prim', "/TARGET_PRIM_NOT_SPECIFIED")
        
        if boundary is None:
            self.logger.error(f"  行为节点 ({self.name}): 未指定探索边界")
            return
            
        # 启动探索技能
        from robot.skill.base.exploration.explore import explore_skill
        self._exploration_generator = explore_skill(
            robot=self.robot,
            boundary=boundary,
            holes=holes,
            target_prim=target_prim
        )

    def update(self):
        """更新行为状态"""
        if not self._is_running:
            return py_trees.common.Status.FAILURE
            
        if self._exploration_generator is None:
            return py_trees.common.Status.FAILURE
            
        try:
            # 获取探索进度
            feedback = next(self._exploration_generator)
            
            if feedback["status"] == "processing":
                self.logger.info(f"  行为节点 ({self.name}): 探索进行中... {feedback.get('message', '')}")
                return py_trees.common.Status.RUNNING
            elif feedback["status"] == "finished":
                self.logger.info(f"  行为节点 ({self.name}): 探索完成")
                return py_trees.common.Status.SUCCESS
            else:
                self.logger.error(f"  行为节点 ({self.name}): 探索失败 - {feedback.get('message', '')}")
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
            self.logger.error(f"  行为节点 ({self.name}): 探索异常 - {str(e)}")
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        """行为结束时的清理"""
        self._is_running = False
        self._exploration_generator = None
        self.logger.debug(f"  行为节点 ({self.name}): 探索结束，状态: {new_status}")


class PlanExplorationWaypointsBehaviour(py_trees.behaviour.Behaviour):
    """规划探索路径点的行为节点"""
    
    def __init__(self, name: str, robot_instance):
        super().__init__(name)
        self.robot = robot_instance
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key("polygon_coords", access=py_trees.common.Access.READ)
        self.blackboard.register_key("holes", access=py_trees.common.Access.READ)
        self.blackboard.register_key("lane_width", access=py_trees.common.Access.READ)
        self.blackboard.register_key("robot_radius", access=py_trees.common.Access.READ)
        self.blackboard.register_key("exploration_waypoints", access=py_trees.common.Access.WRITE)

    def setup(self):
        """初始化设置"""
        self.logger.debug(f"  行为节点 ({self.name}): 设置完成")
        return True

    def initialise(self):
        """行为开始时的初始化"""
        self.logger.debug(f"  行为节点 ({self.name}): 开始规划探索路径点")

    def update(self):
        """更新行为状态"""
        try:
            # 获取规划参数
            polygon_coords = getattr(self.blackboard, 'polygon_coords', None)
            holes = getattr(self.blackboard, 'holes', None)
            lane_width = getattr(self.blackboard, 'lane_width', 1.0)
            robot_radius = getattr(self.blackboard, 'robot_radius', 0.2)
            
            if polygon_coords is None:
                self.logger.error(f"  行为节点 ({self.name}): 未指定多边形坐标")
                return py_trees.common.Status.FAILURE
            
            from robot.skill.base.exploration.plan_exploration_waypoints import plan_exploration_waypoints_skill
            waypoints = plan_exploration_waypoints_skill(
                robot=self.robot,
                polygon_coords=polygon_coords,
                holes=holes,
                lane_width=lane_width,
                robot_radius=robot_radius
            )
            
            # 将路径点写入黑板
            self.blackboard.exploration_waypoints = waypoints
            
            self.logger.info(f"  行为节点 ({self.name}): 探索路径点规划完成，共{len(waypoints.poses)}个点")
            return py_trees.common.Status.SUCCESS
                
        except Exception as e:
            self.logger.error(f"  行为节点 ({self.name}): 路径点规划异常 - {str(e)}")
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        """行为结束时的清理"""
        self.logger.debug(f"  行为节点 ({self.name}): 路径点规划结束，状态: {new_status}")