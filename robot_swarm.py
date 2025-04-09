from typing import Dict, List, Type
import numpy as np

from jetbot_config import Jetbot

class RobotSwarmManager:
    """
    增强型机器人集群管理系统
    专门管理多个机器人的
    有几个要点, 一个是能知道目前所有的机器人类, jetbot/ g1 / go2 等等
    第二个是, 对于同品种的机器人, 需要单独配置他们的name, pos,避免重名或者碰撞
    第三个,对于不同的机器人, 最好都用一个变量来管理, 可能是字典, key为机器人type, value是机器人的实例化,
    第四个,需要能中途加入机器人和删除机器人(但是前期试过, 好像在加入机器人后, 必须要reset world, 那么世界也就完全重置了, 所以我们可以定一个机器人仓库, 比如已经有这么多机器人了, 我们现在要一些新的机器人, 那么仓库里的机器人会加入行动, 非常合理)
    """

    def __init__(self, world_scene):
        self.world_scene = world_scene  # 保留世界场景引用
        self.active_robots: Dict[str, List[Jetbot]] = {}  # 激活的机器人 {type: [instances]}
        self.robot_warehouse: List[Jetbot] = []  # 待激活机器人仓库
        self.robot_classes = {  # 可扩展的机器人类注册
            'jetbot': Jetbot,
            # 'g1': G1,  # 可以后续添加
            # 'go2': Go2
        }

    def register_robot_class(self, name: str, robot_class: Type):
        """注册新的机器人类型"""
        self.robot_classes[name] = robot_class

    def create_robot(self, robot_type: str, name: str, position, orientation, cfg_class=JetbotRobotCfg):
        """创建新机器人并加入仓库"""
        if robot_type not in self.robot_classes:
            raise ValueError(f"Unknown robot type: {robot_type}")

        if not self._is_name_unique(name):
            raise ValueError(f"Robot name '{name}' already exists")

        cfg = cfg_class(
            position=np.array(position),
            orientation=np.array(orientation)
        )

        robot = self.robot_classes[robot_type](
            cfg,
            scene=self.world_scene
        )
        robot.name = name  # 确保每个机器人有唯一标识
        robot.is_active = False
        self.robot_warehouse.append(robot)
        return robot

    def activate_robot(self, name: str):
        """从仓库激活机器人"""
        for i, robot in enumerate(self.robot_warehouse):
            if robot.name == name:
                robot.is_active = True
                robot_type = robot.__class__.__name__.lower()

                if robot_type not in self.active_robots:
                    self.active_robots[robot_type] = []

                self.active_robots[robot_type].append(robot)
                self.robot_warehouse.pop(i)

                # 初始化轨迹记录
                if not hasattr(robot, 'traj'):
                    robot.traj = TrajectoryRecorder()
                return True
        return False

    def deactivate_robot(self, name: str):
        """停用机器人并返回仓库"""
        for robot_type in list(self.active_robots.keys()):
            for i, robot in enumerate(self.active_robots[robot_type]):
                if robot.name == name:
                    robot.is_active = False
                    self.robot_warehouse.append(robot)
                    self.active_robots[robot_type].pop(i)

                    # 清理空类型
                    if not self.active_robots[robot_type]:
                        del self.active_robots[robot_type]
                    return True
        return False

    def move_robot_along_path(self, name: str, path, reset_flag=True):
        """控制指定机器人沿路径移动"""
        robot = self._find_robot(name)
        if robot:
            robot.move_along_path(path, reset_flag)
            if hasattr(robot, 'traj'):
                robot.traj.add_trajectory(robot.get_world_pose()[0])
            return True
        return False

    def get_robot_position(self, name: str):
        """获取机器人当前位置"""
        robot = self._find_robot(name)
        if robot:
            return robot.get_world_pose()[0][:2]
        return None

    def _find_robot(self, name: str):
        """在所有机器人中查找"""
        for robots in self.active_robots.values():
            for robot in robots:
                if robot.name == name:
                    return robot
        for robot in self.robot_warehouse:
            if robot.name == name:
                return robot
        return None

    def _is_name_unique(self, name: str):
        """检查名称是否唯一"""
        return self._find_robot(name) is None


class TrajectoryRecorder:
    """机器人轨迹记录器"""

    def __init__(self):
        self.history = []

    def add_trajectory(self, position):
        self.history.append(position.copy())

    def get_full_trajectory(self):
        return np.array(self.history)
