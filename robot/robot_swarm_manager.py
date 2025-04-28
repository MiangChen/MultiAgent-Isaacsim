from typing import Dict, List, Type
import yaml


from map.map_grid_map import GridMap
from robot.robot_base import RobotBase
from robot.robot_cfg import RobotCfg


import numpy as np
from isaacsim.core.api.scenes import Scene


class RobotSwarmManager:
    """
    增强型机器人集群管理系统
    专门管理多个机器人的
    有几个要点, 一个是能知道目前所有的机器人类, jetbot/ g1 / go2 等等
    第二个是, 对于同品种的机器人, 需要单独配置他们的name, pos,避免重名或者碰撞
    第三个,对于不同的机器人, 最好都用一个变量来管理, 可能是字典, key为机器人type, value是机器人的实例化,
    第四个,需要能中途加入机器人和删除机器人(但是前期试过, 好像在加入机器人后, 必须要reset world, 那么世界也就完全重置了, 所以我们可以定一个机器人仓库, 比如已经有这么多机器人了, 我们现在要一些新的机器人, 那么仓库里的机器人会加入行动, 非常合理)
    """

    def __init__(self, scene: Scene, map_grid: GridMap = None):
        self.scene = scene  # 保留世界场景引用
        self.robot_warehouse: Dict[str, List[RobotBase]] = (
            {}
        )  # 激活的机器人 {type: [instances]}
        self.flag_active: Dict[str, List[int]] = (
            {}
        )  # 机器人激活状态的寄存器, 有id的就是激活的
        self.robot_active: Dict[str, List[RobotBase]] = {}
        self.robot_class = {  # 可扩展的机器人类注册
            # 'jetbot': Jetbot,
            # 'g1': G1,  # 可以后续添加
            # 'go2': Go2
        }
        self.robot_class_cfg = {}  # 对应的机器人
        self.map_grid = map_grid

    def register_robot_class(
        self,
        robot_class_name: str,
        robot_class: Type[RobotBase],
        robot_class_cfg: Type[RobotCfg],
    ) -> None:
        """注册新的机器人类型"""
        self.robot_warehouse[robot_class_name] = []
        self.robot_active[robot_class_name] = []
        self.flag_active[robot_class_name] = []
        self.robot_class[robot_class_name] = robot_class
        self.robot_class_cfg[robot_class_name] = robot_class_cfg

    def load_robot_swarm_cfg(
        self, robot_swarm_cfg_file: str = None, dict: Dict = None
    ) -> None:

        if robot_swarm_cfg_file is not None:
            # 读取 YAML 文件
            with open(robot_swarm_cfg_file, "r") as file:
                dict = yaml.safe_load(file)  # 返回字典
        if dict is None:
            print("No configuration file or dictionary found")
        for robot_class_name in dict.keys():
            for robot in dict[
                robot_class_name
            ]:  # 可能有多个机器人  这里可以优化一下 让yaml的格式就和robot cfg一样
                self.create_robot(
                    robot_class_name,
                    id=robot["id"],
                    position=robot["position"],
                    orientation=robot["orientation"],
                    robot_class_cfg=self.robot_class_cfg[robot_class_name],
                )

    def create_robot(
        self,
        robot_class_name: str = None,
        id: int = None,
        position=None,
        orientation=None,
        robot_class_cfg: Type[RobotCfg] = None,
    ):
        """创建新机器人并加入仓库"""

        if robot_class_name not in self.robot_class:
            raise ValueError(f"Unknown robot type: {robot_class_name}")

        # if not self._is_name_unique(robot_class_name, id):
        #     raise ValueError(f"Robot id '{id}' of robot type {robot_class_name} already exists")

        # 定义对应的机器人的位置和姿态, 以及编号
        cfg = robot_class_cfg(
            id=id,
            position=np.array(position),
            orientation=np.array(orientation),
        )

        # 实例化一个对应的机器人
        robot = self.robot_class[robot_class_name](
            cfg, scene=self.scene, map_grid=self.map_grid
        )
        self.robot_warehouse[robot_class_name].append(robot)
        return robot

    def activate_robot(
        self, flag_file_path: str = None, flag_dict: Dict = None
    ) -> None:
        # 两种寄存器配置模式, 一个是从文件读取, 适合初始化时后加载大量机器人, 另一个是通过dict来配置, 时候后续少量的处理;
        if flag_file_path is not None:
            with open(flag_file_path, "r") as file:
                flag_dict = yaml.safe_load(file)  # 返回字典

        if flag_dict is None:
            print("No flag file or dict found")

        for key in self.robot_class.keys():
            for robot in self.robot_warehouse[key]:
                if robot.config.type in flag_dict.keys():
                    if robot.config.id in flag_dict[key]:
                        robot.flag_active = True  # 机器人自身记录一份
                        self.robot_active[key].append(robot)
                        self.scene.add(robot.robot_entity)
                        pass
        return

    def deactivate_robot(self, name: str):
        """停用机器人并返回仓库"""
        for robot_type in list(self.active_robots.keys()):
            for i, robot in enumerate(self.active_robots[robot_type]):
                if robot.name_prefix == name:
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
            if hasattr(robot, "traj"):
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
                if robot.name_prefix == name:
                    return robot
        for robot in self.robot_warehouse:
            if robot.name == name:
                return robot
        return None

    def _is_name_unique(self, name: str):
        """检查名称是否唯一"""
        return self._find_robot(name) is None


if __name__ == "__main__":
    pass
