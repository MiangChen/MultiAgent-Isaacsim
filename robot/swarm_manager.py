import inspect
from typing import Dict, List, Type
import yaml

from isaacsim.core.api.scenes import Scene

from log.log_manager import LogManager
from map.map_grid_map import GridMap
from map.map_semantic_map import MapSemantic
from robot.sensor.camera import CfgCamera, CfgCameraThird
from robot.robot import Robot
from robot.cfg import CfgRobot
from ros.ros_manager import RosManager, logger
from scene.scene_manager import SceneManager

logger = LogManager.get_logger(__name__)


class SwarmManager:
    """
    增强型机器人集群管理系统
    专门管理多个机器人的
    有几个要点, 一个是能知道目前所有的机器人类, jetbot/ g1 / go2 等等
    第二个是, 对于同品种的机器人, 需要单独配置他们的name, pos,避免重名或者碰撞
    第三个,对于不同的机器人, 最好都用一个变量来管理, 可能是字典, key为机器人type, value是机器人的实例化,
    第四个,需要能中途加入机器人和删除机器人(但是前期试过, 好像在加入机器人后, 必须要reset world, 那么世界也就完全重置了, 所以我们可以定一个机器人仓库, 比如已经有这么多机器人了, 我们现在要一些新的机器人, 那么仓库里的机器人会加入行动, 非常合理)
    """

    def __init__(
        self,
        map_grid: GridMap = None,
        map_semantic: MapSemantic = None,
        ros_manager: RosManager = None,
        scene_manager: SceneManager = None,
    ):
        self.scene: Scene = None
        self.scene_manager = scene_manager
        self.robot_warehouse: Dict[str, List[Robot]] = {}
        self.flag_active: Dict[str, List[int]] = {}
        self.robot_active: Dict[str, List[Robot]] = {
            # 'jetbot': Jetbot cls instance,
        }
        self.robot_class = {  # 可扩展的机器人类注册
            # 'jetbot': Jetbot,
            # 'g1': G1,
            # 'go2': Go2
        }
        self.map_grid = map_grid
        self.map_semantic = map_semantic
        self.ros_manager = ros_manager

    def register_robot_class(
        self,
        robot_class_name: str,
        robot_class: Type[Robot],
    ) -> None:
        """注册新的机器人类型"""
        self.robot_warehouse[robot_class_name] = []
        self.robot_active[robot_class_name] = []
        self.flag_active[robot_class_name] = []
        self.robot_class[robot_class_name] = robot_class

    async def initialize_async(
        self,
        scene: Scene,
        robot_swarm_cfg_path: str = None,
        robot_active_flag_path: str = None,
    ) -> None:
        """
        Complete async initialization of the swarm manager.
        """
        # Set scene reference
        self.scene = scene

        # Load robot swarm configuration if path provided
        if robot_swarm_cfg_path is not None:
            await self.load_robot_swarm_cfg(robot_swarm_cfg_path)

        # Activate robots if flag path provided
        if robot_active_flag_path is not None:
            self.activate_robot(robot_active_flag_path)

    async def load_robot_swarm_cfg(self, robot_swarm_cfg_file: str = None) -> None:
        """异步加载并创建配置文件中定义的所有机器人。"""
        with open(robot_swarm_cfg_file, "r") as file:
            dict = yaml.safe_load(file)

        for robot_class_name in dict.keys():
            for cfg_robot in dict[robot_class_name]:
                if robot_class_name not in self.robot_class:
                    raise ValueError(f"Unknown robot type: {robot_class_name}")

                # 选择同步或异步创建
                robot_cls = self.robot_class[robot_class_name]

                # 检查机器人class是否有 'create' 方法，并且它是一个异步函数
                if hasattr(robot_cls, "create") and inspect.iscoroutinefunction(
                    robot_cls.create
                ):
                    # 如果是，使用 await 调用异步工厂 create 方法
                    robot = await robot_cls.create(
                        cfg_robot=cfg_robot,
                        map_grid=self.map_grid,
                        scene=self.scene,
                        scene_manager=self.scene_manager,
                    )
                else:
                    # 如果不是，使用传统的同步 __init__ 方法
                    robot = robot_cls(
                        cfg_robot=cfg_robot,
                        scene=self.scene,
                        map_grid=self.map_grid,
                        scene_manager=self.scene_manager,
                    )

                self.robot_warehouse[robot_class_name].append(robot)
                self.map_semantic.map_semantic[
                    robot.cfg_robot.name
                ] = robot.cfg_robot.path_prim_robot

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
                if robot.cfg_robot.type in flag_dict.keys():
                    if robot.cfg_robot.id in flag_dict[key]:
                        robot.flag_active = True  # 机器人自身记录一份
                        self.robot_active[key].append(robot)
                        self.scene.add(robot.body.robot_articulation)
        return

    def deactivate_robot(self, name: str):
        """停用机器人并返回仓库"""
        for robot_type in list(self.robot_active.keys()):
            for i, robot in enumerate(self.robot_active[robot_type]):
                if robot.cfg_robot.type == name:
                    robot.is_active = False
                    # self.robot_warehouse.append(robot)
                    self.robot_active[robot_type].pop(i)

                    # 清理空类型
                    if not self.robot_active[robot_type]:
                        del self.robot_active[robot_type]
                    return True
        return False

    def _find_robot(self, name: str):
        """在所有机器人中查找"""
        for robots in self.robot_active.values():
            for robot in robots:
                if robot.cfg_robot.type == name:
                    return robot
        for robot in self.robot_warehouse.keys():
            if robot.cfg_robot.name == name:
                return robot
        return None

    def _is_name_unique(self, name: str):
        """检查名称是否唯一"""
        return self._find_robot(name) is None


if __name__ == "__main__":
    pass
