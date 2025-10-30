# =============================================================================
# Swarm Manager Module - Multi-Robot Swarm Management
# =============================================================================
#
# This module provides enhanced robot swarm management system for managing
# multiple robots of different types, handling their configurations, positions,
# and coordination within the simulation environment.
#
# =============================================================================

# Standard library imports
import inspect
from typing import Dict, List, Type

# Third-party library imports
import yaml

# Local project imports
from log.log_manager import LogManager
from map.map_semantic_map import MapSemantic
from physics_engine.isaacsim_utils import Scene
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
        map_semantic: MapSemantic = None,
        scene_manager: SceneManager = None,
    ):
        self.scene: Scene = None
        self.scene_manager = scene_manager
        self.map_semantic = map_semantic
        self.robot_warehouse: Dict[str, List[Robot]] = {
            # 'jetbot': Jetbot cls instance,
        }
        self.robot_class = {  # 可扩展的机器人类注册
            # 'jetbot': Jetbot,
            # 'g1': G1,
            # 'go2': Go2
        }

    def register_robot_class(
        self,
        robot_class_name: str,
        robot_class: Type[Robot],
    ) -> None:
        """注册新的机器人类型"""
        self.robot_warehouse[robot_class_name] = []
        self.robot_class[robot_class_name] = robot_class

    async def initialize_async(
        self,
        scene: Scene,
        robot_swarm_cfg_path: str = None,
    ) -> None:
        """
        Complete async initialization of the swarm manager.
        """
        # Set scene reference
        self.scene = scene

        # Load robot swarm configuration if path provided
        if robot_swarm_cfg_path is not None:
            await self.load_robot_swarm_cfg(robot_swarm_cfg_path)

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
                        scene=self.scene,
                        scene_manager=self.scene_manager,
                    )
                else:
                    # 如果不是，使用传统的同步 __init__ 方法
                    robot = robot_cls(
                        cfg_robot=cfg_robot,
                        scene=self.scene,
                        scene_manager=self.scene_manager,
                    )

                self.robot_warehouse[robot_class_name].append(robot)
                self.scene.add(robot.body.robot_articulation)
                self.map_semantic.dict_map_semantic[robot.cfg_robot.namespace] = (
                    robot.cfg_robot.path_prim_robot
                )
                self.map_semantic.add_semantic(
                    prim_path=robot.cfg_robot.path_prim_robot, semantic_label="robot"
                )
