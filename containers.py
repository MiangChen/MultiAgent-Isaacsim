import asyncio
import sys
from dependency_injector import containers, providers

from isaacsim.core.api import World
from config.config_manager import config_manager
from environment.env import Env
from log.log_manager import LogManager
from map.map_semantic_map import MapSemantic
from map.map_grid_map import GridMap
from robot.swarm_manager import SwarmManager
from ros.ros_manager import RosManager
from scene.scene_manager import SceneManager
from ui.viewport_manager import ViewportManager


class AppContainer(containers.DeclarativeContainer):
    """
    最终版的依赖注入容器。
    ConfigManager 是所有服务的配置源头。
    """

    config_manager = providers.Object(config_manager)
    config = providers.Factory(lambda manager: manager.config, manager=config_manager)

    log_manager = providers.Singleton(
        LogManager,
        log_level=config.provided["logging"]["level"],
        log_file=config.provided["logging"]["file"],
    )
    loop = providers.Singleton(asyncio.get_event_loop)

    world = providers.Singleton(
        World,
        physics_dt=config.provided["world"]["physics_dt"],
        rendering_dt=config.provided["world"]["rendering_dt"],
        stage_units_in_meters=config.provided["world"]["stage_units_in_meters"],
        backend="torch",
    )

    grid_map = providers.Singleton(
        GridMap,
        cell_size=config.provided["map"]["cell_size"],
        start_point=config.provided["map"]["start_point"],
        min_bounds=config.provided["map"]["min_bounds"],
        max_bounds=config.provided["map"]["max_bounds"],
        occupied_cell=config.provided["map"]["occupied_cell"],
        empty_cell=config.provided["map"]["empty_cell"],
        invisible_cell=config.provided["map"]["invisible_cell"],
    )

    ros_manager = providers.Singleton(
        RosManager,
        action_mode=config.provided["action_mode"],
    )

    scene_manager = providers.Singleton(SceneManager)
    semantic_map = providers.Singleton(MapSemantic)
    viewport_manager = providers.Singleton(ViewportManager)

    swarm_manager = providers.Singleton(
        SwarmManager,
        map_grid=grid_map,
        map_semantic=semantic_map,
        ros_manager=ros_manager,
        scene_manager=scene_manager,
    )

    # skill_manager = providers.Singleton(
    #     SkillManager,
    #     semantic_map=semantic_map,
    #     swarm_manager=swarm_manager,
    # )

    env = providers.Factory(
        Env,
        simulation_app=providers.Object(None),  # 保持不变，由 main.py 运行时提供
        world=world,
        scene_manager=scene_manager,
        swarm_manager=swarm_manager,
        grid_map=grid_map,
    )


# 全局容器实例
_container: AppContainer = None


def get_container() -> AppContainer:
    """
    获取全局容器实例。
    这个函数现在变得极其简单，因为它不再负责加载配置。
    """
    global _container
    if _container is None:
        _container = AppContainer()

        # 自动绑定需要 @inject 装饰器的模块
        modules_to_wire = [__name__, "main", "skill.skill"]  # 添加所有需要注入的模块
        modules_in_sys = [sys.modules.get(m) for m in modules_to_wire]
        _container.wire(modules=[m for m in modules_in_sys if m is not None])

    return _container


def reset_container() -> None:
    """
    重置全局容器实例 (用于测试)。
    """
    global _container
    if _container is not None:
        try:
            _container.unwire()
        except Exception:
            pass
    _container = None
