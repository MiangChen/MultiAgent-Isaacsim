# =============================================================================
# Containers Module - Dependency Injection Container Configuration
# =============================================================================
#
# This module defines the dependency injection container using the
# dependency-injector library, managing all service dependencies and
# their lifecycle within the application.
#
# =============================================================================

# Standard library imports
import asyncio
import sys

# Third-party library imports
from dependency_injector import containers, providers

# Local project imports
from config.config_manager import config_manager
from log.log_manager import LogManager


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

    # 使用simulation层的Server和World（延迟import避免在simulation_app启动前加载Isaac Sim模块）
    server = providers.Singleton(
        lambda: _import_and_create_server(),
    )
    
    world = providers.Singleton(
        lambda server, cfg: server.get_world(
            physics_dt=cfg["world"]["physics_dt"],
            rendering_dt=cfg["world"]["rendering_dt"],
            stage_units_in_meters=cfg["world"]["stage_units_in_meters"],
            sim_params=cfg["world"]["sim_params"],
            backend=cfg["world"]["backend"]
        ),
        server=server,
        cfg=config
    )

    grid_map = providers.Singleton(
        lambda cfg: _import_and_create_grid_map(cfg),
        cfg=config
    )

    ros_manager = providers.Singleton(
        lambda loop, ros_config: _import_and_create_ros_manager(loop, ros_config),
        loop=loop,
        ros_config=config.provided["ros"],
    )

    scene_manager = providers.Singleton(
        lambda world: _import_and_create_scene_manager(world),
        world=world
    )
    
    semantic_map = providers.Singleton(
        lambda: _import_and_create_semantic_map()
    )
    
    viewport_manager = providers.Singleton(
        lambda: _import_and_create_viewport_manager()
    )

    swarm_manager = providers.Singleton(
        lambda map_semantic, scene_manager: _import_and_create_swarm_manager(map_semantic, scene_manager),
        map_semantic=semantic_map,
        scene_manager=scene_manager,
    )
    
    # 配置World的组件（整合原Env的功能）
    world_configured = providers.Singleton(
        lambda w, sm, swm, gm: _configure_world(w, sm, swm, gm),
        w=world,
        sm=scene_manager,
        swm=swarm_manager,
        gm=grid_map
    )


def _import_and_create_server():
    from simulation.server import Server
    return Server()

def _import_and_create_grid_map(cfg):
    from map.map_grid_map import GridMap
    return GridMap(
        cell_size=cfg["map"]["cell_size"],
        start_point=cfg["map"]["start_point"],
        min_bound=cfg["map"]["min_bound"],
        max_bound=cfg["map"]["max_bound"],
        occupied_value=cfg["map"]["occupied_cell"],
        free_value=cfg["map"]["free_cell"],
        unknown_value=cfg["map"]["unknown_cell"],
    )

def _import_and_create_ros_manager(loop, ros_config):
    from ros.ros_manager import RosManager
    return RosManager(loop=loop, config=ros_config)

def _import_and_create_scene_manager(world):
    from scene.scene_manager import SceneManager
    return SceneManager(world=world)

def _import_and_create_semantic_map():
    from map.map_semantic_map import MapSemantic
    return MapSemantic()

def _import_and_create_viewport_manager():
    from ui.viewport_manager import ViewportManager
    return ViewportManager()

def _import_and_create_swarm_manager(map_semantic, scene_manager):
    from robot.swarm_manager import SwarmManager
    return SwarmManager(map_semantic=map_semantic, scene_manager=scene_manager)

def _configure_world(world, scene_manager, swarm_manager, grid_map):
    """配置World的组件"""
    world.set_scene_manager(scene_manager)
    world.set_swarm_manager(swarm_manager)
    world.set_grid_map(grid_map)
    return world


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
