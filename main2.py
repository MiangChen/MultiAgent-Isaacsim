import os
import sys
sys.path.insert(0, os.path.join("/home/ubuntu/PycharmProjects/isaacsim-gsi/src/", 'gsi2isaacsim'))

import argparse
from physics_engine.isaacsim_simulation_app import initialize_simulation_app_from_yaml


def pre_initialize():
    """
    执行最小化的启动，只为了创建 SimulationApp 实例。
    """

    parser = argparse.ArgumentParser(
        add_help=False
    )  # add_help=False 避免与后续的解析器冲突
    parser.add_argument("--config", type=str, default="./config/config_parameter.yaml")

    args, unknown = parser.parse_known_args()

    simulation_app = initialize_simulation_app_from_yaml(args.config)

    return simulation_app


simulation_app = pre_initialize()

###################################################################################################################
import asyncio

# Third-party imports
from dependency_injector.wiring import inject, Provide  # Dependency injection imports

from isaacsim.core.api import World

# Local imports
from config.config_manager import config_manager
from containers import AppContainer, get_container, reset_container
from environment.env import Env
from log.log_manager import LogManager
from map.map_grid_map import GridMap
from map.map_semantic_map import MapSemantic
from robot.robot_cf2x import RobotCf2x, RobotCfgCf2x
from robot.robot_h1 import RobotH1, RobotCfgH1
from robot.robot_jetbot import RobotCfgJetbot, RobotJetbot
from robot.swarm_manager import SwarmManager
from scene.scene_manager import SceneManager

# Log startup information with argument summary
logger = LogManager.get_logger(__name__)
logger.info("Isaac Sim WebManager starting...")

WORLD_USD_PATH = config_manager.get("world_usd_path")
PROJECT_ROOT = config_manager.get("project_root")


@inject
def setup_simulation(
        swarm_manager: SwarmManager = Provide[AppContainer.swarm_manager],
        env: Env = Provide[AppContainer.env],
        world: World = Provide[AppContainer.world],
) -> None:
    """
    Setup simulation environment with injected dependencies.
    """
    # Register robot classes to swarm manager
    swarm_manager.register_robot_class(
        robot_class_name="jetbot",
        robot_class=RobotJetbot,
        robot_class_cfg=RobotCfgJetbot,
    )
    swarm_manager.register_robot_class(
        robot_class_name="h1", robot_class=RobotH1, robot_class_cfg=RobotCfgH1
    )
    swarm_manager.register_robot_class(
        robot_class_name="cf2x", robot_class=RobotCf2x, robot_class_cfg=RobotCfgCf2x
    )

    # Initialize environment and swarm manager
    # Since Isaac Sim runs its own event loop, we need to schedule async tasks properly

    # Create initialization tasks
    async def init_env_and_swarm():
        await env.initialize_async()
        await swarm_manager.initialize_async(
            scene=world.scene,
            robot_swarm_cfg_path=f"{PROJECT_ROOT}/config/robot_swarm_cfg.yaml",
            robot_active_flag_path=f"{PROJECT_ROOT}/config/robot_swarm_active_flag.yaml",
        )

    # Schedule the initialization in Isaac Sim's event loop

    loop = asyncio.get_event_loop()
    # Create a task but don't wait for it to complete immediately
    init_task = loop.create_task(init_env_and_swarm())

    # Wait for initialization to complete before proceeding
    # We'll do this by running a few simulation steps to let the async tasks execute
    print("Waiting for async initialization to complete...")
    for _ in range(10):  # Give some time for async initialization
        simulation_app.update()
        if init_task.done():
            break

    # Check if initialization completed successfully
    if init_task.done():
        if init_task.exception():
            raise init_task.exception()
        print("Async initialization completed successfully")
    else:
        print("Warning: Async initialization may still be running")


def create_car_objects(scene_manager: SceneManager) -> list:
    """
    Create car objects in the scene with semantic labels using injected dependencies.

    Args:
        scene_manager: Injected scene manager instance for creating objects

    Returns:
        list: List of created prim paths
    """
    scale = [2, 5, 1.0]
    cubes_config = {
        "car0": {
            "shape_type": "cuboid",
            "prim_path": "/World/car0",
            "size": scale,
            "scene_name": "car0",
            "position": [11.6, 3.5, 0],
            "color": [255, 255, 255],
        },
        "car1": {
            "shape_type": "cuboid",
            "prim_path": "/World/car1",
            "size": scale,
            "scene_name": "car1",
            "position": [0.3, 3.5, 0],
            "color": [255, 255, 255],
        },
        "car2": {
            "shape_type": "cuboid",
            "prim_path": "/World/car2",
            "size": scale,
            "scene_name": "car2",
            "position": [-13.2, 3.5, 0],
            "color": [255, 255, 255],
        },
        "car3": {
            "shape_type": "cuboid",
            "prim_path": "/World/car3",
            "scene_name": "car3",
            "size": scale,
            "position": [-7.1, 10, 0],
            "color": [255, 255, 255],
        },
        "car4": {
            "shape_type": "cuboid",
            "prim_path": "/World/car4",
            "scene_name": "car4",
            "size": scale,
            "position": [-0.9, 30, 0],
            "orientation": [0.707, 0, 0, 0.707],
            "color": [255, 255, 255],
            "make_dynamic": False,
        },
    }

    created_prim_paths = []
    print(
        "All semantics in scene:",
        scene_manager.count_semantics_in_scene().get("result"),
    )

    for cube_name, config in cubes_config.items():
        print(f"--- Processing: {cube_name} ---")

        # Create shape using unpacking
        creation_result = scene_manager.create_shape(**config)

        # Check the result
        if creation_result.get("status") == "success":
            prim_path = creation_result.get("result")
            print(f"  Successfully created prim at: {prim_path}")
            created_prim_paths.append(prim_path)

            # Add semantic label
            semantic_result = scene_manager.add_semantic(
                prim_path=prim_path, semantic_label="car"
            )

            if semantic_result.get("status") == "success":
                print(f"  Successfully applied semantic label 'car' to {prim_path}")
            else:
                print(
                    f"  [ERROR] Failed to apply semantic label: {semantic_result.get('message')}"
                )
        else:
            print(
                f"  [ERROR] Failed to create shape '{cube_name}': {creation_result.get('message')}"
            )

    return created_prim_paths


def process_semantic_detection(semantic_camera, map_semantic: MapSemantic) -> None:
    """
    Process semantic detection and car pose extraction using injected dependencies.

    Args:
        semantic_camera: Semantic camera instance
        map_semantic: Injected semantic map instance
    """
    try:
        current_frame = semantic_camera.get_current_frame()
        if current_frame and "bounding_box_2d_loose" in current_frame:
            result = current_frame["bounding_box_2d_loose"]
            print("get bounding box 2d loose", result)
            if result:
                car_prim, car_pose = map_semantic.get_prim_and_pose_by_semantic(
                    result, "car"
                )
                if car_prim is not None and car_pose is not None:
                    print("get car prim and pose\n", car_prim, "\n", car_pose)
                else:
                    print("No car detected in current frame")
            else:
                print("No bounding box data available")
        else:
            print("No frame data or bounding box key available")
    except Exception as e:
        print(f"Error getting semantic camera data: {e}")


def main():
    print("\n\n\n\ninto the main\n\n\n\n")
    # Setup dependency injection container
    reset_container()
    container = get_container()

    # Wire the container to this module for @inject decorators in skill functions
    container.wire(modules=[__name__])

    # Get services from container
    config_manager = container.config_manager()

    log_manager = container.log_manager()
    ros_manager = container.ros_manager()
    swarm_manager = container.swarm_manager()
    scene_manager = container.scene_manager()
    grid_map = container.grid_map()
    semantic_map = container.semantic_map()
    skill_manager = container.skill_manager()
    viewport_manager = container.viewport_manager()
    world = container.world()
    env = container.env()
    env.simulation_app = simulation_app
    setup_simulation()
    ros_manager.start()

    # Load scene
    scene_manager.load_scene(usd_path=WORLD_USD_PATH, prim_path_root="/World/Scene")
    # scene_manager.enable_raycasting_for_prim(prim_path="/World/Scene")

    # Create car objects using scene manager
    created_prim_paths = create_car_objects(scene_manager)
    print("All prims with 'car' label:", created_prim_paths)
    print(scene_manager.count_semantics_in_scene().get("result"))

    # Reset environment to ensure all objects are properly initialized
    env.reset()

    # Add physics callbacks for active robots
    for robot_class in swarm_manager.robot_class:
        for i, robot in enumerate(swarm_manager.robot_active[robot_class]):
            callback_name = f"physics_step_{robot_class}_{i}"
            env.world.add_physics_callback(
                callback_name, callback_fn=robot.on_physics_step
            )

    # Create and initialize semantic camera
    result = scene_manager.add_camera(
        position=[1, 4, 2],
        quat=scene_manager.euler_to_quaternion(roll=90),
        prim_path="/World/semantic_camera",
    )
    semantic_camera = result.get("result").get("camera_instance")
    semantic_camera_prim_path = result.get("result").get("prim_path")
    semantic_camera.initialize()

    # Wait for camera and rendering pipeline to fully initialize
    for _ in range(10):
        env.step(action=None)

    # Enable bounding box detection after initialization period
    semantic_camera.add_bounding_box_2d_loose_to_frame()

    # Switch viewport to semantic camera
    from omni.kit.viewport.utility import get_viewport_from_window_name

    viewport_manager.register_viewport(
        name="Viewport", viewport_obj=get_viewport_from_window_name("Viewport")
    )  # isaacsim default viewport
    viewport_manager.change_viewport(
        camera_prim_path=semantic_camera_prim_path, viewport_name="Viewport"
    )

    # Build grid map for planning
    grid_map.generate_grid_map("2d")

    count = 0
    logger.info("Starting main simulation loop...")

    robot_prim_path = "/World/robot/jetbot/jetbot/jetbot_0/chassis"
    object_prim_path = "/World/object"
    object = {
        "shape_type": "cuboid",
        "prim_path": object_prim_path,
        # "scene_name": "object",
        "name": "object",
        "size": [0.1, 0.1, 0.1],
        # "position": semantic_map.map_semantic['place4'],
        "position": [5, 6.5, 0.1],
        "orientation": [0.707, 0, 0, 0.707],
        "color": [255, 255, 255],
        "mass": 0.1,
    }
    scene_manager.create_shape_unified(**object)

    skill_manager._skill_navigate_to(
        rc="jetbot",
        rid=0,
        params={"goal": "place4"},
    )
    flag = 0
    # Main simulation loop
    while simulation_app.is_running():

        # World step
        env.step(action=None)

        if count % 60 == 0 and count != 0:
            skill_manager._skill_take_photo(
                rc="jetbot",
                rid=0,
                params={"file_path": "/home/ubuntu/test.jpg"}
            )

        if flag == 0:
            result = skill_manager._skill_pick_up(
                rc="jetbot",
                rid=0,
                params={
                    "object_prim_path": object_prim_path,
                    "robot_prim_path": robot_prim_path,
                },
            )
            if result != None and result.get("status") == "success":
                flag = 1

        elif count > 500 and flag == 1:
            result = skill_manager._skill_put_down(
                rc="jetbot",
                rid=0,
                params={
                    "object_prim_path": object_prim_path,
                    "robot_prim_path": robot_prim_path,
                },
            )
            flag = 2

        # if count > 400:
        # _skill_navigate_to(
        #     swarm_manager,
        #     rc="jetbot",
        #     rid=0,
        #     params={"goal": "place4"},
        #     semantic_map=semantic_map,
        # )
        # _skill_navigate_to(
        #     swarm_manager,
        #     rc="jetbot",
        #     rid=1,
        #     params={"goal": "place1"},
        #     semantic_map=semantic_map,
        # )
        # _skill_navigate_to(
        #     swarm_manager,
        #     rc="jetbot",
        #     rid=2,
        #     params={"goal": "place2"},
        #     semantic_map=semantic_map,
        # )
        # _skill_navigate_to(
        #     swarm_manager,
        #     rc="jetbot",
        #     rid=3,
        #     params={"goal": "place3"},
        #     semantic_map=semantic_map,
        # )
        # _skill_navigate_to(
        #     swarm_manager,
        #     rc="h1",
        #     rid=0,
        #     params={"goal": "place3"},
        #     semantic_map=semantic_map,
        # )
        # process_semantic_detection(semantic_camera, semantic_map)

        # Process ROS skills if ROS is enabled
        if config_manager.get("ros"):
            skill_manager.process_ros_skills()

        count += 1

    ros_manager.stop()

    container.unwire()

    logger.info("--- Simulation finished. Manually closing application. ---")
    if simulation_app:
        simulation_app.__exit__(None, None, None)


if __name__ == "__main__":
    # 直接调用同步 main 函数
    main()
