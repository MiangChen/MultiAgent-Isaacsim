# =============================================================================
# Main Example - Isaac Sim Robotics Simulation Demo
# =============================================================================
#
# This is the main example demonstrating the complete robotics simulation
# pipeline using Isaac Sim, including robot swarm management, navigation,
# semantic mapping, and ROS2 integration.
#
# =============================================================================

try:
    import pydevd_pycharm

    pydevd_pycharm.settrace(
        "localhost", port=12345, stdout_to_server=True, stderr_to_server=True
    )
except Exception as e:
    print(f"no pydevd found: {repr(e)}")


def create_car_objects(scene_manager, map_semantic, logger) -> list:
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
            "scale": scale,
            "name": "car0",
            "position": [11.6, 3.5, 0],
            "color": [255, 255, 255],
            "entity_type": "visual",
        },
        "car1": {
            "shape_type": "cuboid",
            "prim_path": "/World/car1",
            "scale": scale,
            "name": "car1",
            "position": [0.3, 3.5, 0],
            "color": [255, 255, 255],
            "entity_type": "visual",
        },
        "car2": {
            "shape_type": "cuboid",
            "prim_path": "/World/car2",
            "scale": scale,
            "name": "car2",
            "position": [-13.2, 3.5, 0],
            "color": [255, 255, 255],
            "entity_type": "visual",
        },
        "car3": {
            "shape_type": "cuboid",
            "prim_path": "/World/car3",
            "name": "car3",
            "scale": scale,
            "position": [-7.1, 10, 0],
            "color": [255, 255, 255],
            "entity_type": "visual",
        },
        "car4": {
            "shape_type": "cuboid",
            "prim_path": "/World/car4",
            "name": "car4",
            "scale": scale,
            "position": [-0.9, 30, 0],
            "orientation": [0.707, 0, 0, 0.707],
            "color": [255, 255, 255],
            "entity_type": "visual",
        },
    }

    created_prim_paths = []
    logger.info(
        f"All semantics in scene:{map_semantic.count_semantics_in_scene().get('result')}"
    )

    for cube_name, config in cubes_config.items():
        creation_result = scene_manager.create_shape_unified(**config)

        if creation_result.get("status") == "success":
            prim_path = creation_result.get("prim_path")
            created_prim_paths.append(prim_path)

            # Add semantic label
            semantic_result = map_semantic.add_semantic(
                prim_path=prim_path, semantic_label="car"
            )
            logger.info(semantic_result)

    return created_prim_paths


def process_semantic_detection(
        semantic_camera, map_semantic, target_semantic_class: str
) -> None:
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
                prim_target, target_pose = map_semantic.get_prim_and_pose_by_semantic(
                    result,
                    target_semantic_class=target_semantic_class,
                )
                if prim_target is not None and target_pose is not None:
                    print("get car prim and pose\n", prim_target, "\n", target_pose)
                else:
                    print("No car detected in current frame")
            else:
                print("No bounding box data available")
        else:
            print("No frame data or bounding box key available")
    except Exception as e:
        print(f"Error getting semantic camera data: {repr(e)}")


def main():
    print("\n\n\n\ninto the main\n\n\n\n")

    # Import after simulation_app is created by Server
    from containers import get_container, reset_container
    from config.config_manager import config_manager
    from log.log_manager import LogManager
    from utils import euler_to_quat
    import rclpy

    # Initialize ROS2
    rclpy.init(args=None)
    logger = LogManager.get_logger(__name__)

    WORLD_USD_PATH = config_manager.get("world_usd_path")
    PROJECT_ROOT = config_manager.get("project_root")

    # Setup dependency injection container
    reset_container()
    container = get_container()

    # Wire the container to this module for @inject decorators in skill functions
    container.wire(modules=[__name__])

    # Get services from container
    config_manager = container.config_manager()
    log_manager = container.log_manager()
    loop = container.loop()
    server = container.server()
    ros_manager = container.ros_manager()
    swarm_manager = container.swarm_manager()
    scene_manager = container.scene_manager()
    grid_map = container.grid_map()
    semantic_map = container.semantic_map()
    # skill_manager = container.skill_manager()
    viewport_manager = container.viewport_manager()

    # 使用simulation层的World（已整合Env功能）
    world = container.world_configured()
    simulation_app = server.get_simulation_app()

    # Register robot classes - CARLA style: direct operations in main
    from robot.robot_drone_cf2x import RobotCf2x
    from robot.robot_h1 import RobotH1
    from robot.robot_jetbot import RobotJetbot
    from robot.target import Target

    swarm_manager.register_robot_class("jetbot", RobotJetbot)
    swarm_manager.register_robot_class("h1", RobotH1)
    swarm_manager.register_robot_class("cf2x", RobotCf2x)
    swarm_manager.register_robot_class("target", Target)

    # Load robot swarm from config - CARLA style: synchronous
    logger.info("Loading robot swarm...")
    
    # 临时方案：使用 run_until_complete 包装异步方法
    # TODO: 将 swarm_manager.initialize_async 改为同步方法以符合 CARLA 风格
    loop.run_until_complete(
        swarm_manager.initialize_async(
            scene=world.scene,
            robot_swarm_cfg_path=f"{PROJECT_ROOT}/config/robot_swarm_cfg.yaml",
        )
    )
    
    logger.info("Robot swarm loaded")

    ros_manager.start()

    # Load scene
    scene_manager.load_scene(usd_path=WORLD_USD_PATH, prim_path_root="/World/Scene")
    # scene_manager.enable_raycasting_for_prim(prim_path="/World/Scene")

    # Create car objects using scene manager
    # created_prim_paths = create_car_objects(scene_manager, semantic_map)
    # print("All prims with 'car' label:", created_prim_paths)
    # print(scene_manager.count_semantics_in_scene().get("result"))

    world.reset()
    world.initialize_robots()
    world.initialize_map()

    # Add physics callbacks for active robots
    for robot_class in swarm_manager.robot_class:
        for i, robot in enumerate(swarm_manager.robot_warehouse[robot_class]):
            callback_name = f"physics_step_{robot_class}_{i}"
            world.get_isaac_world().add_physics_callback(
                callback_name, callback_fn=robot.on_physics_step
            )

    # Create and initialize semantic camera
    create_car_objects(scene_manager, semantic_map, logger)
    result = scene_manager.add_camera(
        translation=[1, 4, 2], orientation=euler_to_quat(roll=90)
    )
    semantic_camera = result.get("result").get("camera_instance")
    semantic_camera_prim_path = result.get("result").get("prim_path")
    semantic_camera.initialize()

    # Wait for camera and rendering pipeline to fully initialize
    for _ in range(10):
        world.tick()

    # Enable bounding box detection after initialization period
    semantic_camera.add_bounding_box_2d_loose_to_frame()

    # Switch viewport to semantic camera
    from physics_engine.omni_utils import get_viewport_from_window_name

    viewport_manager.register_viewport(
        name="Viewport", viewport_obj=get_viewport_from_window_name("Viewport")
    )  # isaacsim default viewport
    viewport_manager.change_viewport(
        camera_prim_path=semantic_camera_prim_path, viewport_name="Viewport"
    )

    count = 0
    logger.info("Starting main simulation loop...")

    object_name = "Critical-Package"
    object_prim_path = "/World/Critical_Package"
    object = {
        "shape_type": "cuboid",
        "prim_path": object_prim_path,
        # "scene_name": "object",
        "name": object_name,
        "scale": [0.5, 0.5, 0.5],
        "position": [3, 4.5, 0.25],
        "orientation": [0.707, 0, 0, 0.707],
        "color": [255, 255, 255],
        "mass": 0.1,
        "entity_type": "rigid",
    }
    # 在semantic map中添加这个物体的prim path
    semantic_map.dict_map_semantic[object_name] = object_prim_path
    scene_manager.create_shape_unified(**object)

    # flag = 0
    # LiDAR -------------------------------------------------------------
    # from robot.sensor.lidar.lidar_isaac import CfgLidar, LidarIsaac
    # prim_path = "/World/Critical_Package_Alpha2"
    # lidar_config = "autel_perception_120x352"
    # lidar_cfg = CfgLidar()
    # lidar_cfg.position = [5, 5, 1]
    # lidar_cfg.prim_path = prim_path + "/Lidar/lfr"
    # lidar_cfg.config_file_name = lidar_config
    # lidar = LidarIsaac(
    #     cfg_robot=None,
    #     cfg_lidar=lidar_cfg,
    # )
    # # lidar.copy_lidar_config(lidar_config=lidar_config)
    # lidar.create_lidar(prim_path=lidar_cfg.prim_path)
    # # lidar.lidar.add_linear_depth_data_to_frame()
    # lidar.lidar.add_point_cloud_data_to_frame()
    # lidar.lidar.add_range_data_to_frame()
    # lidar.lidar.add_intensities_data_to_frame()
    # lidar.lidar.add_azimuth_range_to_frame()
    # # lidar.lidar.add_horizontal_resolution_to_frame()
    # lidar.lidar.enable_visualization()
    # lidar.initialize()

    # Build grid map for planning
    grid_map.generate()

    # wait for node planner ompl to receive message
    import time

    time.sleep(2)

    result = True
    # Main simulation loop
    while simulation_app.is_running():
        world.tick()
        ##### navigation usage ex
        if count % 120 == 0 and count > 0:
            result = process_semantic_detection(semantic_camera, semantic_map, "robot")
            print(result)
            result = process_semantic_detection(semantic_camera, semantic_map, "car")
            print(result)

        count += 1

    ros_manager.stop()

    # 统一关闭rclpy上下文
    if rclpy.ok():
        rclpy.shutdown()
        logger.info("ROS context shutdown completed")

    container.unwire()
    loop.close()

    logger.info("--- Simulation finished. Manually closing application. ---")
    if simulation_app:
        simulation_app.__exit__(None, None, None)


if __name__ == "__main__":
    # 直接调用同步 main 函数
    main()
