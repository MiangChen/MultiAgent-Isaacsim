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


def create_car_objects(world):
    """Create car objects using blueprint - CARLA style"""
    from simulation import Transform, Location, Rotation
    
    blueprint_library = world.get_blueprint_library()
    
    cars_config = [
        {"name": "car0", "position": [11.6, 3.5, 0]},
        {"name": "car1", "position": [0.3, 3.5, 0]},
        {"name": "car2", "position": [-13.2, 3.5, 0]},
        {"name": "car3", "position": [-7.1, 10, 0]},
        {"name": "car4", "position": [-0.9, 30, 0], "orientation": [0.707, 0, 0, 0.707]},
    ]
    
    cars = []
    for cfg in cars_config:
        car_bp = blueprint_library.find('static.prop.car')
        car_bp.set_attribute('name', cfg['name'])
        car_bp.set_attribute('scale', [2, 5, 1.0])
        car_bp.set_attribute('color', [255, 255, 255])
        car_bp.set_attribute('semantic_label', 'car')
        
        transform = Transform(location=Location(*cfg['position']))
        if 'orientation' in cfg:
            transform.rotation = Rotation(quaternion=cfg['orientation'])
        
        car = world.spawn_actor(car_bp, transform)
        cars.append(car)
    
    return cars


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
    scene_manager = container.scene_manager()
    grid_map = container.grid_map()
    semantic_map = container.semantic_map()
    viewport_manager = container.viewport_manager()

    world = container.world_configured()
    simulation_app = server.get_simulation_app()

    # Load robots from config - CARLA style (blueprints auto-registered)
    robots = world.load_actors_from_config(f"{PROJECT_ROOT}/config/robot_swarm_cfg.yaml")

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

    # Add physics callbacks for active robots - CARLA style: iterate actors
    for i, robot in enumerate(robots):
        callback_name = f"physics_step_robot_{i}"
        world.get_isaac_world().add_physics_callback(
            callback_name, callback_fn=robot.on_physics_step
        )

    # Create cars using blueprint - CARLA style
    cars = create_car_objects(world)
    
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

    # Create critical package using blueprint - CARLA style
    from simulation import Transform, Location, Rotation
    
    object_name = "Critical-Package"
    blueprint_library = world.get_blueprint_library()
    package_bp = blueprint_library.find('static.prop.box')
    package_bp.set_attribute('name', object_name)
    package_bp.set_attribute('scale', [0.5, 0.5, 0.5])
    package_bp.set_attribute('color', [255, 255, 255])
    package_bp.set_attribute('mass', 0.1)
    package_bp.set_attribute('entity_type', 'rigid')
    package_bp.set_attribute('semantic_label', 'package')
    
    transform = Transform(
        location=Location(3, 4.5, 0.25),
        rotation=Rotation(quaternion=[0.707, 0, 0, 0.707])
    )
    package = world.spawn_actor(package_bp, transform)

    # Build grid map for planning
    grid_map.generate()

    # Setup ROS control bridge (Application layer: ROS -> Control -> Simulation)
    from ros.ros_control_bridge import RosControlBridgeManager
    from simulation.control import RobotControl
    
    ros_bridge_manager = RosControlBridgeManager()
    ros_bridge_manager.add_robots(robots)
    ros_bridge_manager.start()

    # Test: Set all robots to move forward at 1 m/s
    control = RobotControl()
    control.linear_velocity = [1.0, 0.0, 0.0]
    control.angular_velocity = [0.0, 0.0, 0.0]
    
    for robot in robots:
        robot.apply_control(control)

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

    # Cleanup
    ros_bridge_manager.stop()
    ros_manager.stop()

    if rclpy.ok():
        rclpy.shutdown()

    container.unwire()
    loop.close()

    if simulation_app:
        simulation_app.__exit__(None, None, None)


if __name__ == "__main__":
    # 直接调用同步 main 函数
    main()
