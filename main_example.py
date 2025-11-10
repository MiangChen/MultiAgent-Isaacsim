# =============================================================================
# Main Example - Isaac Sim Robotics Simulation Demo
# =============================================================================
#
# Architecture (3 Layers):
#
#   [Application Layer]
#   ├─ Skills: navigate_to, explore, grasp, etc.
#   │  └─ Flow: ROS planning -> MPC velocity -> RobotControl -> apply_control()
#   └─ ROS Bridge: ROS cmd_vel -> RobotControl -> apply_control()
#
#   [Simulation Layer] 
#   ├─ Control: RobotControl objects (CARLA-style)
#   ├─ API: robot.apply_control(control), world.spawn_actor()
#   └─ Blueprint: Robot and object creation
#
#   [Isaac Sim]
#   └─ Physics engine, rendering, sensors
#
# Control Modes:
#   1. Direct: robot.apply_control(control)
#   2. Skills: skill_manager.execute_skill('navigate_to', goal_pos=[10,20,0])
#   3. ROS: ros2 topic pub /robot_0/cmd_vel ...
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
        {"name": "car0", "position": [11.6, 3.5, 0], "prim_path": "/World/car1"},
        {"name": "car1", "position": [0.3, 3.5, 0], "prim_path": "/World/car2"},
        {"name": "car2", "position": [-13.2, 3.5, 0], "prim_path": "/World/car3"},
        {"name": "car3", "position": [-7.1, 10, 0], "prim_path": "/World/car4"},
        {"name": "car4", "position": [-0.9, 30, 0], "orientation": [0.707, 0, 0, 0.707], "prim_path": "/World/car5"},
    ]

    cars = []
    for cfg in cars_config:
        car_bp = blueprint_library.find('static.prop.car')
        car_bp.set_attribute('name', cfg['name'])
        car_bp.set_attribute('scale', [2, 5, 1.0])
        car_bp.set_attribute('color', [255, 255, 255])
        car_bp.set_attribute('semantic_label', 'car')
        car_bp.set_attribute('prim_path', cfg['prim_path'])

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
    
    ros_manager.start()

    # ============================================================================
    # Robots Setup
    # ============================================================================
    
    # Load robots from config - CARLA style (blueprints auto-registered)
    robot_actors = world.load_actors_from_config(f"{PROJECT_ROOT}/config/robot_swarm_cfg.yaml")
    robots = [actor.robot for actor in robot_actors]  # Extract robot objects from actors

    # ============================================================================
    # Scene Setup
    # ============================================================================
    
    # Load scene
    scene_manager.load_scene(usd_path=WORLD_USD_PATH, prim_path_root="/World/Scene")
    
    # Create static objects using blueprint - CARLA style
    from simulation import Transform, Location, Rotation
    
    # Create cars
    cars = create_car_objects(world)
    
    # Create critical package
    blueprint_library = world.get_blueprint_library()
    package_bp = blueprint_library.find('static.prop.box')
    package_bp.set_attribute('name', 'Critical-Package')
    package_bp.set_attribute('scale', [0.5, 0.5, 0.5])
    package_bp.set_attribute('color', [255, 255, 255])
    package_bp.set_attribute('mass', 0.01)
    package_bp.set_attribute('entity_type', 'rigid')
    package_bp.set_attribute('semantic_label', 'package')
    package_bp.set_attribute("prim_path", "/World/package")

    package_transform = Transform(
        location=Location(3, 4.5, 0.25),
        rotation=Rotation(quaternion=[0.707, 0, 0, 0.707])
    )
    package = world.spawn_actor(package_bp, package_transform)
    
    # Initialize world
    world.reset()
    world.initialize_map()
    
    # Initialize robots
    world.initialize_robots()
    
    # Add physics callbacks for robots
    for i, robot in enumerate(robots):
        callback_name = f"physics_step_robot_{i}"
        world.get_isaac_world().add_physics_callback(
            callback_name, callback_fn=robot.on_physics_step
        )
    
    # Setup semantic camera
    result = scene_manager.add_camera(
        translation=[1, 4, 2], orientation=euler_to_quat(roll=90)
    )
    semantic_camera = result.get("result").get("camera_instance")
    semantic_camera_prim_path = result.get("result").get("prim_path")
    semantic_camera.initialize()

    # Wait for camera and rendering pipeline to fully initialize
    for _ in range(10):
        world.tick()

    # Enable bounding box detection
    semantic_camera.add_bounding_box_2d_loose_to_frame()

    # Switch viewport to semantic camera
    from physics_engine.omni_utils import get_viewport_from_window_name
    viewport_manager.register_viewport(
        name="Viewport", viewport_obj=get_viewport_from_window_name("Viewport")
    )
    viewport_manager.change_viewport(
        camera_prim_path=semantic_camera_prim_path, viewport_name="Viewport"
    )

    # Build grid map for planning
    grid_map.generate()

    # ============================================================================
    # Application Layer Setup
    # ============================================================================
    
    # 1. Setup ROS for each robot
    from ros.robot_ros_manager import RobotRosManager
    
    for robot in robots:
        try:
            # Create ROS manager for this robot
            robot_ros_manager = RobotRosManager(
                robot=robot,
                namespace=robot.namespace,
                topics=robot.get_topics()
            )
            # Inject ROS manager
            robot.set_ros_manager(robot_ros_manager)
            # Start ROS
            robot_ros_manager.start()
            logger.info(f"✅ ROS enabled for {robot.namespace}")
        except Exception as e:
            logger.error(f"❌ Failed to setup ROS for {robot.namespace}: {e}")
            raise f"Failed to setup ROS for {robot.namespace}"

    # 2. Skill System: High-level behaviors via ROS actions
    # Note: cmd_vel is now handled directly in NodeRobot (no separate bridge needed)
    from application import SkillManager

    skill_managers = {}
    for robot in robots:
        # SkillManager auto-registers all skills from global registry
        skill_manager = SkillManager(robot, auto_register=True)
        # Attach to robot for ROS action server
        robot.skill_manager = skill_manager
        skill_managers[robot.namespace] = skill_manager

    # ============================================================================
    # Main Simulation Loop
    # ============================================================================

    count = 0

    while simulation_app.is_running():
        # Physics step
        world.tick()

        # Optional: Semantic detection every 2 seconds
        if count % 120 == 0 and count > 0:
            process_semantic_detection(semantic_camera, semantic_map, "robot")
            process_semantic_detection(semantic_camera, semantic_map, "car")

        count += 1

    # Cleanup
    for robot in robots:
        if robot.has_ros():
            robot.cleanup()

    if rclpy.ok():
        rclpy.shutdown()

    container.unwire()
    loop.close()

    if simulation_app:
        simulation_app.__exit__(None, None, None)


if __name__ == "__main__":
    # 直接调用同步 main 函数
    main()
