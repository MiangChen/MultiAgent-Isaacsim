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
#   └─ Physics engine, rendering, sensor
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


# =============================================================================
# Helper Functions
# =============================================================================

def create_car_objects(world):
    """Create car objects using blueprint - CARLA style"""
    from simulation import Transform, Location, Rotation

    blueprint_library = world.get_blueprint_library()

    cars_config = [
        {"name": "car0", "position": [11.6, 3.5, 0], "prim_path": "/World/car1"},
        {"name": "car1", "position": [0.3, 3.5, 0], "prim_path": "/World/car2"},
        {"name": "car2", "position": [-13.2, 3.5, 0], "prim_path": "/World/car3"},
        {"name": "car3", "position": [-7.1, 10, 0], "prim_path": "/World/car4"},
        {
            "name": "car4",
            "position": [-0.9, 30, 0],
            "orientation": [0.707, 0, 0, 0.707],
            "prim_path": "/World/car5",
        },
    ]

    cars = []
    for cfg in cars_config:
        car_bp = blueprint_library.find("static.prop.car")
        car_bp.set_attribute("name", cfg["name"])
        car_bp.set_attribute("scale", [2, 5, 1.0])
        car_bp.set_attribute("color", [255, 255, 255])
        car_bp.set_attribute("semantic_label", "car")
        car_bp.set_attribute("prim_path", cfg["prim_path"])

        transform = Transform(location=Location(*cfg["position"]))
        if "orientation" in cfg:
            transform.rotation = Rotation(quaternion=cfg["orientation"])

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


# =============================================================================
# Main Function
# =============================================================================

def main():
    print("\n\n\n\ninto the main\n\n\n\n")

    # =========================================================================
    # 1. INITIALIZATION
    # =========================================================================
    
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
    container.wire(modules=[__name__])

    # Get services from container
    config_manager = container.config_manager()
    log_manager = container.log_manager()
    loop = container.loop()
    server = container.server()
    ros_manager_isaac = container.ros_manager_isaac()
    scene_manager = container.scene_manager()
    grid_map = container.grid_map()
    semantic_map = container.semantic_map()
    viewport_manager = container.viewport_manager()

    world = container.world_configured()
    simulation_app = server.get_simulation_app()

    # Start ROS manager
    ros_manager_isaac.start()

    # =========================================================================
    # 2. CREATION - Robots, Scene, Objects
    # =========================================================================
    
    logger.info("=" * 80)
    logger.info("CREATION PHASE")
    logger.info("=" * 80)

    # 2.1 Load Robots from Config (CARLA style)
    logger.info("Loading robots from config...")
    robot_actors = world.load_actors_from_config(
        f"{PROJECT_ROOT}/config/robot_swarm_cfg.yaml"
    )
    robots = [actor.robot for actor in robot_actors]
    logger.info(f"✅ Loaded {len(robots)} robots")

    # 2.2 Load Scene
    logger.info("Loading scene...")
    scene_manager.load_scene(usd_path=WORLD_USD_PATH, prim_path_root="/World/scene")
    logger.info("✅ Scene loaded")

    # 2.3 Create Static Objects (CARLA style)
    logger.info("Creating static objects...")
    from simulation import Transform, Location, Rotation
    
    blueprint_library = world.get_blueprint_library()

    # Create cars
    cars = create_car_objects(world)
    logger.info(f"✅ Created {len(cars)} cars")

    # Create critical package
    package_bp = blueprint_library.find("static.prop.box")
    package_bp.set_attribute("name", "Critical-Package")
    package_bp.set_attribute("scale", [0.5, 0.5, 0.5])
    package_bp.set_attribute("color", [255, 255, 255])
    package_bp.set_attribute("mass", 0.01)
    package_bp.set_attribute("entity_type", "rigid")
    package_bp.set_attribute("semantic_label", "package")
    package_bp.set_attribute("prim_path", "/World/package")

    package_transform = Transform(
        location=Location(3, 4.5, 0.25),
        rotation=Rotation(quaternion=[0.707, 0, 0, 0.707]),
    )
    package = world.spawn_actor(package_bp, package_transform)
    logger.info("✅ Created package")

    # 2.4 Initialize World
    logger.info("Initializing world...")
    world.reset()
    world.initialize_map()
    world.initialize_robots()
    logger.info("✅ World initialized")

    # 2.5 Add Physics Callbacks
    logger.info("Adding physics callbacks...")
    for i, robot in enumerate(robots):
        callback_name = f"physics_step_robot_{i}"
        world.get_isaac_world().add_physics_callback(
            callback_name, callback_fn=robot.on_physics_step
        )
    logger.info(f"✅ Added {len(robots)} physics callbacks")

    # 2.6 Create Sensors (CARLA style)
    logger.info("Creating sensors...")
    
    # Add camera to h1_0
    h1_actor = None
    for actor in robot_actors:
        if hasattr(actor, "robot") and actor.robot.namespace == "h1_0":
            h1_actor = actor
            break

    if h1_actor:
        camera_bp = blueprint_library.find("sensor.camera.rgb")
        camera_bp.set_attribute("image_size_x", 1280)
        camera_bp.set_attribute("image_size_y", 720)
        camera_bp.set_attribute("focal_length", 2)
        camera_bp.set_attribute("enable_semantic_detection", True)

        camera_transform = Transform(
            location=Location(x=0.1, y=0.01, z=0.69),
            rotation=Rotation(quaternion=[0.5, -0.5, -0.5, 0.5]),
        )

        h1_camera = world.spawn_actor(camera_bp, camera_transform, attach_to=h1_actor)
        logger.info(f"✅ Camera added to h1_0 (1280x720, focal: 2mm)")
    else:
        logger.warning("h1_0 robot not found, skipping camera")

    # Add LiDAR to cf2x_0
    cf2x_actor = None
    for actor in robot_actors:
        if hasattr(actor, "robot") and actor.robot.namespace == "cf2x_0":
            cf2x_actor = actor
            break

    if cf2x_actor:
        omni_lidar_bp = blueprint_library.find("sensor.lidar.omni")
        omni_lidar_bp.set_attribute("config_file_name", "autel_perception_120x352")
        omni_lidar_bp.set_attribute("erp_height", 352)
        omni_lidar_bp.set_attribute("erp_width", 120)
        omni_lidar_bp.set_attribute("output_size", (352, 120))
        omni_lidar_bp.set_attribute("max_depth", 100.0)
        omni_lidar_bp.set_attribute("frequency", 10)
        # 指定LiDAR attach到机器人的哪个相对prim路径
        # cf2x_0机器人的完整路径是 /World/robot/cf2x/cf2x_0
        # 这里指定attach到 "/body" 组件
        omni_lidar_bp.set_attribute("attach_prim_relative_path", "/body")

        omni_lidar_transform = Transform(
            location=Location(x=0.0, y=0.0, z=0.1),
            rotation=Rotation(quaternion=[0, 0.0, 0.0, 1]),
        )

        omni_lidar = world.spawn_actor(
            omni_lidar_bp, omni_lidar_transform, attach_to=cf2x_actor
        )
        logger.info(f"✅ Omni LiDAR added to cf2x_0 (352x120, 10Hz, attached to /body)")
    else:
        logger.warning("cf2x_0 robot not found, skipping LiDAR")

    # 2.7 Setup Semantic Camera
    logger.info("Setting up semantic camera...")
    result = scene_manager.add_camera(
        translation=[1, 4, 2], orientation=euler_to_quat(roll=90)
    )
    semantic_camera = result.get("result").get("camera_instance")
    semantic_camera_prim_path = result.get("result").get("prim_path")
    semantic_camera.initialize()

    # Wait for camera initialization
    for _ in range(10):
        world.tick()

    semantic_camera.add_bounding_box_2d_loose_to_frame()
    logger.info("✅ Semantic camera initialized")

    # 2.8 Setup Viewport
    logger.info("Setting up viewport...")
    from physics_engine.omni_utils import get_viewport_from_window_name

    viewport_manager.register_viewport(
        name="Viewport", viewport_obj=get_viewport_from_window_name("Viewport")
    )
    viewport_manager.change_viewport(
        camera_prim_path=semantic_camera_prim_path, viewport_name="Viewport"
    )
    logger.info("✅ Viewport configured")

    # =========================================================================
    # 3. APPLICATION LAYER SETUP
    # =========================================================================
    
    logger.info("=" * 80)
    logger.info("APPLICATION LAYER SETUP")
    logger.info("=" * 80)

    # 3.1 Setup ROS for Each Robot
    logger.info("Setting up ROS for robots...")
    from ros.ros_manager_robot import RobotRosManager

    for robot in robots:
        robot_ros_manager = RobotRosManager(
            robot=robot, namespace=robot.namespace, topics=robot.get_topics()
        )
        robot.set_ros_manager(robot_ros_manager)
        robot_ros_manager.start()
        logger.info(f"✅ ROS enabled for {robot.namespace}")

    # 3.2 Setup Skill System
    logger.info("Setting up skill system...")
    from application import SkillManager

    skill_managers = {}
    for robot in robots:
        skill_manager = SkillManager(robot, auto_register=True)
        robot.skill_manager = skill_manager
        skill_managers[robot.namespace] = skill_manager
        logger.info(f"✅ Skill manager created for {robot.namespace}")

    # 3.3 Attach Sensors to ROS (CARLA style)
    logger.info("Attaching sensors to ROS...")
    
    if cf2x_actor:
        cf2x_robot = cf2x_actor.robot
        if cf2x_robot.has_ros():
            ros_manager = cf2x_robot.get_ros_manager()
            ros_manager.attach_sensor_to_ros(omni_lidar, "lidar")
            logger.info(f"✅ Omni LiDAR publishing to /cf2x_0/lidar/points")

    # 3.4 Build Grid Map for Planning
    logger.info("Building grid map...")
    grid_map.generate()
    logger.info("✅ Grid map generated")

    # =========================================================================
    # 4. MAIN LOOP
    # =========================================================================
    
    logger.info("=" * 80)
    logger.info("STARTING MAIN LOOP")
    logger.info("=" * 80)

    count = 0

    while simulation_app.is_running():
        # Physics step
        world.tick()

        # Optional: Semantic detection every 2 seconds
        if count % 240 == 0 and count > 0:
            process_semantic_detection(semantic_camera, semantic_map, "robot")
            process_semantic_detection(semantic_camera, semantic_map, "car")

        count += 1

    # =========================================================================
    # 5. CLEANUP
    # =========================================================================
    
    logger.info("=" * 80)
    logger.info("CLEANUP")
    logger.info("=" * 80)

    for robot in robots:
        if robot.has_ros():
            robot.cleanup()
            logger.info(f"✅ Cleaned up {robot.namespace}")

    if rclpy.ok():
        rclpy.shutdown()
        logger.info("✅ ROS shutdown")

    container.unwire()
    loop.close()

    if simulation_app:
        simulation_app.__exit__(None, None, None)
        logger.info("✅ Simulation app closed")

    logger.info("=" * 80)
    logger.info("DONE")
    logger.info("=" * 80)


if __name__ == "__main__":
    main()
