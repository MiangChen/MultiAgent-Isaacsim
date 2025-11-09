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

    # Setup ROS for each robot
    from ros.robot_ros_manager import RobotRosManager
    for robot in robots:
        try:
            # Create ROS manager for this robot
            robot_ros_manager = RobotRosManager(
                robot=robot,
                namespace=robot.namespace,
                topics=robot.body.cfg_robot.topics
            )
            # Inject ROS manager
            robot.set_ros_manager(robot_ros_manager)
            # Start ROS
            robot_ros_manager.start()
            logger.info(f"✅ ROS enabled for {robot.namespace}")
        except Exception as e:
            logger.error(f"❌ Failed to setup ROS for {robot.namespace}: {e}")
            logger.warning(f"⚠️  {robot.namespace} will run without ROS")

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

    # ============================================================================
    # Application Layer Setup
    # ============================================================================
    
    # 1. ROS Control Bridge: ROS cmd_vel -> Control objects
    from ros.ros_control_bridge import RosControlBridgeManager
    ros_bridge_manager = RosControlBridgeManager()
    ros_bridge_manager.add_robots(robots)
    ros_bridge_manager.start()

    # 2. Skill System: High-level behaviors
    from application import SkillManager
    from application.skills.navigate_to import navigate_to
    from application.skills.explore import explore
    from application.skills.take_off import take_off
    from simulation.control import RobotControl
    
    skill_managers = {}
    for robot in robots:
        skill_manager = SkillManager(robot)
        skill_manager.register_skill('navigate_to', navigate_to)
        skill_manager.register_skill('explore', explore)
        skill_manager.register_skill('take_off', take_off)
        skill_managers[robot.namespace] = skill_manager

    # ============================================================================
    # Control Mode Selection (Choose one)
    # ============================================================================
    
    # Mode 1: Direct velocity control (Simulation layer)
    USE_DIRECT_CONTROL = False
    if USE_DIRECT_CONTROL:
        control = RobotControl()
        control.linear_velocity = [1.0, 0.0, 0.0]
        control.angular_velocity = [0.0, 0.0, 0.0]
        for robot in robots:
            robot.apply_control(control)
    
    # Mode 2: Multi-drone mission (Application layer)
    USE_MULTI_DRONE = True
    if USE_MULTI_DRONE and len(robots) > 0:
        # Find drones
        cf2x_0 = None
        cf2x_1 = None
        for robot in robots:
            if robot.namespace == "cf2x_0":
                cf2x_0 = robot
            elif robot.namespace == "cf2x_1":
                cf2x_1 = robot
        
        # cf2x_0: Take off to 5m
        if cf2x_0:
            print("\n" + "="*60)
            print("CF2X_0 MISSION: TAKE OFF")
            print("="*60)
            print(f"Drone: {cf2x_0.namespace}")
            print(f"Current Position: {cf2x_0.body.get_world_pose()[0]}")
            print(f"Target Altitude: 5.0m")
            print("="*60 + "\n")
            
            skill_managers[cf2x_0.namespace].execute_skill(
                'take_off',
                altitude=5.0
            )
        
        # cf2x_1: Navigate to target position
        if cf2x_1:
            print("\n" + "="*60)
            print("CF2X_1 MISSION: NAVIGATE TO TARGET")
            print("="*60)
            print(f"Drone: {cf2x_1.namespace}")
            print(f"Current Position: {cf2x_1.body.get_world_pose()[0]}")
            print(f"Target Position: [10.0, 15.0, 3.0]")
            print("="*60 + "\n")
            
            skill_managers[cf2x_1.namespace].execute_skill(
                'navigate_to',
                goal_pos=[10.0, 15.0, 3.0],
                goal_quat_wxyz=[1.0, 0.0, 0.0, 0.0]
            )
    
    # Mode 3: Explore area (Application layer) - Jetbot Example
    USE_EXPLORATION = False
    if USE_EXPLORATION and len(robots) > 0:
        # Find jetbot (namespace: Allen)
        jetbot = None
        for robot in robots:
            if robot.namespace in ["Allen", "jetbot_0"]:
                jetbot = robot
                break
        
        if jetbot:
            print("\n" + "="*60)
            print("JETBOT EXPLORATION MISSION")
            print("="*60)
            print(f"Robot: {jetbot.namespace}")
            print(f"Start Position: {jetbot.body.get_world_pose()[0]}")
            
            # Jetbot exploration area (irregular polygon)
            # This area covers a complex region for comprehensive coverage
            # Coordinates match: ros2 action send_goal /jetbot_0/skill_execution
            boundary = [
                [-4.4, 12.0, 0.035],   # Point 1 (jetbot z-height: 0.035m)
                [-4.3, 16.0, 0.035],   # Point 2
                [-1.4, 26.0, 0.035],   # Point 3
                [3.0, 27.4, 0.035],    # Point 4
                [3.3, 19.4, 0.035],    # Point 5
                [0.0, 11.0, 0.035],    # Point 6
            ]
            
            print(f"Exploration Area: {len(boundary)} vertices")
            print(f"Target Detection: car")
            print(f"Path Interpolation: 0.1m (10cm)")
            print("="*60 + "\n")
            
            skill_managers[jetbot.namespace].execute_skill(
                'explore',
                boundary=boundary,
                holes=[],
                target_prim="car",
                interpolation_distance=0.1,  # 10cm spacing for smooth jetbot movement
                interpolation_method="linear"
            )
        else:
            # Fallback: use first robot with simple rectangular area
            print("\n[Warning] Jetbot 'Allen' not found, using first robot")
            boundary = [
                [-5.0, -5.0, 1.0],
                [15.0, -5.0, 1.0],
                [15.0, 15.0, 1.0],
                [-5.0, 15.0, 1.0],
            ]
            skill_managers[robots[0].namespace].execute_skill(
                'explore',
                boundary=boundary,
                holes=[],
                target_prim="car"
            )
    
    # Mode 4: ROS control (run in another terminal)
    # ros2 topic pub /robot_0/cmd_vel geometry_msgs/msg/Twist ...

    # ============================================================================
    # Main Simulation Loop
    # ============================================================================
    result = True
    while simulation_app.is_running():
        world.tick()
        
        # Update all active skills (Application layer)
        for namespace, skill_manager in skill_managers.items():
            # Update navigate_to skill
            nav_state = skill_manager.get_skill_state('navigate_to')
            if nav_state in ['EXECUTING', 'INITIALIZING', 'NAVIGATING_TO_START']:
                skill_manager.execute_skill('navigate_to')
            
            # Update explore skill with progress monitoring
            explore_state = skill_manager.get_skill_state('explore')
            if explore_state in ['EXECUTING', 'INITIALIZING', 'NAVIGATING_TO_START']:
                result = skill_manager.execute_skill('explore')
                
                # Print progress every 2 seconds (120 frames at 60fps)
                if count % 120 == 0 and result:
                    status = result.get('status', 'unknown')
                    message = result.get('message', '')
                    progress = result.get('progress', 0)
                    print(f"[{namespace}] Explore: {status} - {message} ({progress}%)")
            
            elif explore_state == 'COMPLETED' and count % 120 == 0:
                print(f"[{namespace}] Exploration COMPLETED!")
            
            elif explore_state == 'FAILED' and count % 120 == 0:
                error = skill_manager.skill_errors.get('explore', 'Unknown')
                print(f"[{namespace}] Exploration FAILED: {error}")
            
            # Update take_off skill
            takeoff_state = skill_manager.get_skill_state('take_off')
            if takeoff_state in ['EXECUTING', 'INITIALIZING', 'NAVIGATING_TO_ALTITUDE']:
                result = skill_manager.execute_skill('take_off')
                
                if count % 120 == 0 and result:
                    status = result.get('status', 'unknown')
                    message = result.get('message', '')
                    progress = result.get('progress', 0)
                    print(f"[{namespace}] Take off: {status} - {message} ({progress}%)")
            
            elif takeoff_state == 'COMPLETED' and count % 120 == 0:
                print(f"[{namespace}] Take off COMPLETED!")
            
            elif takeoff_state == 'FAILED' and count % 120 == 0:
                error = skill_manager.skill_errors.get('take_off', 'Unknown')
                print(f"[{namespace}] Take off FAILED: {error}")
        
        # Semantic detection
        if count % 120 == 0 and count > 0:
            process_semantic_detection(semantic_camera, semantic_map, "robot")
            process_semantic_detection(semantic_camera, semantic_map, "car")

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
