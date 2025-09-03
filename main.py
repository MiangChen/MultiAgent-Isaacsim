# Standard library imports
import asyncio
import signal
import sys
import threading

# Third-party imports
from dependency_injector.wiring import inject, Provide  # Dependency injection imports

# Local imports - argument parsing
from argument_parser import parse_arguments, get_argument_summary

# Isaac Sim related imports
from physics_engine.isaacsim_simulation_app import initialize_simulation_app_from_yaml

# Parse arguments first
args = parse_arguments()

# Initialize simulation app with parsed config
simulation_app = initialize_simulation_app_from_yaml(args.config)
from isaacsim.core.api import World

# Local imports
from environment.env import Env
from files.variables import WORLD_USD_PATH, PATH_PROJECT
from map.map_grid_map import GridMap
from map.map_semantic_map import MapSemantic
from robot.robot_cf2x import RobotCf2x, RobotCfgCf2x
from robot.robot_h1 import RobotH1, RobotCfgH1
from robot.robot_jetbot import RobotCfgJetbot, RobotJetbot
from robot.swarm_manager import SwarmManager
from scene.scene_manager import SceneManager
from containers import AppContainer
from webmanager.integration import WebManagerSystem
from log.log_manager import LogManager

# Log startup information with argument summary
logger = LogManager.get_logger(__name__)
logger.info("Isaac Sim WebManager starting...")
logger.info("\n" + get_argument_summary(args))

# Determine if WebManager should be enabled
webmanager_enabled = args.enable_webmanager and not args.disable_webmanager

ROS_AVAILABLE = True

# Global variables for WebManager integration
_webmanager_system = None
_webmanager_thread = None
_webmanager_stop_event = None

# Global shutdown flag and process manager
_shutdown_requested = False
_process_manager = None


def signal_handler(signum, frame):
    """Handle shutdown signals gracefully"""
    global _shutdown_requested, _process_manager

    signal_names = {signal.SIGINT: "SIGINT (Ctrl+C)", signal.SIGTERM: "SIGTERM"}

    signal_name = signal_names.get(signum, f"Signal {signum}")
    logger.info(f"Received {signal_name}, initiating graceful shutdown...")

    _shutdown_requested = True

    # Use process manager for coordinated shutdown
    if _process_manager:
        _process_manager.request_shutdown()

    # Stop WebManager system gracefully
    if _webmanager_system:
        logger.info("Stopping WebManager system due to shutdown signal...")
        try:
            stop_webmanager_system()
            logger.info("WebManager system stopped successfully")
        except Exception as e:
            logger.error(f"Error stopping WebManager system: {e}")

    # Stop ROS nodes if running
    global stop_evt, t_ros
    if stop_evt:
        logger.info("Stopping ROS nodes...")
        stop_evt.set()
        if t_ros and t_ros.is_alive():
            t_ros.join(timeout=3.0)
            if t_ros.is_alive():
                logger.warning("ROS thread did not stop within timeout")

    # For SIGINT (Ctrl+C), we can exit immediately
    if signum == signal.SIGINT:
        logger.info("Immediate shutdown requested")
        # Cleanup process manager before exit
        if _process_manager:
            _process_manager.cleanup()
        sys.exit(0)


def initialize_webmanager_system(
        swarm_manager, viewport_manager, simulation_app, config, ros_monitor=None
) -> WebManagerSystem:
    """Initialize and configure the WebManager system for Isaac Sim integration

    Args:
        swarm_manager: SwarmManager instance for robot data
        viewport_manager: ViewportManager instance for camera data
        simulation_app: Isaac Sim application instance
        config: WebManagerConfig instance with configuration
        ros_monitor: Optional ROS monitor node

    Returns:
        WebManagerSystem: Configured WebManager system
    """
    global _webmanager_system

    try:
        logger.info("Initializing WebManager system...")
        logger.info(
            f"WebManager configuration: host={config.web_host}, port={config.web_port}"
        )
        logger.info(
            f"Data collection rate: {config.data_collection_rate}Hz, max history: {config.max_history}"
        )

        # Create WebManager system with configuration
        _webmanager_system = WebManagerSystem(
            host=config.web_host,
            port=config.web_port,
            collection_rate=config.data_collection_rate,
            max_history=config.max_history,
        )

        # Configure WebManager features from config
        webmanager_config = {
            "enable_camera_streaming": not config.disable_camera_streaming,
            "camera_quality": config.camera_quality,
            "enable_compression": config.enable_compression,
            "log_level": config.log_level,
        }

        # Initialize with Isaac Sim components
        _webmanager_system.initialize(
            swarm_manager=swarm_manager,
            isaac_sim_app=simulation_app,
            viewport_manager=viewport_manager,
            ros_monitor=ros_monitor,
            config=webmanager_config,
        )

        logger.info("WebManager system initialized successfully")
        logger.info(
            f"WebManager web interface will be available at http://{config.web_host}:{config.web_port}"
        )
        return _webmanager_system

    except Exception as e:
        logger.error(f"Failed to initialize WebManager system: {e}")
        raise


def start_webmanager_in_thread():
    """Start WebManager system in a separate thread"""
    global _webmanager_system, _webmanager_thread, _webmanager_stop_event

    if not _webmanager_system:
        logger.error("WebManager system not initialized")
        return


def webmanager_thread_worker():
    """Worker function for WebManager thread"""
    try:
        logger.info("Starting WebManager system in separate thread...")
        _webmanager_system.start()

        # Keep thread alive while system is running
        while not _webmanager_stop_event.is_set():
            _webmanager_stop_event.wait(timeout=1.0)

    except Exception as e:
        logger.error(f"Error in WebManager thread: {e}")
    finally:
        logger.info("WebManager thread shutting down")


# Create stop event and thread

_webmanager_stop_event = threading.Event()
_webmanager_thread = threading.Thread(
    target=webmanager_thread_worker, name="WebManagerThread", daemon=True
)

# Start the thread
_webmanager_thread.start()
logger.info("WebManager thread started")


def stop_webmanager_system():
    """Stop the WebManager system and cleanup resources"""
    global _webmanager_system, _webmanager_thread, _webmanager_stop_event

    try:
        logger.info("Initiating WebManager system shutdown...")

        # Signal the thread to stop
        if _webmanager_stop_event:
            logger.debug("Setting WebManager stop event")
            _webmanager_stop_event.set()

        # Stop the WebManager system
        if _webmanager_system:
            logger.debug("Stopping WebManager system")
            _webmanager_system.stop()

            # Report final statistics
            try:
                stats = _webmanager_system.get_system_status()
                logger.info(f"WebManager final status: {stats}")
            except Exception as e:
                logger.debug(f"Could not get final statistics: {e}")

        # Wait for thread to finish
        if _webmanager_thread and _webmanager_thread.is_alive():
            logger.debug("Waiting for WebManager thread to finish...")
            _webmanager_thread.join(timeout=5.0)

            if _webmanager_thread.is_alive():
                logger.warning(
                    "WebManager thread did not shut down gracefully within 5 seconds"
                )
                # Force thread termination is not recommended in Python
                # The daemon thread will be terminated when the main process exits
            else:
                logger.debug("WebManager thread finished successfully")

        # Reset global variables
        _webmanager_system = None
        _webmanager_thread = None
        _webmanager_stop_event = None

        logger.info("WebManager system stopped successfully")

    except Exception as e:
        logger.error(f"Error stopping WebManager system: {e}")
        # Continue with shutdown even if WebManager cleanup fails


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
            robot_swarm_cfg_path=f"{PATH_PROJECT}/files/robot_swarm_cfg.yaml",
            robot_active_flag_path=f"{PATH_PROJECT}/files/robot_swarm_active_flag.yaml",
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
            "position": [11.6, 3.5, 0],
            "color": [255, 255, 255],
            "make_dynamic": False,
        },
        "car1": {
            "shape_type": "cuboid",
            "prim_path": "/World/car1",
            "size": scale,
            "position": [0.3, 3.5, 0],
            "color": [255, 255, 255],
            "make_dynamic": False,
        },
        "car2": {
            "shape_type": "cuboid",
            "prim_path": "/World/car2",
            "size": scale,
            "position": [-13.2, 3.5, 0],
            "color": [255, 255, 255],
            "make_dynamic": False,
        },
        "car3": {
            "shape_type": "cuboid",
            "prim_path": "/World/car3",
            "size": scale,
            "position": [-7.1, 10, 0],
            "color": [255, 255, 255],
            "make_dynamic": False,
        },
        "car4": {
            "shape_type": "cuboid",
            "prim_path": "/World/car4",
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
    print(
        "\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\ninto the main\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n"
    )

    # Setup dependency injection container
    from containers import get_container

    container = get_container()

    # Wire the container to this module for @inject decorators in skill functions
    container.wire(modules=[__name__])

    ### for Web Manager ###
    # Create process manager configuration from command line arguments
    from webmanager.startup_config import WebManagerConfig
    from webmanager.process_manager import ProcessManager

    global _shutdown_requested, _process_manager

    # Create configuration object from parsed arguments
    config = WebManagerConfig.from_args_direct(args)

    # Initialize process manager
    _process_manager = ProcessManager(config)
    _process_manager.set_status("starting")

    logger.info("Process manager initialized")

    # Register shutdown callbacks
    def webmanager_shutdown_callback():
        """Shutdown callback for WebManager system"""
        if _webmanager_system:
            try:
                stop_webmanager_system()
            except Exception as e:
                logger.error(f"Error in WebManager shutdown callback: {e}")

    # Register signal handlers for graceful shutdown
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    logger.info("Signal handlers registered for graceful shutdown")

    # Get services from container
    ros_manager = container.ros_manager()
    swarm_manager = container.swarm_manager()
    scene_manager = container.scene_manager()
    grid_map = container.grid_map()
    semantic_map = container.semantic_map()
    viewport_manager = container.viewport_manager()
    world = container.world()
    env = container.env()
    env.simulation_app = simulation_app
    setup_simulation()

    ros_manager.start()
    _process_manager.register_shutdown_callback(webmanager_shutdown_callback)
    _process_manager.register_shutdown_callback(ros_manager.stop)

    # Start health monitoring
    _process_manager.start_health_monitoring()

    # Load scene
    scene_manager.load_scene(usd_path=WORLD_USD_PATH)

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

    # Initialize and start WebManager system if enabled
    if webmanager_enabled:
        try:
            logger.info("Starting WebManager system...")
            # Get ROS monitor from nodes if available
            ros_monitor = (
                ros_manager.scene_monitor_node
                if ros_manager.scene_monitor_node
                else None
            )

            webmanager_system = initialize_webmanager_system(
                swarm_manager=swarm_manager,
                viewport_manager=viewport_manager,
                simulation_app=simulation_app,
                config=config,
                ros_monitor=ros_monitor,
            )
            start_webmanager_in_thread()

            # Update process manager status
            if _process_manager:
                _process_manager.set_status("running")

            logger.info("WebManager system started successfully")
            logger.info(
                f"Access the web interface at: http://{args.web_host}:{args.web_port}"
            )
        except Exception as e:
            logger.error(f"Failed to start WebManager system: {e}")
            if _process_manager:
                _process_manager.report_error(f"WebManager startup failed: {e}")
            logger.warning("Continuing simulation without WebManager")
            # Continue without WebManager if it fails to start
    else:
        logger.info("WebManager disabled by command line arguments")
        if _process_manager:
            _process_manager.set_status("running")

    count = 0
    logger.info("Starting main simulation loop...")

    # Main simulation loop
    while simulation_app.is_running() and not _shutdown_requested:

        # Check if shutdown was requested via process manager
        if _process_manager and _process_manager.is_shutdown_requested():
            logger.info("Shutdown requested via process manager")
            break

        # World step
        env.step(action=None)

        if count % 120 == 0 and count > 0:
            process_semantic_detection(semantic_camera, semantic_map)

        # Process ROS skills if ROS is enabled
        if args.ros:
            from skill.skill import process_ros_skills

            process_ros_skills(swarm_manager)

        count += 1

        # Update process manager with WebManager statistics
        if (
                _process_manager and _webmanager_system and count % 60 == 0
        ):  # Every second at 60 FPS
            try:
                # Get WebManager connection count if available
                connection_count = 0
                if hasattr(_webmanager_system, "web_server") and hasattr(
                        _webmanager_system.web_server, "websocket_manager"
                ):
                    connection_count = (
                        _webmanager_system.web_server.websocket_manager.get_connection_count()
                    )

                _process_manager.update_webmanager_stats(
                    active_connections=connection_count,
                    total_requests=count,  # Use simulation steps as a proxy for activity
                )
            except Exception as e:
                logger.debug(f"Error updating WebManager stats: {e}")

        # Log status periodically
        if count % 600 == 0:  # Every 10 seconds at 60 FPS
            logger.debug(f"Simulation running - step {count}")
            if _process_manager:
                status = _process_manager.get_status()
                logger.debug(
                    f"Process status: {status.status}, uptime: {status.uptime_seconds:.1f}s, memory: {status.memory_usage_mb:.1f}MB"
                )

    if _shutdown_requested:
        logger.info("Shutdown requested, exiting main loop")
    elif not simulation_app.is_running():
        logger.info("Simulation app stopped, exiting main loop")

    # Clean shutdown
    logger.info("Starting graceful shutdown...")

    # Stop WebManager system if it was enabled
    if webmanager_enabled:
        logger.info("Stopping WebManager system...")
        stop_webmanager_system()

    # Stop ROS
    if stop_evt:
        stop_evt.set()
    if t_ros:
        t_ros.join(timeout=1.0)

    logger.info("Simulation loop ended, starting cleanup...")

    # Cleanup process manager and report final status
    if _process_manager:
        logger.info("Cleaning up process manager...")
        _process_manager.set_status("stopping")

        # Wait for graceful shutdown if requested
        if _process_manager.is_shutdown_requested():
            logger.info(
                f"Waiting for graceful shutdown (timeout: {args.shutdown_timeout}s)..."
            )
            _process_manager.wait_for_shutdown(timeout=args.shutdown_timeout)

        # Get final status report
        final_status = _process_manager.get_status()
        logger.info(
            f"Final process status: uptime={final_status.uptime_seconds:.1f}s, "
            f"memory={final_status.memory_usage_mb:.1f}MB, "
            f"errors={final_status.error_count}"
        )

        # Cleanup process manager resources
        _process_manager.cleanup()
        logger.info("Process manager cleanup completed")

        # Unwire the container
        from containers import get_container

        container = get_container()
        container.unwire()

        logger.info("--- Simulation finished. Manually closing application. ---")
        if simulation_app:
            simulation_app.__exit__(None, None, None)


if __name__ == "__main__":
    # 直接调用同步 main 函数
    main()
