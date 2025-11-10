try:
    import pydevd_pycharm

    pydevd_pycharm.settrace("localhost", port=12345, stdout_to_server=True, stderr_to_server=True)
except:
    pass

import rclpy
from std_msgs.msg import Header


def run_simulation_loop(simulation_app, world, drones, semantic_camera,
                        semantic_camera_prim_path, viewport_manager):
    """Main simulation loop - physics + lidar publishing"""
    world.reset()
    for _ in range(10):
        simulation_app.update()

    semantic_camera.initialize()

    from omni.kit.viewport.utility import get_viewport_from_window_name
    viewport_manager.register_viewport(name="Viewport", viewport_obj=get_viewport_from_window_name("Viewport"))
    viewport_manager.change_viewport(camera_prim_path=semantic_camera_prim_path, viewport_name="Viewport")

    count = 0
    while simulation_app.is_running():
        # Physics step
        world.tick()

        # Publish lidar data periodically
        if count % 10 == 0:  # Every 10 frames
            for drone in drones:
                if hasattr(drone.robot, 'lidar_list') and drone.robot.lidar_list:
                    t_now = drone.robot.ros_manager.get_node().get_clock().now() if drone.robot.has_ros() else None
                    if t_now:
                        header = Header(stamp=t_now.to_msg(), frame_id="map")

                        pc_lfr = drone.robot.lidar_list[0].get_pointcloud()
                        pc_ubd = drone.robot.lidar_list[1].get_pointcloud()

                        from simulation_utils.message_convert import create_pc2_msg, create_image_msg

                        # Publish via ROS node
                        node = drone.robot.ros_manager.get_node()
                        if hasattr(node, 'publisher_dict'):
                            if "lfr_pc" in node.publisher_dict:
                                node.publisher_dict["lfr_pc"].publish(create_pc2_msg(header, pc_lfr))
                            if "ubd_pc" in node.publisher_dict:
                                node.publisher_dict["ubd_pc"].publish(create_pc2_msg(header, pc_ubd))
                            if "lfr_img" in node.publisher_dict:
                                node.publisher_dict["lfr_img"].publish(
                                    create_image_msg(header, drone.robot.lidar_list[0].get_depth()))
                            if "ubd_img" in node.publisher_dict:
                                node.publisher_dict["ubd_img"].publish(
                                    create_image_msg(header, drone.robot.lidar_list[1].get_depth()))

        count += 1


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


def main():
    from containers import get_container, reset_container
    from config.config_manager import config_manager
    from log.log_manager import LogManager
    import rclpy

    rclpy.init(args=None)
    logger = LogManager.get_logger(__name__)

    reset_container()
    container = get_container()
    container.wire(modules=[__name__])

    config_manager = container.config_manager()
    log_manager = container.log_manager()
    server = container.server()
    ros_manager = container.ros_manager()
    scene_manager = container.scene_manager()
    grid_map = container.grid_map()
    semantic_map = container.semantic_map()
    viewport_manager = container.viewport_manager()

    world = container.world_configured()
    simulation_app = server.get_simulation_app()

    ros_manager.start()

    scene_manager.load_scene(usd_path=config_manager.get("world_usd_path"), prim_path_root="/World/Scene")

    # Create objects using blueprint
    cars = create_car_objects(world)

    result = scene_manager.add_camera(translation=[1, 4, 2], orientation=[1, 0, 0, 0])
    camera_result = result.get("result")

    # Create drones using blueprint (same as main_example.py)
    blueprint_library = world.get_blueprint_library()
    drone_bp = blueprint_library.find('robot.drone_autel')

    drones = []
    for idx, ns in enumerate(config_manager.get("namespace")):
        # Don't override 'type' - let CfgDroneAutel use its default "drone_autel"
        # drone_bp.set_attribute('type', 'autel')  # ❌ This overrides the default
        drone_bp.set_attribute('id', idx)
        drone_bp.set_attribute('namespace', ns)
        drone_bp.set_attribute('color_scheme_id', idx)
        drone_bp.set_attribute('disable_gravity', True)

        drone = world.spawn_actor(drone_bp)
        drones.append(drone)

    # Setup ROS for each drone (same as main_example.py)
    from ros.robot_ros_manager import RobotRosManager
    for drone in drones:
        ros_manager_drone = RobotRosManager(
            robot=drone.robot,
            namespace=drone.robot.cfg_robot.namespace,
            topics=drone.robot.cfg_robot.topics
        )
        drone.robot.set_ros_manager(ros_manager_drone)
        ros_manager_drone.start()
        logger.info(f"✅ ROS enabled for {drone.robot.cfg_robot.namespace}")

    # Initialize drones
    world.reset()
    world.initialize_robots()

    # Add physics callbacks
    for i, drone in enumerate(drones):
        callback_name = f"physics_step_drone_{i}"
        world.get_isaac_world().add_physics_callback(
            callback_name, callback_fn=drone.robot.on_physics_step
        )

    # Setup lidar if needed
    from simulation_utils.ros_bridge import setup_ros
    for drone in drones:
        if hasattr(drone.robot, 'setup_lidar_and_ros'):
            drone.robot.setup_lidar_and_ros(setup_ros)

    # ============================================================================
    # Application Layer Setup (same as main_example.py)
    # ============================================================================

    # 1. Skill System: High-level behaviors via ROS actions
    # Note: cmd_vel is now handled directly in NodeRobot (no separate bridge needed)
    from application import SkillManager
    skill_managers = {}
    for drone in drones:
        skill_manager = SkillManager(drone.robot, auto_register=True)
        drone.robot.skill_manager = skill_manager
        skill_managers[drone.robot.cfg_robot.namespace] = skill_manager

    # ============================================================================
    # Control via ROS2 Actions
    # ============================================================================

    print("\n" + "=" * 80)
    print("LIDAR SIMULATION READY - Use ROS2 Actions to Control Drones")
    print("=" * 80)
    print("\nAvailable drones:")
    for drone in drones:
        print(f"  - {drone.robot.cfg_robot.namespace}")

    print("\nAvailable skills:")
    print("  ROS Required: navigate_to, explore, track, move")
    print("  No ROS: take_off, pick_up, put_down, take_photo, detect, object_detection")

    print("\nExample commands:")
    print("\n  # Take off:")
    print('  ros2 action send_goal /autel_0/skill_execution plan_msgs/action/SkillExecution \\')
    print(
        '    \'{skill_request: {skill_list: [{skill: "take_off", params: [{key: "altitude", value: "2.0"}]}]}}\' --feedback')

    print("\n  # Navigate:")
    print('  ros2 action send_goal /autel_0/skill_execution plan_msgs/action/SkillExecution \\')
    print(
        '    \'{skill_request: {skill_list: [{skill: "navigate_to", params: [{key: "goal_pos", value: "[5, 5, 2]"}]}]}}\' --feedback')

    print("\n  # Control via cmd_vel:")
    print('  ros2 topic pub /autel_0/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}}"')

    print("\n  # View lidar topics:")
    print('  ros2 topic list | grep autel')
    print('  ros2 topic echo /autel_0/lfr_pc')
    print("\n" + "=" * 80 + "\n")

    # ============================================================================
    # Main Simulation Loop
    # ============================================================================

    run_simulation_loop(simulation_app, world, drones, camera_result.get("camera_instance"),
                        camera_result.get("prim_path"), viewport_manager)

    # Cleanup
    for drone in drones:
        if drone.robot.has_ros():
            drone.robot.cleanup()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
