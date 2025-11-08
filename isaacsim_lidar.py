try:
    import pydevd_pycharm
    pydevd_pycharm.settrace("localhost", port=12345, stdout_to_server=True, stderr_to_server=True)
except:
    pass


def build_drone_ctx(namespace: str, idx: int, world, setup_ros):
    from robot.sensor.lidar.lidar_omni import LidarOmni
    from robot.sensor.lidar.cfg_lidar import CfgLidar
    from robot.robot_drone_autel import RobotDroneAutel
    
    prim_path = f"/{namespace}/drone" if namespace else "/drone" if idx == 0 else f"/drone_{idx}"
    
    drone = RobotDroneAutel(
        scene_manager=world.get_scene_manager(),
        namespace=namespace,
        prim_path=prim_path,
        color_scheme_id=idx,
    )
    
    cfg_lfr = CfgLidar(name="lfr", prim_path=prim_path + "/lfr", output_size=(352, 120), 
                       quat=(1, 0, 0, 0), config_file_name="autel_perception_120x352")
    cfg_ubd = CfgLidar(name="ubd", prim_path=prim_path + "/ubd", output_size=(352, 120),
                       quat=(0, 0, 0.7071067811865476, 0.7071067811865476), config_file_name="autel_perception_120x352")
    
    node, pubs, subs, srvs = setup_ros(namespace, ctx=drone)
    
    drone.ros_node = node
    drone.pubs = pubs
    drone.subs = subs
    drone.srvs = srvs
    drone.lidar_list = [LidarOmni(cfg_lidar=cfg_lfr), LidarOmni(cfg_lidar=cfg_ubd)]
    
    return drone


def create_car_objects(scene_manager, map_semantic):
    scale = [2, 5, 1.0]
    cubes_config = {
        "car0": {
            "shape_type": "cuboid",
            "prim_path": "/World/car0",
            "scale": scale,
            "name": "car0_LKN1111_pink",
            "position": [11.6, 3.5, 0],
            "color": [255, 255, 255],
            "entity_type": "visual",
        },
        "car1": {
            "shape_type": "cuboid",
            "prim_path": "/World/car1",
            "scale": scale,
            "name": "car1_ZN3J3W_blue",
            "position": [0.3, 3.5, 0],
            "color": [255, 255, 255],
            "entity_type": "visual",
        },
        "car2": {
            "shape_type": "cuboid",
            "prim_path": "/World/car2",
            "scale": scale,
            "name": "car2_JN3839_yellow",
            "position": [-13.2, 3.5, 0],
            "color": [255, 255, 255],
            "entity_type": "visual",
        },
        "car3": {
            "shape_type": "cuboid",
            "prim_path": "/World/car3",
            "name": "car3_QBZ666_black",
            "scale": scale,
            "position": [-7.1, 10, 0],
            "color": [255, 255, 255],
            "entity_type": "visual",
        },
        "car4": {
            "shape_type": "cuboid",
            "prim_path": "/World/car4",
            "name": "car4_PN3S39_white",
            "scale": scale,
            "position": [-0.9, 30, 0],
            "orientation": [0.707, 0, 0, 0.707],
            "color": [255, 255, 255],
            "entity_type": "visual",
        },
    }

    for config in cubes_config.values():
        result = scene_manager.create_shape_unified(**config)
        if result.get("status") == "success":
            map_semantic.add_semantic(prim_path=result.get("result"), semantic_label="car")


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
    create_car_objects(scene_manager, semantic_map)

    result = scene_manager.add_camera(translation=[1, 4, 2], orientation=[1, 0, 0, 0])
    camera_result = result.get("result")

    # Import after simulation_app is started by Server
    from simulation_utils.simulation_core import run_simulation_loop_multi
    from simulation_utils.ros_bridge import setup_ros
    drone_ctxs = [build_drone_ctx(ns, idx, world, setup_ros) 
                  for idx, ns in enumerate(config_manager.get("namespace"))]

    run_simulation_loop_multi(simulation_app, drone_ctxs, camera_result.get("camera_instance"),
                              camera_result.get("prim_path"), semantic_map)


if __name__ == "__main__":
    main()
