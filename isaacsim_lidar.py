# =============================================================================
# Isaac Sim LiDAR Module - LiDAR Simulation and ROS Integration
# =============================================================================
#
# This module provides LiDAR simulation capabilities within Isaac Sim,
# integrating with ROS2 for sensor data publishing and multi-robot
# coordination.
#
# =============================================================================
try:
    import pydevd_pycharm

    pydevd_pycharm.settrace(
        "localhost", port=12345, stdout_to_server=True, stderr_to_server=True
    )
except Exception as e:
    print(f"no pydevd found: {repr(e)}")
##########################################################################################################################
from physics_engine.isaacsim_simulation_app import start_isaacsim_simulation_app

simulation_app = start_isaacsim_simulation_app()

# ROS2 imports
import rclpy

rclpy.init(args=None)
##########################################################################################################################

# Local project imports
from config.config_manager import config_manager
from log.log_manager import LogManager
from map.map_semantic_map import MapSemantic
from robot.robot_drone_autel import RobotDroneAutel
from scene.scene_manager import SceneManager
from simulation_utils.ros_bridge import setup_ros
from simulation_utils.simulation_core import run_simulation_loop_multi

logger = LogManager.get_logger(__name__)

# Derive list of namespaces --------------------------------------------------
if config_manager.get("namespace"):
    namespace_list = config_manager.get("namespace")
    print(f"Running LiDAR sim with namespaces: {namespace_list}")


#############################################################


def build_drone_ctx(namespace: str, idx: int, scene_manager):
    """Create prim, ROS node and sensor annotators for one UAV."""

    prim_path = (
        f"/{namespace}/drone"
        if namespace
        else "/drone" if idx == 0 else f"/drone_{idx}"
    )

    from robot.sensor.lidar.lidar_omni import LidarOmni
    from robot.sensor.lidar.cfg_lidar import CfgLidar

    cfg_lfr = CfgLidar(
        name="lfr",
        prim_path=prim_path + "/lfr",
        output_size=(352, 120),
        quat=(1, 0, 0, 0),
        config_file_name="autel_perception_120x352",
    )

    cfg_ubd = CfgLidar(
        name="ubd",
        prim_path=prim_path + "/ubd",
        output_size=(352, 120),
        quat=(0, 0, 0.7071067811865476, 0.7071067811865476),
        config_file_name="autel_perception_120x352",
    )

    lidar_lfr = LidarOmni(cfg_lidar=cfg_lfr)
    lidar_ubd = LidarOmni(cfg_lidar=cfg_ubd)

    partial_ctx = RobotDroneAutel(
        scene_manager=scene_manager,
        namespace=namespace,
        prim_path=prim_path,
        color_scheme_id=idx,
    )

    # ROS ---------------------------------------------------------------
    node, pubs, subs, srvs = setup_ros(namespace, ctx=partial_ctx)

    # Complete the context with all the ROS and LiDAR information
    partial_ctx.ros_node = node
    partial_ctx.pubs = pubs
    partial_ctx.subs = subs
    partial_ctx.srvs = srvs
    partial_ctx.lidar_list = [lidar_lfr, lidar_ubd]
    # partial_ctx.custom_step_fn = [lidar_lfr.wrapper(size=[352,120]), lidar_ubd.wrapper(size=[352,120])]

    print(f"Drone {namespace} set up with callbacks for:")
    print(
        f"  - EntityState entity names: {[namespace, f'{namespace}/quadrotor', f'{namespace}_quadrotor']}"
    )
    print(
        f"  - LinkState link names: {[f'{namespace}::base_link', f'{namespace}/quadrotor::base_link', f'{namespace}_quadrotor::base_link']}"
    )
    return partial_ctx


def create_car_objects(scene_manager: SceneManager, map_semantic: MapSemantic) -> list:
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

    created_prim_paths = []
    logger.info(
        f"All semantics in scene:{map_semantic.count_semantics_in_scene().get('result')}"
    )

    for cube_name, config in cubes_config.items():
        creation_result = scene_manager.create_shape_unified(**config)

        if creation_result.get("status") == "success":
            prim_path = creation_result.get("result")
            created_prim_paths.append(prim_path)

            # Add semantic label
            semantic_result = map_semantic.add_semantic(
                prim_path=prim_path, semantic_label="car"
            )
            logger.info(semantic_result)

    return created_prim_paths


def main():  # Needed in build_drone_ctx

    from containers import get_container

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
    # skill_manager = container.skill_manager()
    viewport_manager = container.viewport_manager()
    world = container.world()
    env = container.env()
    env.simulation_app = simulation_app
    # setup_simulation()
    ros_manager.start()

    WORLD_USD_PATH = config_manager.get("world_usd_path")
    scene_manager.load_scene(usd_path=WORLD_USD_PATH, prim_path_root="/World/Scene")

    created_prim_paths = create_car_objects(scene_manager, semantic_map)
    # print("All prims with 'car' label:", created_prim_paths)
    print(semantic_map.count_semantics_in_scene().get("result"))

    # Create and initialize semantic camera
    result = scene_manager.add_camera(translation=[1, 4, 2], orientation=[1, 0, 0, 0])

    semantic_camera = result.get("result").get("camera_instance")
    semantic_camera_prim_path = result.get("result").get("prim_path")

    drone_ctxs: list[RobotDroneAutel] = []
    for idx, ns in enumerate(namespace_list):
        drone_ctxs.append(build_drone_ctx(ns, idx, scene_manager=scene_manager))

    # ---------------- Run simulation -----------------------------------
    run_simulation_loop_multi(
        simulation_app,
        drone_ctxs,
        semantic_camera,
        semantic_camera_prim_path,
        semantic_map,
    )

    # Cleanup copied lidar config files ---------------------------------
    # for ctx in drone_ctxs:
    #     cfg = getattr(ctx, "_lidar_cfg_path", None)
    #     if cfg and os.path.exists(cfg):
    #         os.remove(cfg)


if __name__ == "__main__":
    main()
