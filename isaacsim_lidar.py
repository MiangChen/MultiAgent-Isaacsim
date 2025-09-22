#!/usr/bin/env python3

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


g_simulation_app = pre_initialize()
#############################################################3
from config.config_manager import config_manager

# Derive list of namespaces --------------------------------------------------
if config_manager.get("namespace"):
    namespace_list = config_manager.get("namespace")

print(f"Running LiDAR sim with namespaces: {namespace_list}")

# Import initialization module first
from isaacsim_init import initialize_simulation_app

# Initialize SimulationApp globally using parsed arguments
# g_simulation_app = initialize_simulation_app(args)

# Common helpers
from isaacsim_common import (
    create_sim_environment,
    add_drone_body,
    setup_ros,
    run_simulation_loop_multi,
    DroneSimCtx,
)

import omni
import carb
import numpy as np

from pxr import Gf

import os
import shutil
from isaacsim.core.utils.extensions import get_extension_path_from_name

from scene.scene_manager import SceneManager
from map.map_semantic_map import MapSemantic


def copy_lidar_config(lidar_config):
    """Copy the lidar config from the current directory to the extension directory."""

    src_file_path = os.path.join(os.path.dirname(__file__), lidar_config + ".json")
    dst_file_path = os.path.abspath(
        os.path.join(
            get_extension_path_from_name("isaacsim.sensors.rtx"),
            "data/lidar_configs/" + lidar_config + ".json",
        )
    )
    shutil.copyfile(
        src_file_path,
        dst_file_path,
    )
    return dst_file_path


def add_drone_lidar(drone_prim_path, lidar_config):
    """Adds lidar sensors to the drone."""

    # 1. Create The Camera
    _, sensor_lfr = omni.kit.commands.execute(
        "IsaacSensorCreateRtxLidar",
        path=drone_prim_path + "/Lidar/lfr",
        parent=None,
        config=lidar_config,
        translation=(0, 0, 0),
        orientation=Gf.Quatd(1, 0, 0, 0),
    )
    # 2. Create and Attach a render product to the camera
    render_product_lfr = omni.replicator.core.create.render_product(
        sensor_lfr.GetPath(), [1, 1]
    )

    # 3. Create Annotator to read the data from with annotator.get_data()
    annotator_lfr = omni.replicator.core.AnnotatorRegistry.get_annotator(
        "RtxSensorCpuIsaacReadRTXLidarData"
    )
    annotator_lfr.attach(render_product_lfr)

    # 1. Create The Camera
    _, sensor_ubd = omni.kit.commands.execute(
        "IsaacSensorCreateRtxLidar",
        path=drone_prim_path + "/Lidar/ubd",
        parent=None,
        config=lidar_config,
        translation=(0, 0, 0),
        orientation=Gf.Quatd(0, 0, 0.7071067811865476, 0.7071067811865476),
    )
    # 2. Create and Attach a render product to the camera
    render_product_ubd = omni.replicator.core.create.render_product(
        sensor_ubd.GetPath(), [1, 1]
    )

    # 3. Create Annotator to read the data from with annotator.get_data()
    annotator_ubd = omni.replicator.core.AnnotatorRegistry.get_annotator(
        "RtxSensorCpuIsaacReadRTXLidarData"
    )
    annotator_ubd.attach(render_product_ubd)

    return annotator_lfr, annotator_ubd


def create_lidar_step_wrapper(lidar_annotators):
    """Create a wrapper function for lidar simulation step that captures all required arguments."""

    depths = np.empty((2, 352, 120), dtype=np.float32)

    @carb.profiler.profile
    def lidar_step_wrapper():
        nonlocal depths

        depths.fill(1000)
        # Process lidar data
        for i, annotator in enumerate(lidar_annotators):
            data = annotator.get_data()
            # print(f"Lidar {i}\n{data}")
            lidar_depths = data["distances"]
            emitter_ids = data["emitterIds"]
            depths_flat = depths[i].reshape((depths.shape[1] * depths.shape[2]))
            depths_flat[emitter_ids] = lidar_depths
        return depths

    return lidar_step_wrapper


def build_drone_ctx(namespace: str, idx: int):
    """Create prim, ROS node and sensor annotators for one UAV."""

    prim_path = f"/{namespace}/drone" if namespace else "/drone" if idx == 0 else f"/drone_{idx}"

    print(f"Adding drone body to {prim_path} with color scheme {idx}")
    drone_prim = add_drone_body(curr_stage, prim_path, color_scheme=idx)
    print(f"Drone prim: {drone_prim}")

    # Create a partial context for ROS setup
    partial_ctx = DroneSimCtx(
        namespace=namespace,
        prim_path=prim_path,
        ros_node=None,  # Will be set below
        drone_prim=drone_prim,
    )

    # ROS ---------------------------------------------------------------
    node, pubs, subs, srvs = setup_ros(namespace, ctx=partial_ctx)

    # LiDAR -------------------------------------------------------------
    lidar_config = "autel_perception_120x352"
    lidar_config_path = copy_lidar_config(lidar_config)
    lidar_annotators = add_drone_lidar(prim_path, lidar_config)
    lidar_step_wrapper = create_lidar_step_wrapper(lidar_annotators)

    # Complete the context with all the ROS and LiDAR information
    partial_ctx.ros_node = node
    partial_ctx.pubs = pubs
    partial_ctx.subs = subs
    partial_ctx.srvs = srvs
    partial_ctx.custom_step_fn = lidar_step_wrapper

    # Store lidar config path for cleanup
    partial_ctx._lidar_cfg_path = lidar_config_path  # type: ignore[attr-defined]

    print(f"Drone {namespace} set up with callbacks for:")
    print(f"  - EntityState entity names: {[namespace, f'{namespace}/quadrotor', f'{namespace}_quadrotor']}")
    print(
        f"  - LinkState link names: {[f'{namespace}::base_link', f'{namespace}/quadrotor::base_link', f'{namespace}_quadrotor::base_link']}")
    return partial_ctx


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
    global curr_stage  # Needed in build_drone_ctx

    scene_manager = SceneManager()
    world, curr_stage = create_sim_environment(
        config_manager.get("world").get("path"), rendering_hz=config_manager.get("rendering_hz"),
        simulation_app=g_simulation_app,
    )

    created_prim_paths = create_car_objects(scene_manager)
    print("All prims with 'car' label:", created_prim_paths)
    print(scene_manager.count_semantics_in_scene().get("result"))

    # Create and initialize semantic camera
    result = scene_manager.add_camera(
        position=[1, 4, 2],
        quat=scene_manager.euler_to_quaternion(roll=90),
        prim_path="/World/semantic_camera",
    )

    semantic_map = MapSemantic()
    semantic_camera = result.get("result").get("camera_instance")
    semantic_camera_prim_path = result.get("result").get("prim_path")

    drone_ctxs: list[DroneSimCtx] = []
    for idx, ns in enumerate(namespace_list):
        drone_ctxs.append(build_drone_ctx(ns, idx))

    # ---------------- Run simulation -----------------------------------
    run_simulation_loop_multi(
        g_simulation_app, world, curr_stage, drone_ctxs, semantic_camera, semantic_camera_prim_path, semantic_map
    )

    # Cleanup copied lidar config files ---------------------------------
    for ctx in drone_ctxs:
        cfg = getattr(ctx, "_lidar_cfg_path", None)
        if cfg and os.path.exists(cfg):
            os.remove(cfg)


if __name__ == "__main__":
    main()

