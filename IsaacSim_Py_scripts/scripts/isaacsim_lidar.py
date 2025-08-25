#!/usr/bin/env python3

import argparse

# Parse arguments *before* SimulationApp initialization
# ------------------------------- CLI ---------------------------------------
parser = argparse.ArgumentParser(
    description="Setup Isaac Sim environment for Planner (LiDAR)."
)
parser.add_argument(
    "--world",
    type=str,
    default="default_world",
    help="Name or path of the world/scene to load.",
)
parser.add_argument("--gui", action="store_true", help="Run simulation in GUI mode.")
group = parser.add_mutually_exclusive_group()
group.add_argument(
    "--namespace",
    type=str,
    default="",
    help="(Deprecated) Single ROS namespace for topics and services.",
)
group.add_argument(
    "--namespaces",
    type=str,
    default="",
    help="Comma-separated list of namespaces for multi-UAV simulation.",
)
# Use parse_known_args to ignore extra args potentially passed by ROS launch
args, unknown = parser.parse_known_args()

# Derive list of namespaces --------------------------------------------------
if args.namespaces:
    namespace_list = [ns.strip() for ns in args.namespaces.split(",") if ns.strip()]
else:
    # Fallback to legacy --namespace option
    namespace_list = [args.namespace]

print(f"Running LiDAR sim with namespaces: {namespace_list}")

# Import initialization module first
from isaacsim_init import initialize_simulation_app

# Initialize SimulationApp globally using parsed arguments
g_simulation_app = initialize_simulation_app(args)

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
    print(f"  - LinkState link names: {[f'{namespace}::base_link', f'{namespace}/quadrotor::base_link', f'{namespace}_quadrotor::base_link']}")
    return partial_ctx


def main():
    rendering_hz = 10.0
    global curr_stage  # Needed in build_drone_ctx

    world, curr_stage = create_sim_environment(
        args.world, rendering_hz, g_simulation_app
    )

    drone_ctxs: list[DroneSimCtx] = []
    for idx, ns in enumerate(namespace_list):
        drone_ctxs.append(build_drone_ctx(ns, idx))

    # ---------------- Run simulation -----------------------------------
    run_simulation_loop_multi(
        g_simulation_app, world, curr_stage, drone_ctxs
    )

    # Cleanup copied lidar config files ---------------------------------
    for ctx in drone_ctxs:
        cfg = getattr(ctx, "_lidar_cfg_path", None)
        if cfg and os.path.exists(cfg):
            os.remove(cfg)


if __name__ == "__main__":
    main()
