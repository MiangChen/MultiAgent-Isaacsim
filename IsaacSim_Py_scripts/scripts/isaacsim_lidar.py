#!/usr/bin/env python3

import argparse

# Parse arguments *before* SimulationApp initialization
parser = argparse.ArgumentParser(description="Setup Isaac Sim environment for Planner.")
parser.add_argument(
    "--world",
    type=str,
    default="default_world",
    help="Name or path of the world/scene to load.",
)
parser.add_argument("--gui", action="store_true", help="Run simulation in GUI mode.")
# Use parse_known_args to ignore extra args potentially passed by ROS launch
args, unknown = parser.parse_known_args()

# Import initialization module first
from isaacsim_init import initialize_simulation_app



# Initialize SimulationApp globally using parsed arguments
g_simulation_app = initialize_simulation_app(args)

import carb

carb.settings.get_settings().set(
    "/presitent/isaac/asset_root/default",
    f"/home/ubuntu/isaacsim_assets/Assets/Isaac/4.5",
)

# Import common functionality after simulation app is initialized
from isaacsim.core.utils.extensions import get_extension_path_from_name

from isaacsim_common import (
    create_sim_environment,
    add_drone_body,
    run_simulation_loop,
    g_drone_prim_path,
)

import omni
import carb
import numpy as np

from pxr import Gf

import os
import shutil



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


def main():
    rendering_hz = 10.0
    world, curr_stage = create_sim_environment(
        args.world, rendering_hz, g_simulation_app
    )
    drone_prim = add_drone_body(curr_stage)

    lidar_config = "autel_perception_120x352"
    lidar_config_path = copy_lidar_config(lidar_config)
    lidar_annotators = add_drone_lidar(g_drone_prim_path, lidar_config)

    # Create wrapper function with captured arguments
    lidar_step_wrapper = create_lidar_step_wrapper(lidar_annotators)

    run_simulation_loop(
        g_simulation_app, world, curr_stage, drone_prim, lidar_step_wrapper
    )

    # Delete the copied lidar config
    os.remove(lidar_config_path)


if __name__ == "__main__":
    main()
