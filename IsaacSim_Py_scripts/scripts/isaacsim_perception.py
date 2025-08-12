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
parser.add_argument("--robot_model", type=str, default="modele", help="robot model.")
parser.add_argument("--trt_engine_file", type=str, default="", help="TRT engine file.")
parser.add_argument(
    "--trt_feat_coords_file", type=str, default="", help="TRT feat coords file."
)
parser.add_argument(
    "--trt_overlap_mask_file", type=str, default="", help="TRT overlap mask file."
)
# Use parse_known_args to ignore extra args potentially passed by ROS launch
args, unknown = parser.parse_known_args()

# Import initialization module first
from isaacsim_init import initialize_simulation_app

# Initialize SimulationApp globally using parsed arguments
g_simulation_app = initialize_simulation_app(args)

# Import common functionality after simulation app is initialized
from isaacsim_common import (
    create_sim_environment,
    add_drone_body,
    run_simulation_loop,
    g_drone_prim_path,
)

import carb
from pxr import Gf, UsdGeom

from isaacsim.sensors.camera import CameraView

from minimal_camera import (
    MinimalCamera,
)  # a stripped down version of isaacsim.sensors.camera

from collections import namedtuple
from imgs2depth_trt import FisheyeTRTProcessor

CamRigComponent = namedtuple(
    "CamRigComponent", ["pos", "wxyz", "cam_type", "render_types"]
)


def gen_cam_rig_components_mde():
    """Generates the camera rig components for the drone."""
    rig_comps = {}

    rig_comps["HD_LT"] = CamRigComponent(
        pos=[0.09283, 0.03483, 0.00139],
        wxyz=[0.52991553, 0.756797808, -0.313475916, -0.219498199],
        cam_type="fe_mde",
        render_types=["rgb"],
    )  # Head Left

    rig_comps["HD_RT"] = CamRigComponent(
        pos=[0.09283, -0.03483, 0.00139],
        wxyz=[0.219498199, 0.313475916, -0.756797808, -0.52991553],
        cam_type="fe_mde",
        render_types=["rgb"],
    )  # Head Right

    # HD_UP with 18 deg pitch
    # rig_comps["HD_UP"] = CamRigComponent(
    #     pos=[0.05585, 0, 0.01905],
    #     wxyz=[0.110615871, 0.698401123, -0.698401123, -0.110615871],
    #     cam_type="fe_mde",
    #     render_types=["rgb"],
    # )  # Head Up

    # HD_UP with 12 deg pitch
    rig_comps["HD_UP"] = CamRigComponent(
        pos=[0.05585, 0, 0.01905],
        wxyz=[0.0739127852, 0.703233176, -0.703233176, -0.0739127852],
        cam_type="fe_mde",
        render_types=["rgb"],
    )  # Head Up

    rig_comps["RR_LT"] = CamRigComponent(
        pos=[-0.12283, -0.03083, 0.00739],
        wxyz=[-0.219498199, -0.313475916, -0.756797808, -0.52991553],
        cam_type="fe_mde",
        render_types=["rgb"],
    )  # Rear Left

    rig_comps["RR_RT"] = CamRigComponent(
        pos=[-0.12283, 0.03083, 0.00739],
        wxyz=[0.52991553, 0.756797808, 0.313475916, 0.219498199],
        cam_type="fe_mde",
        render_types=["rgb"],
    )  # Rear Right

    rig_comps["DN_LT"] = CamRigComponent(
        pos=[-0.111, 0.0245, -0.08284],
        wxyz=[0.707106781, 1.11022302e-16, 0, -0.707106781],
        cam_type="fe_mde",
        render_types=["rgb"],
    )  # Down Left

    rig_comps["DN_RT"] = CamRigComponent(
        pos=[-0.111, -0.0245, -0.08284],
        wxyz=[0.707106781, 1.11022302e-16, 0, -0.707106781],
        cam_type="fe_mde",
        render_types=["rgb"],
    )  # Down Right

    return rig_comps


def add_drone_cameras_tiled(
    curr_stage, drone_prim_path, rig_comps: list[CamRigComponent]
):
    """Adds cameras to the drone and sets up their publishing graphs."""
    # assumed parameters
    pixel_size_um = 3
    f_stop = 0.0  # a value of 0 disables the depth of field effect
    focus_distance = 0.0  # in meters, the distance from the camera to the object plane

    # from eucm model
    width = 1024
    height = 1024
    fx = 314.0
    fy = 314.0
    cx = (width - 1) / 2
    cy = (height - 1) / 2
    alpha = 0.6123
    beta = 1.0335

    # Kannala Brandt Distortion Coefficients (obtained from EUCM model)
    distortion_coefficients = [
        0.0169273,
        -0.00120855,
        -0.000609953,
        -5.50584e-05,
    ]

    # camera parameters
    horizontal_aperture = pixel_size_um * 1e-3 * width
    vertical_aperture = pixel_size_um * 1e-3 * height
    focal_length_x = fx * pixel_size_um * 1e-3
    focal_length_y = fy * pixel_size_um * 1e-3
    focal_length = (focal_length_x + focal_length_y) / 2  # in mm

    for cam_prim_name, comp in rig_comps.items():
        cam_prim_path = drone_prim_path + "/Cameras/" + cam_prim_name
        cam_prim = UsdGeom.Camera.Define(curr_stage, cam_prim_path)
        cam_prim.AddTranslateOp().Set(Gf.Vec3d(comp.pos))
        cam_prim.AddOrientOp().Set(Gf.Quatf(*comp.wxyz))

        camera = MinimalCamera(cam_prim_path, resolution=(width, height))

        # Set the camera parameters, note the unit conversion between Isaac Sim sensor and Kit
        camera.set_focal_length(focal_length / 10.0)
        camera.set_focus_distance(focus_distance)
        camera.set_lens_aperture(f_stop * 100.0)
        camera.set_horizontal_aperture(horizontal_aperture / 10.0)
        camera.set_vertical_aperture(vertical_aperture / 10.0)

        camera.set_clipping_range(0.05, 1.0e5)

        # Set the distortion coefficients
        camera.set_projection_type("fisheyePolynomial")
        camera.set_kannala_brandt_properties(
            width, height, cx, cy, None, distortion_coefficients
        )

    # Create tiled camera using the cameraview
    camera_view = CameraView(
        prim_paths_expr=g_drone_prim_path + "/Cameras/*",
        camera_resolution=(width, height),
        output_annotators=comp.render_types,
    )
    return camera_view


def create_perception_step_wrapper(camera_view, fisheye_trt_processor):
    """Create a wrapper function for perception simulation step that captures all required arguments."""

    # Use a closure variable to store rgb_data across multiple calls
    rgb_data = None

    @carb.profiler.profile
    def perception_step_wrapper():
        nonlocal rgb_data
        # Process camera data
        if camera_view is None:
            return None

        rgb_data = camera_view.get_rgb(rgb_data)

        if rgb_data is None:
            return None

        # print(f"{rgb_data.shape=}")
        depth = fisheye_trt_processor.process_images(rgb_data)
        return depth

    return perception_step_wrapper


def main():
    """Main function to orchestrate the simulation setup and execution."""
    # Arguments are parsed globally before SimulationApp initializatin
    rendering_hz = 10.0
    world, curr_stage = create_sim_environment(
        args.world, rendering_hz, g_simulation_app
    )
    drone_prim = add_drone_body(curr_stage)

    # Setup camera rig configuration
    if args.robot_model == "modele":
        cam_rig_components = gen_cam_rig_components_mde()
    else:
        print("Incorrect model provided:", args.robot_model)

    camera_view = add_drone_cameras_tiled(
        curr_stage, g_drone_prim_path, cam_rig_components
    )

    # Initialize the fisheye TRT processor
    fisheye_trt_processor = FisheyeTRTProcessor(
        engine_file=args.trt_engine_file,
        feat_coords_file=args.trt_feat_coords_file,
        overlap_mask_file=args.trt_overlap_mask_file,
    )

    # Create wrapper function with captured arguments
    perception_step_wrapper = create_perception_step_wrapper(
        camera_view, fisheye_trt_processor
    )

    run_simulation_loop(
        g_simulation_app, world, curr_stage, drone_prim, perception_step_wrapper
    )


if __name__ == "__main__":
    main()
