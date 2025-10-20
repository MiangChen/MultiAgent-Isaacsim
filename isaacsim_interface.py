#!/usr/bin/env python
# Author: Subhransu Mishra

import sys
import argparse
import os

print("isaacsim_interface args: ", sys.argv)
# Parse arguments *before* SimulationApp initialization
parser = argparse.ArgumentParser(description="Setup Isaac Sim environment for Planner.")
parser.add_argument(
    "--world",
    type=str,
    default="default_world",
    help="Name or path of the world/scene to load.",
)
parser.add_argument("--gui", action="store_true", help="Run simulation in GUI mode.")
parser.add_argument("--robot_model", type=str, default="modelx", help="robot model.")
# Use parse_known_args to ignore extra args potentially passed by ROS launch
args, unknown = parser.parse_known_args()

# Initialize SimulationApp globally using parsed arguments
from physics_engine.isaacsim_utils import SimulationApp

CONFIG = {"renderer": "RaytracedLighting", "headless": not args.gui}
print(f"Initializing Isaac Sim with headless={not args.gui}")
g_simulation_app = SimulationApp(CONFIG)

# Set some settings to reduce the GPU/CPU usage
g_simulation_app.set_setting("/rtx/ecoMode/enabled", False)

g_simulation_app.set_setting("/rtx/directLighting/enabled", False)
g_simulation_app.set_setting("/rtx/indirectDiffuse/enabled", False)
g_simulation_app.set_setting("/rtx/sceneDb/ambientLightIntensity", 3.0)

g_simulation_app.set_setting("/rtx/reflections/enabled", False)
g_simulation_app.set_setting("/rtx/translucency/enabled", False)
g_simulation_app.set_setting("/rtx/raytracing/subsurface/enabled", False)
g_simulation_app.set_setting("/rtx/caustics/enabled", False)

g_simulation_app.set_setting("/persistent/physics/numThreads", 0)

if not args.gui:
    # disable the viewport if not using any gui
    from omni.kit.viewport.utility import get_active_viewport

    get_active_viewport().set_updates_enabled(False)

import carb

import math

from typing import Optional

import omni
import omni.graph.core as og
import usdrt.Sdf

from physics_engine.isaacsim_utils import (
    SimulationContext,
    World,
    VisualCuboid,
    VisualCylinder,
    VisualSphere,
    extensions,
    stage,
    get_assets_root_path,
)

from minimal_camera import (
    MinimalCamera,
)  # a stripped down version of isaacsim.sensors.camera

from physics_engine.pxr_utils import Gf, UsdGeom, Sdf, Usd

import time
import numpy as np
from collections import namedtuple
import threading

# enable ROS bridge extension and then import ros modules
extensions.enable_extension("isaacsim.ros1.bridge")
import rosgraph
import rospy

# Add imports for Gazebo and standard services
from gazebo_msgs.msg import LinkState
from gazebo_msgs.srv import SetLinkState, SetLinkStateResponse
from gazebo_msgs.srv import SpawnModel, SpawnModelResponse
from std_srvs.srv import Empty, EmptyResponse
from diagnostic_msgs.srv import AddDiagnostics, AddDiagnosticsResponse

# Global variables
g_drone_prim_path = "/drone"
g_is_drone_des_pose_changed = False
g_drone_des_pose = None
g_is_gimbal_des_pose_changed = False
g_gimbal_des_pose = None
pose_lock = threading.Lock()
g_pending_spawn_requests = []
spawn_lock = threading.Lock()
g_pending_move_requests = []
move_lock = threading.Lock()

wxyz_front = [0.5, 0.5, -0.5, -0.5]
wxyz_back = [0.5, 0.5, 0.5, 0.5]
wxyz_top = [0.0, 0.707106781, -0.707106781, 0.0]
wxyz_bottom = [0.707106781, 0.0, 0.0, -0.707106781]

CamRigComponent = namedtuple(
    "CamRigComponent", ["pos", "wxyz", "cam_type", "render_types"]
)


def export_prim_to_layer(
    prim: Usd.Prim, flatten=True, include_session_layer=True
) -> Optional[Sdf.Layer]:
    """
    Creates a temporary layer with only the given prim in it

    Parameters
    ----------
    prim: Usd.Prim
        The usd object we wish to export by itself into its own layer
    flatten: bool
        If True, then the returned layer will have all composition arcs
        flattened. If False, then the returned layer will contain a reference
        to the original prim.
    include_session_layer: bool
        If True, then include changes from the session layer. If this is
        False, and the prim is ONLY defined in the session layer, then None
        will be returned.

    Note that no parents are included, and xforms are not "flattened" (even if
    `flatten` is True - `flatten` refers to USD composition arcs, not parent
    hierarchy or xforms).
    """
    orig_stage = prim.GetStage()
    orig_root_layer = orig_stage.GetRootLayer()
    orig_session_layer = orig_stage.GetSessionLayer()
    if orig_session_layer and not include_session_layer:
        # Make sure that the prim still exists if we exclude the session layer
        stage_no_session = Usd.Stage.Open(orig_root_layer, sessionLayer=None)
        if not stage_no_session.GetPrimAtPath(prim.GetPrimPath()):
            return None

        orig_session_layer = None

    # If there is a session layer, to get an EXACT copy including possible
    # modifications by the session layer, we need to create a new "copy" layer,
    # with a layer stack composed of the orig_stage's session layer
    # and root layer
    if not orig_session_layer:
        copy_layer = orig_root_layer
    else:
        copy_layer = Sdf.Layer.CreateAnonymous()
        copy_layer.subLayerPaths.append(orig_session_layer.identifier)
        copy_layer.subLayerPaths.append(orig_root_layer.identifier)

    # Now create a "solo" stage, with only our prim (from the copy layer)
    # referenced in
    solo_stage = Usd.Stage.CreateInMemory()
    solo_prim_path = Sdf.Path(f"/{prim.GetName()}")
    solo_prim = solo_stage.DefinePrim(solo_prim_path)
    solo_prim.GetReferences().AddReference(copy_layer.identifier, prim.GetPrimPath())
    solo_stage.SetDefaultPrim(solo_prim)

    if flatten:
        return solo_stage.Flatten()
    else:
        return solo_stage.GetRootLayer()


class DronePose:
    def __init__(self, pos=Gf.Vec3d(0, 0, 0), quat=Gf.Quatf.GetIdentity()):
        self.pos = pos
        self.quat = quat


def create_sim_environment(world_usd_path, rendering_hz):
    """Sets up the simulation environment, checks ROS, finds assets, and adds a ground plane."""
    global g_simulation_app
    # check if rosmaster node is running
    g_simulation_app.update()
    if not rosgraph.is_master_online():
        carb.log_error("Please run roscore before executing this script")
        g_simulation_app.close()
        sys.exit()

    world = World(
        physics_dt=1.0 / rendering_hz,
        rendering_dt=1.0 / rendering_hz,
        stage_units_in_meters=1.0,
    )
    curr_stage = stage.get_current_stage()

    carb.log_info(
        f"World argument received: {world_usd_path} (currently unused for scene loading)"
    )

    # Locate Isaac Sim assets folder
    assets_root_path = get_assets_root_path()
    if assets_root_path is None and world_usd_path != "":
        carb.log_error("Could not find Isaac Sim assets folder")
        g_simulation_app.close()
        sys.exit()

    # TODO(subhransu): Make assets available offline.
    if world_usd_path == "":
        if assets_root_path is None:
            world.scene.add_ground_plane(
                size=1000, z_position=-0.5, color=np.array([1, 1, 1])
            )
        else:
            world.scene.add_default_ground_plane()
    else:
        # Check if args.world is a direct path that exists
        if os.path.exists(world_usd_path):
            usd_path = world_usd_path
        else:
            # Try using path relative to assets_root_path
            usd_path = os.path.join(assets_root_path, args.world)

        prim_path = "/World"
        stage.add_reference_to_stage(usd_path, prim_path)
        # Wait two frames so that stage starts loading
        g_simulation_app.update()
        g_simulation_app.update()
        print(f"Loading stage from {usd_path}...")
        while stage.is_stage_loading():
            g_simulation_app.update()
        print("Loading Complete")

    return world, curr_stage


def add_drone_body(curr_stage):
    """Creates the drone's main Xform and adds its body components."""
    global g_drone_prim_path
    # Define the parent drone prim (UsdGeom.Xform(Usd.Prim(</drone>)))
    drone_xform = UsdGeom.Xform.Define(curr_stage, g_drone_prim_path)
    drone_xform.AddTranslateOp().Set(Gf.Vec3d(0, 0, 0))
    drone_xform.AddOrientOp().Set(Gf.Quatf.GetIdentity())
    drone_prim = drone_xform.GetPrim()  # Usd.Prim(</drone>)

    # Setup Drone geometry (main body, arms, props)
    MechComponent = namedtuple(
        "MechComponent", ["pos", "scale", "wxyz", "geom_type", "color"]
    )

    mech_comps = {}
    mech_comps["MainBody"] = MechComponent(
        pos=[0.0, 0.0, 0.0],
        scale=[0.1, 0.05, 0.05],
        wxyz=[1.0, 0.0, 0.0, 0.0],
        geom_type="cuboid",
        color=[0.5, 0.5, 0.5],
    )

    # Define a base arm component template
    base_arm = MechComponent(
        pos=None,
        scale=[0.2, 0.02, 0.02],
        wxyz=None,
        geom_type="cuboid",
        color=[0.5, 0.5, 0.5],
    )

    # Create individual arm components
    # wxyz_z_p65 = [math.cos(65 * np.pi / 180), 0, 0, math.sin(65 * np.pi / 180)]
    # wxyz_z_m65 = [math.cos(-65 * np.pi / 180), 0, 0, math.sin(-65 * np.pi / 180)]
    # mech_comps["Arm_FL"] = base_arm._replace(pos=[0.13, 0.1, 0.04], wxyz=wxyz_z_m65)
    # mech_comps["Arm_FR"] = base_arm._replace(pos=[0.13, -0.1, 0.04], wxyz=wxyz_z_p65)
    # mech_comps["Arm_BL"] = base_arm._replace(pos=[-0.13, 0.1, -0.04], wxyz=wxyz_z_p65)
    # mech_comps["Arm_BR"] = base_arm._replace(pos=[-0.13, -0.1, -0.04], wxyz=wxyz_z_m65)

    # Define a base prop component template
    base_prop = MechComponent(
        pos=None,
        scale=[0.14, 0.14, 0.005],
        wxyz=[1.0, 0.0, 0.0, 0.0],
        geom_type="cylinder",
        color=[0.2, 0.2, 0.2],
    )

    # Create individual prop components
    # mech_comps["Prop_FL"] = base_prop._replace(pos=[0.18, 0.19, 0.06])
    # mech_comps["Prop_FR"] = base_prop._replace(pos=[0.18, -0.19, 0.06])
    # mech_comps["Prop_BL"] = base_prop._replace(pos=[-0.18, 0.19, -0.02])
    # mech_comps["Prop_BR"] = base_prop._replace(pos=[-0.18, -0.19, -0.02])

    # Create and add all drone components on the stage
    for prim_name, comp in mech_comps.items():
        if comp.geom_type == "cuboid":
            VisualCuboid(
                prim_path=g_drone_prim_path + "/" + prim_name,
                name=prim_name,
                position=np.array(comp.pos),
                orientation=comp.wxyz,
                scale=np.array(comp.scale),
                color=np.array(comp.color),
            )
        elif comp.geom_type == "cylinder":
            VisualCylinder(
                prim_path=g_drone_prim_path + "/" + prim_name,
                name=prim_name,
                position=np.array(comp.pos),
                orientation=comp.wxyz,
                scale=np.array(comp.scale),
                color=np.array(comp.color),
            )
    return drone_prim


# Creating an on-demand push graph with cameraHelper nodes to generate ROS image publishers
def create_camera_graph(cam_name, cam_prim_path, width, height, render_types):
    """Helper function to create the OmniGraph for a single camera based on render_types."""
    cam_graph_path = "/cam_graph_" + cam_name

    create_nodes_list = [
        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
        ("RunOnce", "isaacsim.core.nodes.OgnIsaacRunOneSimulationFrame"),
        ("RendProd", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
    ]
    connect_list = [
        ("OnPlaybackTick.outputs:tick", "RunOnce.inputs:execIn"),
        ("RunOnce.outputs:step", "RendProd.inputs:execIn"),
    ]
    set_values_list = [
        ("RendProd.inputs:width", width),
        ("RendProd.inputs:height", height),
        ("RendProd.inputs:cameraPrim", [usdrt.Sdf.Path(cam_prim_path)]),
    ]

    if "rgb" in render_types:
        create_nodes_list.append(
            ("CamHelpRGB", "isaacsim.ros1.bridge.ROS1CameraHelper")
        )
        connect_list.extend(
            [
                ("RendProd.outputs:execOut", "CamHelpRGB.inputs:execIn"),
                (
                    "RendProd.outputs:renderProductPath",
                    "CamHelpRGB.inputs:renderProductPath",
                ),
            ]
        )
        set_values_list.extend(
            [
                ("CamHelpRGB.inputs:frameId", cam_name),
                ("CamHelpRGB.inputs:topicName", "/" + cam_name + "/rgb/image_raw"),
                ("CamHelpRGB.inputs:type", "rgb"),
                ("CamHelpRGB.inputs:useSystemTime", 1),
            ]
        )

    if "depth" in render_types:
        create_nodes_list.extend(
            [
                ("CamHelpDepth", "isaacsim.ros1.bridge.ROS1CameraHelper"),
                ("CamHelpDepthInfo", "isaacsim.ros1.bridge.ROS1CameraHelper"),
            ]
        )
        connect_list.extend(
            [
                ("RendProd.outputs:execOut", "CamHelpDepth.inputs:execIn"),
                ("RendProd.outputs:execOut", "CamHelpDepthInfo.inputs:execIn"),
                (
                    "RendProd.outputs:renderProductPath",
                    "CamHelpDepth.inputs:renderProductPath",
                ),
                (
                    "RendProd.outputs:renderProductPath",
                    "CamHelpDepthInfo.inputs:renderProductPath",
                ),
            ]
        )
        set_values_list.extend(
            [
                ("CamHelpDepthInfo.inputs:frameId", cam_name),
                (
                    "CamHelpDepthInfo.inputs:topicName",
                    "/" + cam_name + "/depth/camera_info",
                ),
                ("CamHelpDepthInfo.inputs:type", "camera_info"),
                ("CamHelpDepthInfo.inputs:useSystemTime", 1),
                ("CamHelpDepth.inputs:frameId", cam_name),
                ("CamHelpDepth.inputs:topicName", "/" + cam_name + "/depth/image_raw"),
                ("CamHelpDepth.inputs:type", "depth"),
                ("CamHelpDepth.inputs:useSystemTime", 1),
            ]
        )

    (cam_graph_handle, _, _, _) = og.Controller.edit(
        {
            "graph_path": cam_graph_path,
            "evaluator_name": "push",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
        },
        {
            og.Controller.Keys.CREATE_NODES: create_nodes_list,
            og.Controller.Keys.CONNECT: connect_list,
            og.Controller.Keys.SET_VALUES: set_values_list,
        },
    )
    # Run the ROS Camera graph once to generate ROS image publishers
    og.Controller.evaluate_sync(cam_graph_handle)


def gen_cam_rig_components_mdx():
    """Generates the camera rig components for the drone."""
    rig_comps = {}
    rig_comps["st_front"] = CamRigComponent(
        pos=[0.13, 0.0, 0.04],
        wxyz=wxyz_front,
        cam_type="ph_mdx",
        render_types=["rgb", "depth"],
    )
    rig_comps["st_back"] = CamRigComponent(
        pos=[-0.13, 0.0, 0.04],
        wxyz=wxyz_back,
        cam_type="ph_mdx",
        render_types=["depth"],
    )
    rig_comps["st_top"] = CamRigComponent(
        pos=[0.0, 0.0, 0.06], wxyz=wxyz_top, cam_type="fe_mdx", render_types=["depth"]
    )
    rig_comps["st_bottom"] = CamRigComponent(
        pos=[0.0, 0.0, -0.06],
        wxyz=wxyz_bottom,
        cam_type="fe_mdx",
        render_types=["depth"],
    )
    rig_comps["gimbal"] = CamRigComponent(
        pos=[0.16, 0.0, 0.00],
        wxyz=[1, 0, 0, 0],
        cam_type="gimbal",
        render_types=["rgb"],
    )

    return rig_comps


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

    rig_comps["gimbal"] = CamRigComponent(
        pos=[0.16, 0.0, 0.00],
        wxyz=[1, 0, 0, 0],
        cam_type="gimbal",
        render_types=["rgb"],
    )  # Gimbal

    return rig_comps


def gen_cam_rig_components_mdjjie():
    """Generates the camera rig components for the drone."""
    rig_comps = {}
    rig_comps["st_front"] = CamRigComponent(
        pos=[0.13, 0.0, 0.04],
        wxyz=wxyz_front,
        cam_type="ph_mdjjie",
        render_types=["rgb", "depth"],
    )
    rig_comps["st_back"] = CamRigComponent(
        pos=[-0.13, 0.0, 0.04],
        wxyz=wxyz_back,
        cam_type="ph_mdjjie",
        render_types=["depth"],
    )
    rig_comps["st_top"] = CamRigComponent(
        pos=[0.0, 0.0, 0.06],
        wxyz=wxyz_top,
        cam_type="ph_mdjjie",
        render_types=["depth"],
    )
    rig_comps["st_bottom"] = CamRigComponent(
        pos=[0.0, 0.0, -0.06],
        wxyz=wxyz_bottom,
        cam_type="ph_mdjjie",
        render_types=["depth"],
    )
    rig_comps["gimbal"] = CamRigComponent(
        pos=[0.16, 0.0, 0.00],
        wxyz=[1, 0, 0, 0],
        cam_type="gimbal",
        render_types=["rgb"],
    )

    return rig_comps


def set_cam_prim_params(width, height, hfov_degrees, cam_prim):
    """
    Calculates and sets camera prim attributes based on image dimension and HFOV.

    Args:
        width (int): Width of the camera image in pixels.
        height (int): Height of the camera image in pixels.
        hfov_degrees (float): Desired horizontal field of view in degrees.
        cam_prim (Usd.Prim): The camera prim to configure.
    """

    # 1. Assume a focal length (in mm).
    foc_len = 17.47  # mm

    # 2. Convert HFOV from degrees to radians for math functions
    if hfov_degrees <= 0 or hfov_degrees >= 180:
        raise ValueError("Horizontal FOV must be between 0 and 180 degrees.")
    hfov_radians = math.radians(hfov_degrees)

    # 3. Calculate horizontal aperture (in mm)
    # Formula: horz_aperture = 2 * focal_length * tan(HFOV_radians / 2)
    horz_aperture = 2 * foc_len * math.tan(hfov_radians / 2.0)

    # 4. Calculate vertical aperture (in mm)
    # Formula: vert_aperture = horz_aperture * (image_height / image_width)
    if width <= 0 or height <= 0:
        raise ValueError("Image width and height must be positive.")
    aspect_ratio = float(height) / float(width)
    vert_aperture = horz_aperture * aspect_ratio

    # Set the camera attributes
    cam_prim.GetFocalLengthAttr().Set(foc_len)
    cam_prim.GetHorizontalApertureAttr().Set(horz_aperture)
    cam_prim.GetVerticalApertureAttr().Set(vert_aperture)
    cam_prim.GetProjectionAttr().Set("perspective")
    cam_prim.GetClippingRangeAttr().Set((0.1, 1.0e4))


def add_drone_cameras(curr_stage, rig_comps: list[CamRigComponent]):
    """Adds cameras to the drone and sets up their publishing graphs."""
    global g_drone_prim_path

    gimbal_prim = None
    for cam_prim_name, comp in rig_comps.items():
        cam_prim_path = g_drone_prim_path + "/" + cam_prim_name
        cam_prim = UsdGeom.Camera.Define(curr_stage, cam_prim_path)
        cam_prim.AddTranslateOp().Set(Gf.Vec3d(comp.pos))
        cam_prim.AddOrientOp().Set(Gf.Quatf(*comp.wxyz))
        width = 0
        height = 0
        hfov = 0
        if comp.cam_type == "ph_mdx":
            width = 112
            height = 156
            # hfov = 62
            # TODO(subhransu): Get actual parameter for ph_mdx camera
            cam_prim.GetFocalLengthAttr().Set(17.47)
            cam_prim.GetHorizontalApertureAttr().Set(21.0)
            cam_prim.GetVerticalApertureAttr().Set(29.25)
            cam_prim.GetProjectionAttr().Set("perspective")
            cam_prim.GetClippingRangeAttr().Set((0.1, 1.0e4))

        elif comp.cam_type == "fe_mdx":
            width = 160
            height = 120
            # hfov = 175
            # TODO(subhransu): Get actual parameter for fe_mdx camera
            cam_prim.GetFocalLengthAttr().Set(0.46)
            cam_prim.GetHorizontalApertureAttr().Set(21.0)
            cam_prim.GetVerticalApertureAttr().Set(15.75)
            cam_prim.GetProjectionAttr().Set("perspective")
            cam_prim.GetClippingRangeAttr().Set((0.1, 1.0e4))

        elif comp.cam_type == "fe_mde":
            # assumed parameters
            pixel_size_um = 3
            f_stop = 0.0  # a value of 0 disables the depth of field effect
            focus_distance = (
                0.0  # in meters, the distance from the camera to the object plane
            )

            # from eucm model
            width = 1024
            height = 1024
            fx = 314
            fy = 314
            cx = 511.5
            cy = 511.5
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
            diagonal_fov = None

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
                width, height, cx, cy, diagonal_fov, distortion_coefficients
            )

        elif comp.cam_type == "ph_mdjjie":
            width = 160
            height = 90
            hfov = 90
            set_cam_prim_params(width, height, hfov, cam_prim)

        elif comp.cam_type == "gimbal":
            # cam_prim:UsdGeom.Camera(Usd.Prim(</drone/gimbal>))
            # gimbal_prim:Usd.Prim(</drone/gimbal>)
            gimbal_prim = cam_prim.GetPrim()

            width = 320
            height = 180
            # hfov = 65.9
            # Calculated parameters based on width, height, hfov, and assuming HA=21
            cam_prim.GetFocalLengthAttr().Set(16.2)
            cam_prim.GetHorizontalApertureAttr().Set(21.0)
            cam_prim.GetVerticalApertureAttr().Set(11.8125)
            cam_prim.GetProjectionAttr().Set("perspective")
            cam_prim.GetClippingRangeAttr().Set((0.1, 1.0e4))

        create_camera_graph(
            cam_prim_name, cam_prim_path, width, height, comp.render_types
        )

    g_simulation_app.update()

    return gimbal_prim


def create_clock_graph():
    """Sets up the OmniGraph for publishing simulation time."""
    global g_simulation_app
    g_simulation_app.update()  # Ensure updates before graph setup

    # Setup graph for clock publishing
    clock_topic = "/clock"
    (clock_graph_handle, _, _, _) = og.Controller.edit(
        {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("PublishClock", "isaacsim.ros1.bridge.ROS1PublishClock"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("PublishClock.inputs:topicName", clock_topic),
            ],
        },
    )

    # Run the clock graph once to generate ROS clock publisher
    og.Controller.evaluate_sync(clock_graph_handle)

    g_simulation_app.update()


def callback_gazebo_linkstate_msg(linkstate_msg):
    """ROS callback function to update the desired drone pose."""
    global g_drone_des_pose, g_is_drone_des_pose_changed, pose_lock
    global g_gimbal_des_pose, g_is_gimbal_des_pose_changed
    pos = linkstate_msg.pose.position
    quat = linkstate_msg.pose.orientation
    # Directly update the attributes of the mutable DronePose object
    with pose_lock:
        if linkstate_msg.link_name == "quadrotor::base_link":
            g_drone_des_pose.pos = Gf.Vec3d(pos.x, pos.y, pos.z)
            g_drone_des_pose.quat = Gf.Quatf(quat.w, quat.x, quat.y, quat.z)
            g_is_drone_des_pose_changed = True
        elif linkstate_msg.link_name == "quadrotor::camera_link":
            g_gimbal_des_pose.pos = Gf.Vec3d(pos.x, pos.y, pos.z)
            g_gimbal_des_pose.quat = Gf.Quatf(quat.w, quat.x, quat.y, quat.z)
            g_is_gimbal_des_pose_changed = True
        else:
            print(f"Error: Invalid link name: {linkstate_msg.link_name}")


def extract_geom_info(input_string):
    """
    Extracts geometry type and scale from a string formatted as
    "isaac <geom_type> <scale_x> <scale_y> <scale_z>".

    Args:
        input_string: The string to parse.

    Returns:
        A dictionary containing 'geom_type' (str) and 'scale' (list of floats),
        or None if the string format is invalid.

    Examples:
        >>> extract_geom_info("isaac cube 3.4 1.6 4.5")
        {'geom_type': 'cube', 'scale': [3.4, 1.6, 4.5]}

        >>> extract_geom_info("isaac sphere 1.0 1.0 1.0")
        {'geom_type': 'sphere', 'scale': [1.0, 1.0, 1.0]}
    """
    if input_string == "":
        print("Error: Invalid string: empty string")
        return None

    parts = input_string.split()

    if len(parts) != 5 or parts[0] != "isaac":
        print(f"Error: Invalid string format: {input_string}")
        return None

    geom_type = parts[1]

    try:
        scale = [float(x) for x in parts[2:]]
    except ValueError:
        print(f"Error: Invalid scale values in string: {input_string}")
        return None

    return {"geom_type": geom_type, "scale": scale}


def spawn_model_fun(req):
    """Callback for /gazebo/spawn_sdf_model service. Queues request for main thread."""
    global g_pending_spawn_requests, spawn_lock
    rospy.loginfo(f"spawn req for {req.model_name} with pose: {req.initial_pose}")

    response = SpawnModelResponse()
    response.success = False

    # Extract pose information
    pos = req.initial_pose.position
    quat = req.initial_pose.orientation
    position_np = np.array([pos.x, pos.y, pos.z])
    orientation_np = np.array([quat.w, quat.x, quat.y, quat.z])

    # Check if req.model_xml is a .usd file path
    if req.model_xml.endswith(".usd"):
        # Handle direct .usd file path
        if not os.path.isfile(req.model_xml):
            rospy.logerr(f"USD file not found: {req.model_xml}")
            return response

        # Prepare data for USD model spawning
        spawn_data = {
            "name": req.model_name,
            "pos": position_np,
            "quat": orientation_np,
            "type": "usd_model",
            "usd_path": req.model_xml,
            "prim_path": f"/World/{req.model_name}",  # Assuming /World scope
        }
    else:
        # Use existing isaac schema validation
        geom_info = extract_geom_info(req.model_xml)
        if geom_info is None:
            return response

        # Prepare data for primitive geometry spawning
        spawn_data = {
            "name": req.model_name,
            "pos": position_np,
            "quat": orientation_np,
            "scale": geom_info["scale"],
            "type": geom_info["geom_type"],
            "prim_path": f"/World/{req.model_name}",  # Assuming /World scope
        }

    # Acquire lock, add request to queue, release lock
    with spawn_lock:
        g_pending_spawn_requests.append(spawn_data)

    # send success response
    response.success = True
    return response


def move_model_fun(req):
    """callback for /gazebo/set_link_state service."""
    global g_pending_move_requests, move_lock
    rospy.loginfo(
        f"move req for {req.link_state.link_name} with pose: {req.link_state.pose}"
    )

    # Extract pose information
    pos = req.link_state.pose.position
    quat = req.link_state.pose.orientation
    model_name = req.link_state.link_name[:-6]

    # Prepare data for queuing
    move_data = {
        "name": model_name,
        "pos": Gf.Vec3d(pos.x, pos.y, pos.z),
        "quat": Gf.Quatf(quat.w, Gf.Vec3d(quat.x, quat.y, quat.z)),
        "prim_path": f"/World/{model_name}",
    }

    # Acquire lock, add request to queue, release lock
    with move_lock:
        g_pending_move_requests.append(move_data)

    # send success response
    response = SetLinkStateResponse()
    response.success = True  # Placeholder value
    return response


def pause_physics_fun(req):
    """callback for /gazebo/pause_physics service."""
    rospy.loginfo("Pausing physics")
    return EmptyResponse()


def unpause_physics_fun(req):
    """callback for /gazebo/unpause_physics service."""
    rospy.loginfo("Unpausing physics")
    return EmptyResponse()


def save_world_prim_fun(req):
    """Callback for /world_model/save_world_prim service."""
    rospy.loginfo(f"Save world prim request for filename: {req.load_namespace}")
    response = AddDiagnosticsResponse()
    response.success = False

    if not req.load_namespace:
        response.message = "Error: Filename (load_namespace) cannot be empty."
        rospy.logerr(response.message)
        return response

    try:
        curr_stage = omni.usd.get_context().get_stage()
        if not curr_stage:
            response.message = "Error: Could not get current USD stage."
            rospy.logerr(response.message)
            return response

        world_prim_path = "/World"
        world_prim = curr_stage.GetPrimAtPath(world_prim_path)
        if not world_prim.IsValid():
            response.message = f"Error: Prim at path '{world_prim_path}' is not valid."
            rospy.logerr(response.message)
            return response

        rospy.loginfo(f"Exporting prim: {world_prim.GetPath()}")
        # Assuming export_prim_to_layer is defined and accessible globally or in this class scope
        # And that g_simulation_app is accessible for getting the stage if needed by export_prim_to_layer
        exported_layer = export_prim_to_layer(
            world_prim, flatten=True, include_session_layer=True
        )

        if exported_layer:
            # Ensure the directory exists, if the filename includes a path
            output_dir = os.path.dirname(req.load_namespace)
            if output_dir and not os.path.exists(output_dir):
                os.makedirs(output_dir)
                rospy.loginfo(f"Created directory: {output_dir}")

            exported_layer.Export(req.load_namespace)
            response.success = True
            response.message = (
                f"Successfully saved prim '{world_prim_path}' to '{req.load_namespace}'"
            )
            rospy.loginfo(response.message)
        else:
            response.message = f"Error: Failed to export prim '{world_prim_path}'."
            rospy.logerr(response.message)

    except Exception as e:
        response.message = f"Exception during save_world_prim: {str(e)}"
        rospy.logerr(response.message)

    return response


def move_prim(stage, prim_path, pos, quat):
    geom_prim = stage.GetPrimAtPath(prim_path)
    if not geom_prim:
        print(f"Error: Invalid prim path: {prim_path}")
        return
    if not geom_prim.HasAttribute("xformOp:translate"):
        UsdGeom.Xformable(geom_prim).AddTranslateOp().Set(pos)
    else:
        geom_prim.GetAttribute("xformOp:translate").Set(pos)

    if not geom_prim.HasAttribute("xformOp:orient"):
        UsdGeom.Xformable(geom_prim).AddOrientOp().Set(quat)
    else:
        geom_prim.GetAttribute("xformOp:orient").Set(quat)


class Rate:
    """
    A substitute for rospy.Rate but always uses wall time.
    """

    def __init__(self, frequency):
        self.frequency = frequency
        self.period = 1.0 / frequency
        self.start_time = time.time()

    def sleep(self):
        end_time = time.time()
        elapsed_time = end_time - self.start_time
        sleep_duration = self.period - elapsed_time

        if sleep_duration > 0:
            time.sleep(sleep_duration)
        self.start_time = time.time()


def run_simulation(world, curr_stage, drone_prim, gimbal_prim):
    """Initializes ROS node, subscriber, simulation context, and runs the main simulation loop."""
    global g_simulation_app
    global g_drone_des_pose, g_is_drone_des_pose_changed
    global g_gimbal_des_pose, g_is_gimbal_des_pose_changed
    global g_pending_spawn_requests, spawn_lock
    global g_pending_move_requests, move_lock

    # Initialize the drone pose object
    g_drone_des_pose = DronePose(pos=Gf.Vec3d(0, 0, 2), quat=Gf.Quatf.GetIdentity())
    g_gimbal_des_pose = DronePose(pos=Gf.Vec3d(0, 0, 0), quat=Gf.Quatf.GetIdentity())

    # Setup ROS node, subscriber and services
    rospy.init_node(
        "isaacsim_interface",
        anonymous=True,
        disable_signals=True,
        log_level=rospy.INFO,  # Changed node name and log level
    )
    ros_sub = rospy.Subscriber(
        "/gazebo/set_link_state",
        LinkState,
        callback_gazebo_linkstate_msg,
        queue_size=10,
    )

    # Advertise Gazebo-compatible services
    move_model_srv = rospy.Service(
        "/gazebo/set_link_state", SetLinkState, move_model_fun
    )
    spawn_model_srv = rospy.Service(
        "/gazebo/spawn_sdf_model", SpawnModel, spawn_model_fun
    )
    pause_physics_srv = rospy.Service("/gazebo/pause_physics", Empty, pause_physics_fun)
    unpause_physics_srv = rospy.Service(
        "/gazebo/unpause_physics", Empty, unpause_physics_fun
    )
    save_world_prim_srv = rospy.Service(
        "/isaacsim_interface/save_world", AddDiagnostics, save_world_prim_fun
    )
    rospy.loginfo("Gazebo compatible services advertised.")

    world.play()

    rate = Rate(10)

    # Main simulation loop
    while g_simulation_app.is_running():
        rate.sleep()

        # Make a copy of the pending spawn requests
        pending_spawn_requests = []
        with spawn_lock:
            if g_pending_spawn_requests:
                pending_spawn_requests = g_pending_spawn_requests[:]
                g_pending_spawn_requests.clear()

        # Spawn models
        if pending_spawn_requests:
            for request_data in pending_spawn_requests:
                if request_data["type"] == "cube":
                    VisualCuboid(
                        prim_path=request_data["prim_path"],
                        name=request_data["name"],
                        position=request_data["pos"],
                        orientation=request_data["quat"],
                        scale=request_data["scale"],
                        color=np.array([0.8, 0.2, 0.2]),  # Example color
                        size=1.0,
                    )
                elif request_data["type"] == "sphere":
                    VisualSphere(
                        prim_path=request_data["prim_path"],
                        name=request_data["name"],
                        position=request_data["pos"],
                        orientation=request_data["quat"],
                        scale=request_data["scale"],
                        color=np.array([0.8, 0.2, 0.2]),  # Example color
                        radius=0.5,
                    )
                elif request_data["type"] == "cylinder":
                    VisualCylinder(
                        prim_path=request_data["prim_path"],
                        name=request_data["name"],
                        position=request_data["pos"],
                        orientation=request_data["quat"],
                        scale=request_data["scale"],
                        color=np.array([0.8, 0.2, 0.2]),  # Example color
                        radius=0.5,
                        height=1.0,
                    )
                elif request_data["type"] == "usd_model":
                    # Spawn USD model using stage reference
                    try:
                        stage.add_reference_to_stage(
                            request_data["usd_path"], request_data["prim_path"]
                        )
                        # Set the position and orientation for the spawned model
                        spawned_prim = curr_stage.GetPrimAtPath(
                            request_data["prim_path"]
                        )
                        if spawned_prim.IsValid():
                            # Add transform operations if they don't exist
                            xformable = UsdGeom.Xformable(spawned_prim)
                            if not spawned_prim.HasAttribute("xformOp:translate"):
                                xformable.AddTranslateOp().Set(
                                    Gf.Vec3d(*request_data["pos"])
                                )
                            else:
                                spawned_prim.GetAttribute("xformOp:translate").Set(
                                    Gf.Vec3d(*request_data["pos"])
                                )

                            if not spawned_prim.HasAttribute("xformOp:orient"):
                                xformable.AddOrientOp().Set(
                                    Gf.Quatf(*request_data["quat"])
                                )
                            else:
                                spawned_prim.GetAttribute("xformOp:orient").Set(
                                    Gf.Quatf(*request_data["quat"])
                                )

                            rospy.loginfo(
                                f"Successfully spawned USD model: {request_data['name']} from {request_data['usd_path']}"
                            )
                        else:
                            rospy.logerr(
                                f"Failed to find spawned prim at path: {request_data['prim_path']}"
                            )
                    except Exception as e:
                        rospy.logerr(
                            f"Failed to spawn USD model {request_data['name']}: {str(e)}"
                        )

                # Apply collision APIs so that we can detect collisions
                # prim = GeometryPrim(request_data["prim_path"])
                # prim.apply_collision_apis()

        # Make a copy of the pending move requests
        pending_move_requests = []
        with move_lock:
            if g_pending_move_requests:
                pending_move_requests = g_pending_move_requests[:]
                g_pending_move_requests.clear()

        # Move models
        if pending_move_requests:
            for data in pending_move_requests:
                move_prim(curr_stage, data["prim_path"], data["pos"], data["quat"])

        # Make a copy of the desired drone pose
        is_drone_des_pose_changed = False
        drone_des_pose = None
        with pose_lock:
            is_drone_des_pose_changed = g_is_drone_des_pose_changed
            if is_drone_des_pose_changed:
                drone_des_pose = g_drone_des_pose
                g_is_drone_des_pose_changed = False

        # Update the drone pose
        if is_drone_des_pose_changed:
            drone_prim.GetAttribute("xformOp:translate").Set(drone_des_pose.pos)
            drone_prim.GetAttribute("xformOp:orient").Set(drone_des_pose.quat)

        # Make a copy of the desired gimbal pose
        is_gimbal_des_pose_changed = False
        gimbal_des_pose = None
        with pose_lock:
            is_gimbal_des_pose_changed = g_is_gimbal_des_pose_changed
            if is_gimbal_des_pose_changed:
                gimbal_des_pose = g_gimbal_des_pose
                g_is_gimbal_des_pose_changed = False

        # Update the gimbal pose
        if is_gimbal_des_pose_changed:
            gimbal_prim.GetAttribute("xformOp:translate").Set(gimbal_des_pose.pos)
            gimbal_prim.GetAttribute("xformOp:orient").Set(gimbal_des_pose.quat)

        # Run 1 simulation step
        world.step(render=True)

    # Cleanup
    rospy.loginfo("Shutting down ROS services and node.")
    # Services are automatically cleaned up when the node shuts down
    ros_sub.unregister()
    world.stop()
    g_simulation_app.close()


def main():
    """Main function to orchestrate the simulation setup and execution."""
    # Arguments are parsed globally before SimulationApp initialization
    rendering_hz = 10.0
    world, curr_stage = create_sim_environment(args.world, rendering_hz)
    drone_prim = add_drone_body(curr_stage)

    # Setup camera rig configuration
    if args.robot_model == "modeljjie":
        cam_rig_components = gen_cam_rig_components_mdjjie()
    elif args.robot_model == "modelx":
        cam_rig_components = gen_cam_rig_components_mdx()
    elif args.robot_model == "modele":
        cam_rig_components = gen_cam_rig_components_mde()
    else:
        print("Incorrect model provided:", args.robot_model)

    gimbal_prim = add_drone_cameras(curr_stage, cam_rig_components)
    # TODO: add clock graph that can publish at 1000Hz
    # create_clock_graph()  # only at 20Hz
    run_simulation(world, curr_stage, drone_prim, gimbal_prim)


if __name__ == "__main__":
    main()
