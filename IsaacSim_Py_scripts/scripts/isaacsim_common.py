#!/usr/bin/env python3

import sys
import os
import time
import math
import numpy as np
from collections import namedtuple
from dataclasses import dataclass, field
import threading

import carb
import omni

from isaacsim.core.api import World
from isaacsim.core.api.objects import VisualCuboid, VisualCylinder, VisualSphere
from isaacsim.core.utils import extensions, stage
from isaacsim.storage.native import get_assets_root_path
from pxr import Gf, UsdGeom
from omni.isaac.core.utils.viewports import set_camera_view

# enable ROS2 bridge extension and then import ros modules
extensions.enable_extension("isaacsim.ros2.bridge")
import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import EntityState, ContactsState, ContactState
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.srv import SpawnEntity
from std_msgs.msg import Header
from std_srvs.srv import Empty
from sensor_msgs.msg import PointCloud2, PointField, Image

# -----------------------------------------------------------------------------
# Global variables (legacy single-UAV path)
# -----------------------------------------------------------------------------
# NOTE: These globals are kept for backward compatibility with the previous
# single-UAV implementation.  The multi-UAV refactor introduces the DroneSimCtx
# dataclass (see below) and associated helpers, but the old code path still
# works when only **one** namespace/drone is spawned.

g_drone_prim_path = "/drone"
g_is_drone_des_pose_changed = False
g_drone_des_pose = None
pose_lock = threading.Lock()
g_pending_spawn_requests = []
spawn_lock = threading.Lock()
g_pending_move_requests = []
move_lock = threading.Lock()
g_node = None  # Global ROS2 node instance (first ctx)

def generate_omap_fun(req, response):
    """callback for /generate_omap service."""
    g_node.get_logger().info("Generating occupancy map")

    # Enable omap extension
    extensions.enable_extension("isaacsim.asset.gen.omap")
    from isaacsim.asset.gen.omap.bindings import _omap as omap

    # Get PhysX interface and stage ID
    physx = omni.physx.get_physx_interface()
    stage_id = omni.usd.get_context().get_stage_id()

    generator = omap.Generator(physx, stage_id)

    resolution = 0.2
    occupied_value = 1
    unoccupied_value = 0
    unknown_value = -1
    generator.update_settings(
        resolution, occupied_value, unoccupied_value, unknown_value
    )

    # Set location to map from and the min and max bounds to map to
    # TODO(Kartik): Make this dynamic based on the world size
    min_bound = np.array([-10.0, -10.0, 0.0])
    max_bound = np.array([10.0, 10.0, 10.0])
    generator.set_transform(
        (0, 0, 0), min_bound - resolution / 2, max_bound + resolution / 2
    )

    # Generate the omap
    generator.generate3d()

    dims = generator.get_dimensions()
    g_node.get_logger().info(f"Buffer dimensions: {dims}")

    # Get locations of the occupied cells in the stage
    points = generator.get_occupied_positions()
    g_node.get_logger().info(f"Generated occupancy map with {len(points)} points")

    # Save occupied cells to npy file
    np.save("omap.npy", points)

    return response


class DronePose:
    def __init__(self, pos=Gf.Vec3d(0, 0, 0), quat=Gf.Quatf.GetIdentity()):
        self.pos = pos
        self.quat = quat
# -----------------------------------------------------------------------------
# Per-UAV context for multi-drone simulation
# -----------------------------------------------------------------------------


@dataclass
class DroneSimCtx:
    """Container holding all state associated with a single UAV instance."""

    namespace: str  # Empty string means root namespace
    prim_path: str  # USD prim path, e.g. "/robot1/drone" or "/drone"

    # ROS2
    ros_node: Node
    pubs: dict = field(default_factory=dict)
    subs: dict = field(default_factory=dict)
    srvs: dict = field(default_factory=dict)

    # Simulation prim handle for this drone (returned by add_drone_body)
    drone_prim: object | None = None

    # Desired pose tracking
    des_pose: DronePose | None = None
    is_pose_dirty: bool = False

    # Optional custom per-step callable (LiDAR / perception wrapper)
    custom_step_fn: callable = None

    # Pre-computed LUT for depth→point-cloud conversion (used by perception)
    depth2pc_lut: np.ndarray | None = None

    # Locks & queues (spawn/move) — kept separate per UAV to avoid contention
    pending_spawn_requests: list = field(default_factory=list)
    spawn_lock: threading.Lock = field(default_factory=threading.Lock)
    pending_move_requests: list = field(default_factory=list)
    move_lock: threading.Lock = field(default_factory=threading.Lock)

def create_sim_environment(world_usd_path, rendering_hz, simulation_app):
    """Sets up the simulation environment, finds assets, and adds a ground plane."""

    world = World(
        physics_dt=0,
        rendering_dt=1.0 / rendering_hz,
        stage_units_in_meters=1.0,
        backend="torch",
    )
    curr_stage = stage.get_current_stage()

    # TODO(subhransu): Make assets available offline.
    if world_usd_path == "":
        # assets_root_path = get_assets_root_path()
        assets_root_path = "/home/ubuntu/isaacsim_assets/Assets/Isaac/4.5"

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
            # Locate Isaac Sim assets folder
            # assets_root_path = get_assets_root_path()
            assets_root_path = "/home/ubuntu/isaacsim_assets/Assets/Isaac/4.5"

            print(f"Using {assets_root_path}")
            if assets_root_path is None:
                carb.log_error("Could not find Isaac Sim assets folder")
                simulation_app.close()
                sys.exit()
            # Try using path relative to assets_root_path
            usd_path = os.path.join(assets_root_path, world_usd_path)

        prim_path = "/World"
        stage.add_reference_to_stage(usd_path, prim_path)
        # Wait two frames so that stage starts loading
        simulation_app.update()
        simulation_app.update()
        print(f"Loading stage from {usd_path}...")
        while stage.is_stage_loading():
            simulation_app.update()
        print("Loading Complete")

    return world, curr_stage


def add_drone_body(curr_stage, prim_path: str | None = None, color_scheme: int = 0):
    """Create a quadrotor body under *prim_path* and return the prim handle.

    If *prim_path* is ``None`` the legacy global ``g_drone_prim_path`` is used
    (thus preserving backward compatibility).
    
    Args:
        curr_stage: USD stage to add the drone to
        prim_path: Path for the drone prim
        color_scheme: Integer index for different color schemes (0=gray, 1=red, 2=blue, 3=green, etc.)
    """

    if prim_path is None:
        prim_path = g_drone_prim_path

    # Define the parent drone prim (UsdGeom.Xform(Usd.Prim(<prim_path>)))
    drone_xform = UsdGeom.Xform.Define(curr_stage, prim_path)
    drone_xform.AddTranslateOp().Set(Gf.Vec3d(0, 0, 0))
    drone_xform.AddOrientOp().Set(Gf.Quatf.GetIdentity())
    drone_prim = drone_xform.GetPrim()  # Usd.Prim(<prim_path>)

    # Define color schemes for different drones
    color_schemes = [
        {"main": [0.6, 0.6, 0.6], "nose": [0.3, 0.3, 0.3]},  # Gray
        {"main": [0.8, 0.2, 0.2], "nose": [0.9, 0.4, 0.4]},  # Red
        {"main": [0.2, 0.4, 0.8], "nose": [0.4, 0.6, 0.9]},  # Blue  
        {"main": [0.2, 0.8, 0.3], "nose": [0.4, 0.9, 0.5]},  # Green
        {"main": [0.8, 0.6, 0.2], "nose": [0.9, 0.7, 0.4]},  # Orange
        {"main": [0.7, 0.2, 0.8], "nose": [0.8, 0.4, 0.9]},  # Purple
        {"main": [0.2, 0.8, 0.8], "nose": [0.4, 0.9, 0.9]},  # Cyan
        {"main": [0.8, 0.8, 0.2], "nose": [0.9, 0.9, 0.4]},  # Yellow
    ]
    
    # Select color scheme (cycle through available schemes)
    scheme_names = ["Gray", "Red", "Blue", "Green", "Orange", "Purple", "Cyan", "Yellow"]
    scheme_idx = color_scheme % len(color_schemes)
    colors = color_schemes[scheme_idx]
    scheme_name = scheme_names[scheme_idx] if scheme_idx < len(scheme_names) else f"Color{scheme_idx}"
    print(f"Creating drone at {prim_path} with {scheme_name} color scheme")

    # Setup Drone geometry (main body + simple orientation indicator)
    MechComponent = namedtuple(
        "MechComponent", ["pos", "scale", "wxyz", "geom_type", "color"]
    )

    mech_comps = {}
    # Main body - slightly larger and more rectangular for better visibility
    mech_comps["MainBody"] = MechComponent(
        pos=[0.0, 0.0, 0.0],
        scale=[0.12, 0.06, 0.06],  # Slightly larger than before
        wxyz=[1.0, 0.0, 0.0, 0.0],
        geom_type="cuboid",
        color=colors["main"],
    )
    
    # Simple nose indicator for orientation - small cube at front
    mech_comps["Nose"] = MechComponent(
        pos=[0.08, 0.0, 0.0],  # Position at front
        scale=[0.03, 0.02, 0.02],  # Small indicator
        wxyz=[1.0, 0.0, 0.0, 0.0],
        geom_type="cuboid",
        color=colors["nose"],
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
                prim_path=prim_path + "/" + prim_name,
                name=prim_name,
                position=np.array(comp.pos),
                orientation=comp.wxyz,
                scale=np.array(comp.scale),
                color=np.array(comp.color),
            )
        elif comp.geom_type == "cylinder":
            VisualCylinder(
                prim_path=prim_path + "/" + prim_name,
                name=prim_name,
                position=np.array(comp.pos),
                orientation=comp.wxyz,
                scale=np.array(comp.scale),
                color=np.array(comp.color),
            )
    return drone_prim


def callback_gazebo_entitystate_msg(entitystate_msg):
    """ROS callback function to update the desired drone pose."""
    global g_drone_des_pose, g_is_drone_des_pose_changed, pose_lock
    pos = entitystate_msg.pose.position
    quat = entitystate_msg.pose.orientation
    # Directly update the attributes of the mutable DronePose object
    with pose_lock:
        if entitystate_msg.name == "quadrotor":
            g_drone_des_pose.pos = Gf.Vec3d(pos.x, pos.y, pos.z)
            g_drone_des_pose.quat = Gf.Quatf(quat.w, quat.x, quat.y, quat.z)
            g_is_drone_des_pose_changed = True


def create_drone_pose_callback(ctx: DroneSimCtx):
    """Create a pose callback function for a specific drone context."""
    def callback_drone_entitystate_msg(entitystate_msg):
        """ROS callback function to update the desired drone pose for this specific drone."""
        pos = entitystate_msg.pose.position
        quat = entitystate_msg.pose.orientation
        
        # Expected entity names: "quadrotor" for legacy, or namespace-specific names
        expected_names = ["quadrotor", ctx.namespace, f"{ctx.namespace}/quadrotor", f"{ctx.namespace}_quadrotor"]
        
        # Debug: Print all received entity state messages for this drone's namespace
        print(f"[{ctx.namespace}] Received EntityState for '{entitystate_msg.name}' (expecting one of {expected_names})")
        
        if entitystate_msg.name in expected_names:
            # Update the drone context's desired pose
            ctx.des_pose = DronePose(
                pos=Gf.Vec3d(pos.x, pos.y, pos.z),
                quat=Gf.Quatf(quat.w, quat.x, quat.y, quat.z)
            )
            ctx.is_pose_dirty = True
            print(f"✓ Updated pose for drone {ctx.namespace}: pos=({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})")
        else:
            print(f"✗ Ignoring entity '{entitystate_msg.name}' for drone {ctx.namespace}")
    
    return callback_drone_entitystate_msg


def create_drone_linkstate_callback(ctx: DroneSimCtx):
    """Create a LinkState callback function for a specific drone context."""
    def callback_drone_linkstate_msg(linkstate_msg):
        """ROS callback function to update the desired drone pose from LinkState messages."""
        pos = linkstate_msg.pose.position
        quat = linkstate_msg.pose.orientation
        
        # Expected link names for base_link: "quadrotor::base_link" for legacy, or namespace-specific names
        expected_base_links = [
            "quadrotor::base_link",
            f"{ctx.namespace}::base_link", 
            f"{ctx.namespace}/quadrotor::base_link",
            f"{ctx.namespace}_quadrotor::base_link"
        ]
        
        # Debug: Print all received link state messages for this drone's namespace
        print(f"[{ctx.namespace}] Received LinkState for '{linkstate_msg.link_name}' (expecting one of {expected_base_links})")
        
        if linkstate_msg.link_name in expected_base_links:
            # Update the drone context's desired pose
            ctx.des_pose = DronePose(
                pos=Gf.Vec3d(pos.x, pos.y, pos.z),
                quat=Gf.Quatf(quat.w, quat.x, quat.y, quat.z)
            )
            ctx.is_pose_dirty = True
            print(f"✓ Updated pose from LinkState for drone {ctx.namespace}: pos=({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})")
        else:
            print(f"✗ Ignoring link '{linkstate_msg.link_name}' for drone {ctx.namespace}")
    
    return callback_drone_linkstate_msg


def create_drone_set_entity_state_callback(ctx: DroneSimCtx):
    """Create a SetEntityState service callback function for a specific drone context."""
    def set_entity_state_callback(req, response):
        """ROS service callback to update the desired drone pose for this specific drone."""
        ctx.ros_node.get_logger().info(
            f"move req for {req.state.name} with pose: {req.state.pose}"
        )

        # Extract pose information
        pos = req.state.pose.position
        quat = req.state.pose.orientation
        
        # Expected entity names: "quadrotor" for legacy, or namespace-specific names
        model_name = req.state.name.rsplit("::", 1)[0] if "::" in req.state.name else req.state.name
        expected_names = [
            "quadrotor", 
            ctx.namespace, 
            f"{ctx.namespace}/quadrotor", 
            f"{ctx.namespace}_quadrotor"
        ]
        
        # Debug: Print all received entity state messages for this drone's namespace  
        print(f"[{ctx.namespace}] Received SetEntityState for '{req.state.name}' / model '{model_name}' (expecting one of {expected_names})")
        
        if model_name in expected_names:
            # Update the drone context's desired pose directly
            ctx.des_pose = DronePose(
                pos=Gf.Vec3d(pos.x, pos.y, pos.z),
                quat=Gf.Quatf(quat.w, quat.x, quat.y, quat.z)
            )
            ctx.is_pose_dirty = True
            print(f"✓ Updated pose from SetEntityState for drone {ctx.namespace}: pos=({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})")
            response.success = True
        else:
            print(f"✗ Ignoring entity '{req.state.name}' for drone {ctx.namespace}")
            response.success = False
            
        return response
    
    return set_entity_state_callback


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


def spawn_entity_fun(req, response):
    """Callback for /gazebo/spawn_entity service. Queues request for main thread."""
    global g_pending_spawn_requests, spawn_lock
    g_node.get_logger().info(f"spawn req for {req.name} with pose: {req.initial_pose}")

    response.success = False

    # Extract pose information
    pos = req.initial_pose.position
    quat = req.initial_pose.orientation
    position_np = np.array([pos.x, pos.y, pos.z])
    orientation_np = np.array([quat.w, quat.x, quat.y, quat.z])

    # Check if req.xml is a .usd file path
    if req.xml.endswith(".usd"):
        # Handle direct .usd file path
        if not os.path.isfile(req.xml):
            g_node.get_logger().error(f"USD file not found: {req.xml}")
            return response

        # Prepare data for USD model spawning
        spawn_data = {
            "name": req.name,
            "pos": position_np,
            "quat": orientation_np,
            "type": "usd_model",
            "usd_path": req.xml,
            "prim_path": f"/World/{req.name}",  # Assuming /World scope
        }
    else:
        # Use existing isaac schema validation
        geom_info = extract_geom_info(req.xml)
        if geom_info is None:
            return response

        # Prepare data for primitive geometry spawning
        spawn_data = {
            "name": req.name,
            "pos": position_np,
            "quat": orientation_np,
            "scale": geom_info["scale"],
            "type": geom_info["geom_type"],
            "prim_path": f"/World/{req.name}",  # Assuming /World scope
        }

    # Acquire lock, add request to queue, release lock
    with spawn_lock:
        g_pending_spawn_requests.append(spawn_data)

    # send success response
    response.success = True
    return response


def set_entity_state_fun(req, response):
    """callback for /set_entity_state service."""
    global g_pending_move_requests, move_lock
    g_node.get_logger().info(
        f"move req for {req.state.name} with pose: {req.state.pose}"
    )

    # Extract pose information
    pos = req.state.pose.position
    quat = req.state.pose.orientation
    # model name is the entity name before the last "::"
    model_name = req.state.name.rsplit("::", 1)[0]
    if model_name == "quadrotor":
        prim_path = g_drone_prim_path
    else:
        prim_path = f"/World/{model_name}"

    # Prepare data for queuing
    move_data = {
        "pos": Gf.Vec3d(pos.x, pos.y, pos.z),
        "quat": Gf.Quatf(quat.w, Gf.Vec3f(quat.x, quat.y, quat.z)),
        "prim_path": prim_path,
    }

    # Acquire lock, add request to queue, release lock
    with move_lock:
        g_pending_move_requests.append(move_data)

    # Wait for g_pending_move_requests to be processed
    # while len(g_pending_move_requests) > 0:
    #     time.sleep(0.01)

    # send success response
    response.success = True
    return response


def pause_physics_fun(req, response):
    """callback for /pause_physics service."""
    g_node.get_logger().info("Pausing physics")
    return response


def unpause_physics_fun(req, response):
    """callback for /unpause_physics service."""
    g_node.get_logger().info("Unpausing physics")
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

    @carb.profiler.profile
    def sleep(self):
        end_time = time.time()
        elapsed_time = end_time - self.start_time
        sleep_duration = self.period - elapsed_time

        if sleep_duration > 0:
            time.sleep(sleep_duration)
        self.start_time = time.time()


@carb.profiler.profile
def process_spawn_requests(curr_stage, pending_spawn_requests):
    """Process pending spawn requests and create objects in the simulation."""
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
                spawned_prim = curr_stage.GetPrimAtPath(request_data["prim_path"])
                if spawned_prim.IsValid():
                    # Add transform operations if they don't exist
                    xformable = UsdGeom.Xformable(spawned_prim)
                    if not spawned_prim.HasAttribute("xformOp:translate"):
                        xformable.AddTranslateOp().Set(Gf.Vec3d(*request_data["pos"]))
                    else:
                        spawned_prim.GetAttribute("xformOp:translate").Set(
                            Gf.Vec3d(*request_data["pos"])
                        )

                    if not spawned_prim.HasAttribute("xformOp:orient"):
                        xformable.AddOrientOp().Set(Gf.Quatf(*request_data["quat"]))
                    else:
                        spawned_prim.GetAttribute("xformOp:orient").Set(
                            Gf.Quatf(*request_data["quat"])
                        )

                    g_node.get_logger().info(
                        f"Successfully spawned USD model: {request_data['name']} from {request_data['usd_path']}"
                    )
                else:
                    g_node.get_logger().error(
                        f"Failed to find spawned prim at path: {request_data['prim_path']}"
                    )
            except Exception as e:
                g_node.get_logger().error(
                    f"Failed to spawn USD model {request_data['name']}: {str(e)}"
                )

        # Apply collision APIs so that we can detect collisions
        # prim = GeometryPrim(request_data["prim_path"])
        # prim.apply_collision_apis()


@carb.profiler.profile
def process_move_requests(curr_stage, pending_move_requests):
    """Process pending move requests and update object positions."""
    for data in pending_move_requests:
        move_prim(curr_stage, data["prim_path"], data["pos"], data["quat"])


@carb.profiler.profile
def get_pending_spawn_requests(spawn_lock, pending_spawn_requests):
    """Get and clear pending spawn requests in a thread-safe manner."""
    requests = []
    with spawn_lock:
        if pending_spawn_requests:
            requests = pending_spawn_requests[:]
            pending_spawn_requests.clear()
    return requests


@carb.profiler.profile
def get_pending_move_requests(move_lock, pending_move_requests):
    """Get and clear pending move requests in a thread-safe manner."""
    requests = []
    with move_lock:
        if pending_move_requests:
            requests = pending_move_requests[:]
            pending_move_requests.clear()
    return requests


@carb.profiler.profile
def get_drone_pose_update(pose_lock, is_drone_des_pose_changed, drone_des_pose):
    """Get updated drone pose in a thread-safe manner."""
    pose_changed = False
    pose = None
    with pose_lock:
        pose_changed = is_drone_des_pose_changed
        if pose_changed:
            pose = drone_des_pose
    return pose_changed, pose


@carb.profiler.profile
def update_drone_pose(drone_prim, is_drone_des_pose_changed, drone_des_pose):
    """Update the drone pose if it has changed."""
    if is_drone_des_pose_changed and drone_des_pose is not None:
        drone_prim.GetAttribute("xformOp:translate").Set(drone_des_pose.pos)
        drone_prim.GetAttribute("xformOp:orient").Set(drone_des_pose.quat)


def setup_ros(namespace: str = "", ctx: DroneSimCtx = None):
    """Create a ROS2 *Node* and standard pubs/subs/srvs for one UAV.

    Returns *(node, pubs, subs, srvs)*.  The very first call also populates the
    legacy global ``g_node`` so that unchanged parts of the codebase continue
    to work when only one drone is present.
    
    Args:
        namespace: ROS namespace for this drone
        ctx: DroneSimCtx for setting up per-drone pose callbacks
    """

    global g_node

    # Initialise rclpy exactly once
    if not rclpy.ok():
        rclpy.init(signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)

    if namespace and not namespace.startswith("/"):
        namespace = "/" + namespace

    node = rclpy.create_node("isaacsim_interface", namespace=namespace)

    pubs = {
        "collision": node.create_publisher(ContactsState, "collision", 1),
        "lfr_pc": node.create_publisher(PointCloud2, "front/pointcloud", 1),
        "ubd_pc": node.create_publisher(PointCloud2, "back/pointcloud", 1),
        "lfr_img": node.create_publisher(Image, "front/depth", 1),
        "ubd_img": node.create_publisher(Image, "back/depth", 1),
    }

    # Set up pose callbacks - use per-drone callbacks if ctx provided, else legacy callback
    if ctx is not None:
        entity_pose_callback = create_drone_pose_callback(ctx)
        linkstate_pose_callback = create_drone_linkstate_callback(ctx)
        
        # Create odometry callback as well for nav_msgs/Odometry
        def odom_callback(odom_msg):
            """Handle nav_msgs/Odometry messages."""
            pos = odom_msg.pose.pose.position
            quat = odom_msg.pose.pose.orientation
            ctx.des_pose = DronePose(
                pos=Gf.Vec3d(pos.x, pos.y, pos.z),
                quat=Gf.Quatf(quat.w, quat.x, quat.y, quat.z)
            )
            ctx.is_pose_dirty = True
            print(f"Updated pose from odometry for drone {ctx.namespace}: pos=({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})")
    else:
        entity_pose_callback = callback_gazebo_entitystate_msg
        linkstate_pose_callback = None
        odom_callback = None

    subs = {
        "gazebo_entitystate": node.create_subscription(
            EntityState,
            "gazebo/set_entity_state",
            entity_pose_callback,
            1,
        ),
    }
    
    # Add LinkState subscription for multi-drone case (this is what the mission system actually uses)
    if ctx is not None and linkstate_pose_callback is not None:
        try:
            from gazebo_msgs.msg import LinkState
            subs["gazebo_linkstate"] = node.create_subscription(
                LinkState,
                "/gazebo/set_link_state",  # Absolute topic path as used by mission system
                linkstate_pose_callback,
                1,
            )
            print(f"Subscribed to LinkState topic for drone {ctx.namespace}")
        except ImportError:
            print("gazebo_msgs.msg.LinkState not available, skipping LinkState subscription")
    
    # Add odometry subscription for multi-drone case
    if ctx is not None and odom_callback is not None:
        # Import nav_msgs locally to avoid global import issues
        try:
            from nav_msgs.msg import Odometry
            subs["odometry"] = node.create_subscription(
                Odometry,
                "odom",  # Standard odometry topic
                odom_callback,
                1,
            )
            print(f"Subscribed to odometry topic for drone {ctx.namespace}")
        except ImportError:
            print("nav_msgs not available, skipping odometry subscription")

    # Set up service callbacks - use per-drone callbacks if ctx provided, else legacy callbacks
    if ctx is not None:
        set_entity_state_callback = create_drone_set_entity_state_callback(ctx)
    else:
        set_entity_state_callback = set_entity_state_fun

    srvs = {
        "set_entity_state": node.create_service(
            SetEntityState, "set_entity_state", set_entity_state_callback
        ),
        "spawn_entity": node.create_service(
            SpawnEntity, "spawn_entity", spawn_entity_fun
        ),
        "pause_physics": node.create_service(
            Empty, "pause_physics", pause_physics_fun
        ),
        "unpause_physics": node.create_service(
            Empty, "unpause_physics", unpause_physics_fun
        ),
        "generate_omap": node.create_service(Empty, "generate_omap", generate_omap_fun),
    }

    # Preserve old behaviour: expose the first node via the global handle so
    # that legacy helper functions (logging, etc.) keep functioning.
    if g_node is None:
        g_node = node

    return node, pubs, subs, srvs


def check_drone_collision(stage, prim_path=None):
    """Check collision for a drone at the specified prim path.
    
    Args:
        stage: The USD stage containing the drone
        prim_path: Path to the drone prim. If None, uses legacy g_drone_prim_path
    
    Returns:
        bool: True if collision detected, False otherwise
    """
    if prim_path is None:
        prim_path = g_drone_prim_path
    
    drone_prim = stage.GetPrimAtPath(prim_path)
    if not drone_prim.IsValid():
        print(f"Drone prim {prim_path} is not valid")
        return False
        
    origin = drone_prim.GetAttribute("xformOp:translate").Get()
    if origin is None:
        print(f"Drone origin {prim_path} is not valid")
        return False
        
    radius = 0.5  # TODO(Kartik): Get radius from the drone model

    # physX query to detect hits for a sphere
    ret = omni.physx.get_physx_scene_query_interface().overlap_sphere_any(
        radius, carb.Float3(origin[0], origin[1], origin[2])
    )
    # print(f"Collision detected for drone {prim_path}: {ret}")
    if ret:
        print(f"WARNING: Collision detected for drone {prim_path}")
    return ret

@carb.profiler.profile
def depth2pointcloud_lut(depth, depth2pc_lut, max_depth=1000):
    depth = np.minimum(depth, max_depth)
    point_cloud = depth.reshape((depth.shape[0], depth.shape[1], 1)) * depth2pc_lut
    return point_cloud


@carb.profiler.profile
def depth2pointclouds(depths, depth2pc_lut):
    pcd_LFR = depth2pointcloud_lut(depths[0], depth2pc_lut)
    pcd_UBD = depth2pointcloud_lut(depths[1], depth2pc_lut)
    return pcd_LFR, pcd_UBD


def quaternion_to_euler(quat):
    """Convert quaternion to euler angles (roll, pitch, yaw)."""
    # Input quaternion format: (w, x, y, z)
    w, x, y, z = quat.real, quat.imaginary[0], quat.imaginary[1], quat.imaginary[2]
    
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    
    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)
    
    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    
    return roll, pitch, yaw


def update_viewer_camera(curr_stage):
    """Update viewer camera position to follow the drone."""
    global g_drone_prim_path
    
    try:
        # Get drone position and orientation from the stage
        drone_prim = curr_stage.GetPrimAtPath(g_drone_prim_path)
        if not drone_prim.IsValid():
            return
            
        # Get drone transform
        drone_pos = drone_prim.GetAttribute("xformOp:translate").Get()
        drone_quat = drone_prim.GetAttribute("xformOp:orient").Get()
        
        if drone_pos is None or drone_quat is None:
            return
            
        # Convert to numpy arrays for calculation
        drone_pos_np = np.array([drone_pos[0], drone_pos[1], drone_pos[2]])
        
        # Extract yaw from quaternion for directional following
        try:
            _, _, yaw = quaternion_to_euler(drone_quat)
            
            # Calculate camera position relative to drone orientation
            offset_distance = 5.0  # distance behind the drone
            height_offset = 3.0   # height above the drone
            
            # Calculate forward direction (negative Y direction in Isaac Sim)
            forward_x = -math.sin(yaw)
            forward_y = -math.cos(yaw)
            
            # Position camera behind the drone
            camera_pos = drone_pos_np.copy()
            camera_pos[0] += forward_x * offset_distance  # Move camera back relative to drone heading
            camera_pos[1] += forward_y * offset_distance  # Move camera back relative to drone heading  
            camera_pos[2] += height_offset    # Move camera up in Z direction
            
        except:
            # Fallback to simple fixed offset if quaternion conversion fails
            offset_distance = 5.0
            height_offset = 3.0
            camera_pos = drone_pos_np.copy()
            camera_pos[0] -= offset_distance  # Move camera back in X direction
            camera_pos[2] += height_offset    # Move camera up in Z direction
        
        # Set camera view to look at the drone from this position
        set_camera_view(eye=camera_pos, target=drone_pos_np)
        
    except Exception as e:
        # Silently handle any errors to avoid disrupting the simulation
        pass


def create_depth2pc_lut():
    """Create lookup table for depth to pointcloud conversion."""
    erp_width = 120
    erp_height = 352
    erp_width_fov = 90
    erp_height_fov = 270

    fx_erp = erp_width / np.deg2rad(erp_width_fov)
    fy_erp = erp_height / np.deg2rad(erp_height_fov)
    cx_erp = (erp_width - 1) / 2
    cy_erp = (erp_height - 1) / 2

    grid = np.mgrid[0:erp_height, 0:erp_width]
    v, u = grid[0], grid[1]
    theta_l_map = -(u - cx_erp) / fx_erp  # elevation
    phi_l_map = -(v - cy_erp) / fy_erp  # azimuth

    sin_el = np.sin(theta_l_map)
    cos_el = np.cos(theta_l_map)
    sin_az = np.sin(phi_l_map)
    cos_az = np.cos(phi_l_map)
    X = cos_az * cos_el
    Y = sin_az * cos_el
    Z = -sin_el
    point_cloud = np.stack([X, Y, Z], axis=-1).astype(np.float32)

    return point_cloud


@carb.profiler.profile
def create_pc2_msg(header, points):
    """Create PointCloud2 message from points."""
    assert points.dtype == np.float32
    assert len(points.shape) == 3
    assert points.shape[2] == 3

    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]

    point_step = points.shape[2] * 4
    pc = PointCloud2(
        header=header,
        height=points.shape[0],
        width=points.shape[1],
        is_dense=True,
        is_bigendian=False,
        fields=fields,
        point_step=point_step,
        row_step=points.shape[1] * point_step,
        data=points.tobytes(),
    )
    return pc


@carb.profiler.profile
def create_image_msg(header, depth):
    """Create Image message from depth data."""
    assert depth.dtype == np.float32
    assert len(depth.shape) == 2

    img_msg = Image(
        header=header,
        height=depth.shape[0],
        width=depth.shape[1],
        encoding="32FC1",
        is_bigendian=False,
        data=depth.tobytes(),
        step=(depth.shape[1] * 4),
    )
    return img_msg


def run_simulation_loop(
    simulation_app, world, curr_stage, drone_prim, custom_step_function=None, namespace=""
):
    """Common simulation loop that handles spawn/move requests and drone pose updates."""
    global g_drone_des_pose, g_is_drone_des_pose_changed
    global g_pending_spawn_requests, spawn_lock
    global g_pending_move_requests, move_lock

    # Initialize the drone pose object
    g_drone_des_pose = DronePose(pos=Gf.Vec3d(0, 0, 2), quat=Gf.Quatf.GetIdentity())
    g_is_drone_des_pose_changed = True

    # Setup ROS publishers, subscribers and services with namespace
    node, ros_pubs, ros_subs, ros_srvs = setup_ros(namespace)
    # For backward compatibility maintain g_node reference
    global g_node
    g_node = node

    depth2pc_lut = create_depth2pc_lut()

    # Reset the simulation
    world.reset()
    for i in range(10):
        simulation_app.update()

    rendering_dt = world.get_rendering_dt()
    assert rendering_dt > 0
    rate = Rate(1.0 / rendering_dt)

    print("Starting simulation loop")

    # Main simulation loop
    while simulation_app.is_running():
        rate.sleep()

        # Spin ROS2 node to process callbacks
        rclpy.spin_once(g_node, timeout_sec=0.01)

        # Get and process pending spawn requests
        pending_spawn_requests = get_pending_spawn_requests(
            spawn_lock, g_pending_spawn_requests
        )
        process_spawn_requests(curr_stage, pending_spawn_requests)

        # Get and process pending move requests
        pending_move_requests = get_pending_move_requests(
            move_lock, g_pending_move_requests
        )
        process_move_requests(curr_stage, pending_move_requests)

        # Get and update drone pose
        is_drone_des_pose_changed, drone_des_pose = get_drone_pose_update(
            pose_lock, g_is_drone_des_pose_changed, g_drone_des_pose
        )
        g_is_drone_des_pose_changed = False
        update_drone_pose(drone_prim, is_drone_des_pose_changed, drone_des_pose)

        t_now = g_node.get_clock().now()

        # Run 1 simulation step
        world.step(render=True)

        header = Header()
        header.stamp = t_now.to_msg()

        # Publish collision information
        header.frame_id = "world"
        msg = ContactsState(header=header)
        if check_drone_collision(curr_stage):
            # print("Collision detected")
            msg.states.append(ContactState(collision1_name="drone"))
        ros_pubs["collision"].publish(msg)

        # Update viewer camera to follow the drone (only if GUI is enabled)
        try:
            # Check if we're not in headless mode by seeing if we can access viewport
            import omni.kit.viewport.utility
            viewport = omni.kit.viewport.utility.get_active_viewport()
            if viewport and viewport.updates_enabled:
                update_viewer_camera(curr_stage)
        except:
            # If viewport access fails, we're likely in headless mode, so skip camera update
            pass

        # Call custom step function if provided
        if custom_step_function:
            depths = custom_step_function()
            if depths is not None:
                pcd_LFR, pcd_UBD = depth2pointclouds(depths, depth2pc_lut)
                header = Header()
                header.stamp = t_now.to_msg()
                header.frame_id = "lfr"
                ros_pubs["lfr_img"].publish(create_image_msg(header, depths[0]))
                ros_pubs["lfr_pc"].publish(create_pc2_msg(header, pcd_LFR))
                header.frame_id = "ubd"
                ros_pubs["ubd_img"].publish(create_image_msg(header, depths[1]))
                ros_pubs["ubd_pc"].publish(create_pc2_msg(header, pcd_UBD))

    # Cleanup
    g_node.get_logger().info("Shutting down ROS services and node.")
    rclpy.shutdown()
    world.stop()
    simulation_app.close()


# =============================================================================
# Multi-UAV Support (experimental)
# =============================================================================


def run_simulation_loop_multi(simulation_app, world, curr_stage, drone_ctxs: list[DroneSimCtx]):
    """Simulation loop that handles *multiple* DroneSimCtx objects.

    The original single-drone function is left untouched for backward
    compatibility.  This multi-drone loop iterates over each ctx, spins its ROS
    node, updates the drone pose, processes custom sensor callbacks and
    publishes outputs on the ctx-specific publishers.
    """

    if not drone_ctxs:
        raise ValueError("run_simulation_loop_multi: empty drone_ctxs list")

    # Use the first ctx as the reference for world reset timing, viewer camera,
    # etc.
    reference_ctx = drone_ctxs[0]

    # Reset simulation
    world.reset()
    for _ in range(10):
        simulation_app.update()

    rendering_dt = world.get_rendering_dt()
    assert rendering_dt > 0
    rate = Rate(1.0 / rendering_dt)

    # Create depth→point-cloud LUT once and share
    depth2pc_lut = create_depth2pc_lut()
    for ctx in drone_ctxs:
        ctx.depth2pc_lut = depth2pc_lut

    # Initial desired poses (arranged in a grid pattern for better visualization)
    for i, ctx in enumerate(drone_ctxs):
        # Arrange drones in a 3x3 grid pattern with proper spacing
        grid_size = int(math.ceil(math.sqrt(len(drone_ctxs))))
        row = i // grid_size
        col = i % grid_size
        
        # Space drones 4 units apart in X and Y, 2 units high in Z
        x_pos = (col - grid_size // 2) * 4.0
        y_pos = (row - grid_size // 2) * 4.0 
        z_pos = 2.0
        
        ctx.des_pose = DronePose(pos=Gf.Vec3d(x_pos, y_pos, z_pos), quat=Gf.Quatf.GetIdentity())
        ctx.is_pose_dirty = True

    print(f"Starting multi-UAV simulation loop with {len(drone_ctxs)} drones")

    while simulation_app.is_running():
        rate.sleep()

        # ------------------------------------------------------------------
        # ROS2 spin for each drone
        # ------------------------------------------------------------------
        for ctx in drone_ctxs:
            rclpy.spin_once(ctx.ros_node, timeout_sec=0.01)

        t_now = drone_ctxs[0].ros_node.get_clock().now()

        # ------------------------------------------------------------------
        # Update each drone
        # ------------------------------------------------------------------
        for ctx in drone_ctxs:
            # Update pose if dirty
            if ctx.is_pose_dirty and ctx.des_pose is not None:
                ctx.drone_prim.GetAttribute("xformOp:translate").Set(ctx.des_pose.pos)
                ctx.drone_prim.GetAttribute("xformOp:orient").Set(ctx.des_pose.quat)
                ctx.is_pose_dirty = False
                print(f"🚁 Applied visual update for drone {ctx.namespace}: pos=({ctx.des_pose.pos[0]:.2f}, {ctx.des_pose.pos[1]:.2f}, {ctx.des_pose.pos[2]:.2f})")

        # Run single simulation step
        world.step(render=True)

        # ------------------------------------------------------------------
        # Publish per-drone outputs
        # ------------------------------------------------------------------
        for ctx in drone_ctxs:
            header = Header()
            header.stamp = t_now.to_msg()

            # Collision detection
            header.frame_id = "world"
            msg = ContactsState(header=header)
            if check_drone_collision(curr_stage, ctx.prim_path):
                msg.states.append(ContactState(collision1_name=f"drone_{ctx.namespace}"))
                print(f"Collision detected for drone {ctx.namespace}")
            ctx.pubs["collision"].publish(msg)

            # Custom step (LiDAR / perception)
            if ctx.custom_step_fn is not None:
                depths = ctx.custom_step_fn()
                if depths is not None:
                    pc_LFR, pc_UBD = depth2pointclouds(depths, ctx.depth2pc_lut)
                    header = Header()
                    header.stamp = t_now.to_msg()

                    ctx.pubs["lfr_pc"].publish(create_pc2_msg(header, pc_LFR))
                    ctx.pubs["ubd_pc"].publish(create_pc2_msg(header, pc_UBD))

                    ctx.pubs["lfr_img"].publish(create_image_msg(header, depths[0]))
                    ctx.pubs["ubd_img"].publish(create_image_msg(header, depths[1]))

        # ------------------------------------------------------------------
        # Viewer camera follow first drone (if GUI enabled)
        # ------------------------------------------------------------------
        try:
            import omni.kit.viewport.utility
            viewport = omni.kit.viewport.utility.get_active_viewport()
            if viewport and viewport.updates_enabled:
                update_viewer_camera(curr_stage)
        except Exception:
            pass
