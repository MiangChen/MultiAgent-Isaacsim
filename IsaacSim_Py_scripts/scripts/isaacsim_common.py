#!/usr/bin/env python3

import sys
import os
import time
import numpy as np
from collections import namedtuple
import threading

import carb
import omni

from isaacsim.core.api import World
from isaacsim.core.api.objects import VisualCuboid, VisualCylinder, VisualSphere
from isaacsim.core.utils import extensions, stage
from isaacsim.storage.native import get_assets_root_path
from pxr import Gf, UsdGeom

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

from scipy.spatial.transform import Rotation

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
g_node = None  # Global ROS2 node instance

# Rotation matrices lambda functions
Rx = lambda theta: Rotation.from_euler("x", theta, degrees=True)
Ry = lambda theta: Rotation.from_euler("y", theta, degrees=True)
Rz = lambda theta: Rotation.from_euler("z", theta, degrees=True)
R_CAM_GIM = Rz(-90) * Rx(90)

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
        assets_root_path = get_assets_root_path()
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
            assets_root_path = get_assets_root_path()
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
        scale=[0.08, 0.04, 0.04],
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


def add_gimbal_camera(curr_stage, drone_prim_path):
    """Creates a gimbal camera on the drone."""
    from collections import namedtuple

    CamRigComponent = namedtuple("CamRigComponent", ["pos", "wxyz", "cam_type", "render_types"])

    # Define gimbal camera component
    gimbal_comp = CamRigComponent(
        pos=[0.16, 0.0, 0.00],
        wxyz=[1, 0, 0, 0],
        cam_type="gimbal",
        render_types=["rgb"]
    )

    # Create camera prim
    cam_prim_name = "gimbal"
    cam_prim_path = drone_prim_path + "/" + cam_prim_name
    cam_prim = UsdGeom.Camera.Define(curr_stage, cam_prim_path)
    cam_prim.AddTranslateOp().Set(Gf.Vec3d(gimbal_comp.pos))
    cam_prim.AddOrientOp().Set(Gf.Quatf(*gimbal_comp.wxyz))

    # Set camera parameters
    width = 320
    height = 180
    cam_prim.GetFocalLengthAttr().Set(16.2)
    cam_prim.GetHorizontalApertureAttr().Set(21.0)
    cam_prim.GetVerticalApertureAttr().Set(11.8125)
    cam_prim.GetProjectionAttr().Set("perspective")
    cam_prim.GetClippingRangeAttr().Set((0.1, 1.0e4))

    gimbal_prim = cam_prim.GetPrim()

    return gimbal_prim, cam_prim_path, (width, height)


def setup_gimbal_image_acquisition(gimbal_prim_path, resolution):
    """Setup image acquisition for gimbal camera. Returns annotator or None."""
    try:
        # Import omni replicator (should be available after simulation app init)
        import omni.replicator.core

        # Create render product from the camera prim
        render_product = omni.replicator.core.create.render_product(
            gimbal_prim_path, resolution
        )

        # Create RGB annotator to read image data
        annotator = omni.replicator.core.AnnotatorRegistry.get_annotator("rgb")
        annotator.attach(render_product)

        print(f"Successfully set up gimbal image acquisition for {gimbal_prim_path} at {resolution}")
        return annotator

    except Exception as e:
        print(f"Failed to setup gimbal image acquisition: {str(e)}")
        return None


def get_gimbal_image(gimbal_annotator):
    """Get RGB image from gimbal camera. Returns numpy array or None."""
    if gimbal_annotator is None:
        return None

    try:
        # Get camera data from annotator
        rgba_data = gimbal_annotator.get_data()
        if rgba_data is None:
            return None

        # Convert RGBA to RGB by dropping alpha channel
        # Expected format: (180, 320, 4) -> (180, 320, 3)
        rgb_data = rgba_data[:, :, :3]

        return rgb_data

    except Exception as e:
        print(f"Error getting gimbal image: {str(e)}")
        return None


def callback_gazebo_entitystate_msg(entitystate_msg):
    """ROS callback function to update the desired drone and gimbal poses."""
    global g_drone_des_pose, g_is_drone_des_pose_changed, pose_lock
    global g_gimbal_des_pose, g_is_gimbal_des_pose_changed
    pos = entitystate_msg.pose.position
    quat = entitystate_msg.pose.orientation
    entity_name = entitystate_msg.name

    # Directly update the attributes of the mutable DronePose object
    with pose_lock:
        if entity_name == "quadrotor::base_link":
            g_drone_des_pose.pos = Gf.Vec3d(pos.x, pos.y, pos.z)
            g_drone_des_pose.quat = Gf.Quatf(quat.w, quat.x, quat.y, quat.z)
            g_is_drone_des_pose_changed = True
        elif entity_name == "quadrotor::camera_link":
            gim_rot = Rotation.from_quat([quat.x, quat.y, quat.z, quat.w])
            cam_rot = R_CAM_GIM * gim_rot
            cam_quat = cam_rot.as_quat() # [x,y,z,w]
            g_gimbal_des_pose.pos = Gf.Vec3d(pos.x, pos.y, pos.z)
            g_gimbal_des_pose.quat = Gf.Quatf(cam_quat[3], cam_quat[0], cam_quat[1], cam_quat[2])
            g_is_gimbal_des_pose_changed = True
        else:
            print(f"Warning: Unhandled entity name: {entity_name}")


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
    # g_node.get_logger().info(
    #     f"move req for {req.state.name} with pose: {req.state.pose}"
    # )

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


@carb.profiler.profile
def get_gimbal_pose_update(pose_lock, is_gimbal_des_pose_changed, gimbal_des_pose):
    """Get updated gimbal pose in a thread-safe manner."""
    pose_changed = False
    pose = None
    with pose_lock:
        pose_changed = is_gimbal_des_pose_changed
        if pose_changed:
            pose = gimbal_des_pose
    return pose_changed, pose


@carb.profiler.profile
def update_gimbal_pose(curr_stage, gimbal_prim_path, is_gimbal_des_pose_changed, gimbal_des_pose):
    """Update the gimbal pose if it has changed."""
    if is_gimbal_des_pose_changed and gimbal_des_pose is not None:
        gimbal_prim = curr_stage.GetPrimAtPath(gimbal_prim_path)
        if gimbal_prim and gimbal_prim.IsValid():
            gimbal_prim.GetAttribute("xformOp:translate").Set(gimbal_des_pose.pos)
            gimbal_prim.GetAttribute("xformOp:orient").Set(gimbal_des_pose.quat)


def setup_ros():
    """Setup ROS node, publishers, subscribers and services for both lidar and perception."""
    global g_node
    rclpy.init(signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
    g_node = rclpy.create_node("isaacsim_interface")

    # Common publishers
    ros_pubs = {
        "collision": g_node.create_publisher(ContactsState, "/collision", 1),
        "lfr_pc": g_node.create_publisher(PointCloud2, "/front/pointcloud", 1),
        "ubd_pc": g_node.create_publisher(PointCloud2, "/back/pointcloud", 1),
        "lfr_img": g_node.create_publisher(Image, "/front/depth", 1),
        "ubd_img": g_node.create_publisher(Image, "/back/depth", 1),
        "gim_img": g_node.create_publisher(Image, "/gimbal_camera/image_raw", 1),
    }

    # Common subscribers
    # Note(subhransu): This doesn't exist on ros2(humble) gazebo_ros yet
    ros_subs = {
        "gazebo_linkstate": g_node.create_subscription(
            EntityState,
            "/set_entity_state",
            callback_gazebo_entitystate_msg,
            1,
        ),
    }

    # Common services
    ros_srvs = {
        "set_entity_state": g_node.create_service(
            SetEntityState, "/set_entity_state", set_entity_state_fun
        ),
        "spawn_entity": g_node.create_service(
            SpawnEntity, "/spawn_entity", spawn_entity_fun
        ),
        "pause_physics": g_node.create_service(
            Empty, "/pause_physics", pause_physics_fun
        ),
        "unpause_physics": g_node.create_service(
            Empty, "/unpause_physics", unpause_physics_fun
        ),
        "generate_omap": g_node.create_service(Empty, "/generate_omap", generate_omap_fun),
    }

    return ros_pubs, ros_subs, ros_srvs


def check_drone_collision(stage):
    drone_prim = stage.GetPrimAtPath(g_drone_prim_path)
    origin = drone_prim.GetAttribute("xformOp:translate").Get()
    radius = 0.5  # TODO(Kartik): Get radius from the drone model

    # physX query to detect hits for a sphere
    return omni.physx.get_physx_scene_query_interface().overlap_sphere_any(
        radius, carb.Float3(origin[0], origin[1], origin[2])
    )


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


@carb.profiler.profile
def create_rgb_image_msg(header, rgb_image):
    """Create Image message from RGB image data."""
    # Expected format: (180, 320, 3) uint8 RGB image
    img_msg = Image(
        header=header,
        height=rgb_image.shape[0],
        width=rgb_image.shape[1],
        encoding="rgb8",
        is_bigendian=False,
        data=rgb_image.tobytes(),
        step=(rgb_image.shape[1] * 3),  # 3 bytes per pixel for RGB
    )
    return img_msg


def run_simulation_loop(
    simulation_app, world, curr_stage, drone_prim, custom_step_function=None
):
    """Common simulation loop that handles spawn/move requests and drone pose updates."""
    global g_drone_des_pose, g_is_drone_des_pose_changed
    global g_gimbal_des_pose, g_is_gimbal_des_pose_changed
    global g_pending_spawn_requests, spawn_lock
    global g_pending_move_requests, move_lock

    # Initialize the drone pose object
    g_drone_des_pose = DronePose(pos=Gf.Vec3d(0, 0, 2), quat=Gf.Quatf.GetIdentity())
    g_is_drone_des_pose_changed = True

    # Initialize the gimbal pose object
    g_gimbal_des_pose = DronePose(pos=Gf.Vec3d(0.16, 0.0, 0.0), quat=Gf.Quatf.GetIdentity())
    g_is_gimbal_des_pose_changed = True

    # Setup ROS publishers, subscribers and services
    ros_pubs, ros_subs, ros_srvs = setup_ros()

    depth2pc_lut = create_depth2pc_lut()

    # Setup gimbal camera
    gimbal_prim, gimbal_prim_path, gimbal_resolution = add_gimbal_camera(curr_stage, g_drone_prim_path)
    gimbal_annotator = setup_gimbal_image_acquisition(gimbal_prim_path, gimbal_resolution)

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

        # Get and update gimbal pose
        is_gimbal_des_pose_changed, gimbal_des_pose = get_gimbal_pose_update(
            pose_lock, g_is_gimbal_des_pose_changed, g_gimbal_des_pose
        )
        g_is_gimbal_des_pose_changed = False
        update_gimbal_pose(curr_stage, gimbal_prim_path, is_gimbal_des_pose_changed, gimbal_des_pose)

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

        # Process gimbal camera
        gimbal_image = get_gimbal_image(gimbal_annotator)
        if gimbal_image is not None:
            header = Header()
            header.stamp = t_now.to_msg()
            header.frame_id = "gimbal_camera"
            ros_pubs["gim_img"].publish(create_rgb_image_msg(header, gimbal_image))


    # Cleanup
    g_node.get_logger().info("Shutting down ROS services and node.")
    rclpy.shutdown()
    world.stop()
    simulation_app.close()
