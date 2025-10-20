import os
import time
import math
import numpy as np
from collections import namedtuple

import threading

import carb

from physics_engine.pxr_utils import Gf

import rclpy

from gazebo_msgs.msg import EntityState, ContactsState, ContactState
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.srv import SpawnEntity
from std_msgs.msg import Header
from std_srvs.srv import Empty
from sensor_msgs.msg import PointCloud2, PointField, Image

from containers import get_container
from map.map_semantic_map import MapSemantic
from robot.robot_drone_autel import DronePose, RobotDroneAutel

# -----------------------------------------------------------------------------
# Global variables (legacy single-UAV path)
# -----------------------------------------------------------------------------
# NOTE: These globals are kept for backward compatibility with the previous
# single-UAV implementation.  The multi-UAV refactor introduces the RobotDroneAutel
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

container = get_container()
scene_manager = container.scene_manager()
grid_map = container.grid_map()
world = container.world()


def setup_ros(namespace: str = "", ctx=None):
    """Create a ROS2 *Node* and standard pubs/subs/srvs for one UAV.

    Returns *(node, pubs, subs, srvs)*.  The very first call also populates the
    legacy global ``g_node`` so that unchanged parts of the codebase continue
    to work when only one drone is present.

    Args:
        namespace: ROS namespace for this drone
        ctx: RobotDroneAutel for setting up per-drone pose callbacks
    """

    global g_node

    # # Initialise rclpy exactly once
    # if not rclpy.ok():
    #     rclpy.init(signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)

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
                quat=Gf.Quatf(quat.w, quat.x, quat.y, quat.z),
            )
            ctx.is_pose_dirty = True
            print(
                f"Updated pose from odometry for drone {ctx.namespace}: pos=({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})"
            )

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
            print(
                "gazebo_msgs.msg.LinkState not available, skipping LinkState subscription"
            )

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
        "pause_physics": node.create_service(Empty, "pause_physics", pause_physics_fun),
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


def create_drone_pose_callback(ctx: RobotDroneAutel):
    """Create a pose callback function for a specific drone context."""

    def callback_drone_entitystate_msg(entitystate_msg):
        """ROS callback function to update the desired drone pose for this specific drone."""
        pos = entitystate_msg.pose.position
        quat = entitystate_msg.pose.orientation

        # Expected entity names: "quadrotor" for legacy, or namespace-specific names
        expected_names = [
            "quadrotor",
            ctx.namespace,
            f"{ctx.namespace}/quadrotor",
            f"{ctx.namespace}_quadrotor",
        ]

        # Debug: Print all received entity state messages for this drone's namespace
        print(
            f"[{ctx.namespace}] Received EntityState for '{entitystate_msg.name}' (expecting one of {expected_names})"
        )

        if entitystate_msg.name in expected_names:
            # Update the drone context's desired pose
            ctx.des_pose = DronePose(
                pos=Gf.Vec3d(pos.x, pos.y, pos.z),
                quat=Gf.Quatf(quat.w, quat.x, quat.y, quat.z),
            )
            ctx.is_pose_dirty = True
            print(
                f"✓ Updated pose for drone {ctx.namespace}: pos=({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})"
            )
        else:
            print(
                f"✗ Ignoring entity '{entitystate_msg.name}' for drone {ctx.namespace}"
            )

    return callback_drone_entitystate_msg


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


def create_drone_set_entity_state_callback(ctx: RobotDroneAutel):
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
        model_name = (
            req.state.name.rsplit("::", 1)[0]
            if "::" in req.state.name
            else req.state.name
        )
        expected_names = [
            "quadrotor",
            ctx.namespace,
            f"{ctx.namespace}/quadrotor",
            f"{ctx.namespace}_quadrotor",
        ]

        # Debug: Print all received entity state messages for this drone's namespace
        print(
            f"[{ctx.namespace}] Received SetEntityState for '{req.state.name}' / model '{model_name}' (expecting one of {expected_names})"
        )

        if model_name in expected_names:
            # Update the drone context's desired pose directly
            ctx.des_pose = DronePose(
                pos=Gf.Vec3d(pos.x, pos.y, pos.z),
                quat=Gf.Quatf(quat.w, quat.x, quat.y, quat.z),
            )
            ctx.is_pose_dirty = True
            print(
                f"✓ Updated pose from SetEntityState for drone {ctx.namespace}: pos=({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})"
            )
            response.success = True
        else:
            print(f"✗ Ignoring entity '{req.state.name}' for drone {ctx.namespace}")
            response.success = False

        return response

    return set_entity_state_callback


def create_drone_linkstate_callback(ctx: RobotDroneAutel):
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
            f"{ctx.namespace}_quadrotor::base_link",
        ]

        # Debug: Print all received link state messages for this drone's namespace
        print(
            f"[{ctx.namespace}] Received LinkState for '{linkstate_msg.link_name}' (expecting one of {expected_base_links})"
        )

        if linkstate_msg.link_name in expected_base_links:
            # Update the drone context's desired pose
            ctx.des_pose = DronePose(
                pos=Gf.Vec3d(pos.x, pos.y, pos.z),
                quat=Gf.Quatf(quat.w, quat.x, quat.y, quat.z),
            )
            ctx.is_pose_dirty = True
            print(
                f"✓ Updated pose from LinkState for drone {ctx.namespace}: pos=({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})"
            )
        else:
            print(
                f"✗ Ignoring link '{linkstate_msg.link_name}' for drone {ctx.namespace}"
            )

    return callback_drone_linkstate_msg


def generate_omap_fun(req, response):
    """callback for /generate_omap service."""
    g_node.get_logger().info("Generating occupancy map")
    # # Generate the omap
    grid_map.generate_grid_map("3d")

    dims = grid_map.generator.get_dimensions()
    g_node.get_logger().info(f"Buffer dimensions: {dims}")

    # Get locations of the occupied cells in the stage
    points = grid_map.generator.get_occupied_positions()
    g_node.get_logger().info(f"Generated occupancy map with {len(points)} points")

    # Save occupied cells to npy file
    np.save("omap.npy", points)

    return response


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


@carb.profiler.profile
def process_move_requests(curr_stage, pending_move_requests):
    """Process pending move requests and update object positions."""
    for data in pending_move_requests:
        scene_manager.adjust_prim(
            prim_path=data["prim_path_absolute"],
            position=data["pos"],
            quat=data["quat"],
            scale=None,
        )


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
