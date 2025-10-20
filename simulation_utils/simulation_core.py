import time
import math

import numpy as np

import carb
from physics_engine.isaacsim_utils import extensions, stage, set_camera_view
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

from robot.robot_drone_autel import DronePose, RobotDroneAutel
from containers import get_container
from simulation_utils.perception import (
    create_depth2pc_lut,
    depth2pointclouds,
    process_semantic_detection,
)
from simulation_utils.message_convert import create_pc2_msg, create_image_msg
from containers import get_container


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
            from utils.quat_to_euler import quat_to_euler

            quat = (
                drone_quat.real,
                drone_quat.imaginary[0],
                drone_quat.imaginary[1],
                drone_quat.imaginary[2],
            )
            _, _, yaw = quat_to_euler(quat)

            # Calculate camera position relative to drone orientation
            offset_distance = 5.0  # distance behind the drone
            height_offset = 3.0  # height above the drone

            # Calculate forward direction (negative Y direction in Isaac Sim)
            forward_x = -math.sin(yaw)
            forward_y = -math.cos(yaw)

            # Position camera behind the drone
            camera_pos = drone_pos_np.copy()
            camera_pos[0] += (
                forward_x * offset_distance
            )  # Move camera back relative to drone heading
            camera_pos[1] += (
                forward_y * offset_distance
            )  # Move camera back relative to drone heading
            camera_pos[2] += height_offset  # Move camera up in Z direction

        except:
            # Fallback to simple fixed offset if quaternion conversion fails
            offset_distance = 5.0
            height_offset = 3.0
            camera_pos = drone_pos_np.copy()
            camera_pos[0] -= offset_distance  # Move camera back in X direction
            camera_pos[2] += height_offset  # Move camera up in Z direction

        # Set camera view to look at the drone from this position
        set_camera_view(eye=camera_pos, target=drone_pos_np)

    except Exception as e:
        # Silently handle any errors to avoid disrupting the simulation
        pass


def run_simulation_loop_multi(
    simulation_app,
    drone_ctxs: list[RobotDroneAutel],
    semantic_camera,
    semantic_camera_prim_path,
    semantic_map,
):
    """Simulation loop that handles *multiple* DroneSimCtx objects.

    The original single-drone function is left untouched for backward
    compatibility.  This multi-drone loop iterates over each ctx, spins its ROS
    node, updates the drone pose, processes custom sensor callbacks and
    publishes outputs on the ctx-specific publishers.
    """

    container = get_container()
    world = container.world()
    scene_manager = container.scene_manager()
    if not drone_ctxs:
        raise ValueError("run_simulation_loop_multi: empty drone_ctxs list")

    # Reset simulation
    world.reset()
    for _ in range(10):
        simulation_app.update()

    semantic_camera.initialize()  # wait 10 frames

    # Switch viewport to semantic camera
    from omni.kit.viewport.utility import get_viewport_from_window_name

    viewport_manager = container.viewport_manager()
    # isaacsim default viewport
    viewport_manager.register_viewport(
        name="Viewport", viewport_obj=get_viewport_from_window_name("Viewport")
    )
    viewport_manager.change_viewport(
        camera_prim_path=semantic_camera_prim_path, viewport_name="Viewport"
    )

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
        y_pos = (row - grid_size // 2) * 4.0 + 5
        z_pos = 2.0

        ctx.des_pose = DronePose(
            pos=Gf.Vec3d(x_pos, y_pos, z_pos), quat=Gf.Quatf.GetIdentity()
        )
        ctx.is_pose_dirty = True

    print(f"Starting multi-UAV simulation loop with {len(drone_ctxs)} drones")
    count = 0
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
                print(ctx, ctx.drone_prim)
                ctx.drone_prim.GetAttribute("xformOp:translate").Set(ctx.des_pose.pos)
                ctx.drone_prim.GetAttribute("xformOp:orient").Set(ctx.des_pose.quat)
                ctx.is_pose_dirty = False
                print(
                    f"🚁 Applied visual update for drone {ctx.namespace}: pos=({ctx.des_pose.pos[0]:.2f}, {ctx.des_pose.pos[1]:.2f}, {ctx.des_pose.pos[2]:.2f})"
                )

        # Run single simulation step
        world.step(render=True)

        # semantic camera detection
        if count % 120 == 0 and count > 0:
            result = process_semantic_detection(semantic_camera, semantic_map)
        count += 1

        # ------------------------------------------------------------------
        # Publish per-drone outputs
        # ------------------------------------------------------------------
        for ctx in drone_ctxs:
            header = Header()
            header.stamp = t_now.to_msg()

            # Collision detection
            header.frame_id = "world"
            msg = ContactsState(header=header)
            if scene_manager.check_prim_collision(prim_path=ctx.prim_path):
                msg.states.append(
                    ContactState(collision1_name=f"drone_{ctx.namespace}")
                )
                print(f"Collision detected for drone {ctx.namespace}")
            ctx.pubs["collision"].publish(msg)

            # Custom step (LiDAR / perception)
            if ctx.custom_step_fn is not None:
                depths = ctx.custom_step_fn()
                if depths is not None:
                    pc_LFR, pc_UBD = depth2pointclouds(depths, ctx.depth2pc_lut)
                    # pc_LFR = ctx
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
                update_viewer_camera(scene_manager.stage)
        except Exception:
            pass
