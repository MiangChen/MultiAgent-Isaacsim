# Standard library imports
import math
import time

# Third-party library imports
import numpy as np
import carb

# Local project imports
from containers import get_container
from physics_engine.isaacsim_utils import extensions, stage, set_camera_view
from physics_engine.pxr_utils import Gf
from robot.robot_drone_autel import DronePose, RobotDroneAutel
from simulation_utils.message_convert import create_pc2_msg, create_image_msg

# ROS2 imports
import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import EntityState, ContactsState, ContactState
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.srv import SpawnEntity
from std_msgs.msg import Header
from std_srvs.srv import Empty
from sensor_msgs.msg import PointCloud2, PointField, Image


class Rate:
    def __init__(self, frequency):
        self.period = 1.0 / frequency
        self.start_time = time.time()

    @carb.profiler.profile
    def sleep(self):
        elapsed = time.time() - self.start_time
        sleep_duration = self.period - elapsed
        if sleep_duration > 0:
            time.sleep(sleep_duration)
        self.start_time = time.time()


def update_viewer_camera(curr_stage):
    global g_drone_prim_path
    try:
        drone_prim = curr_stage.GetPrimAtPath(g_drone_prim_path)
        if not drone_prim.IsValid():
            return

        drone_pos = drone_prim.GetAttribute("xformOp:translate").Get()
        drone_quat = drone_prim.GetAttribute("xformOp:orient").Get()
        if not drone_pos or not drone_quat:
            return

        drone_pos_np = np.array([drone_pos[0], drone_pos[1], drone_pos[2]])

        try:
            from utils.quat_to_euler import quat_to_euler
            quat = (drone_quat.real, drone_quat.imaginary[0], drone_quat.imaginary[1], drone_quat.imaginary[2])
            _, _, yaw = quat_to_euler(quat)

            offset_distance, height_offset = 5.0, 3.0
            forward_x, forward_y = -math.sin(yaw), -math.cos(yaw)

            camera_pos = drone_pos_np.copy()
            camera_pos[0] += forward_x * offset_distance
            camera_pos[1] += forward_y * offset_distance
            camera_pos[2] += height_offset
        except:
            offset_distance, height_offset = 5.0, 3.0
            camera_pos = drone_pos_np.copy()
            camera_pos[0] -= offset_distance
            camera_pos[2] += height_offset

        set_camera_view(eye=camera_pos, target=drone_pos_np)
    except:
        pass


def run_simulation_loop_multi(simulation_app, drone_ctxs: list[RobotDroneAutel],
                              semantic_camera, semantic_camera_prim_path, semantic_map):

    container = get_container()
    world = container.world_configured()
    scene_manager = container.scene_manager()
    
    if not drone_ctxs:
        raise ValueError("run_simulation_loop_multi: empty drone_ctxs list")

    world.reset()
    for _ in range(10):
        simulation_app.update()

    semantic_camera.initialize()

    from omni.kit.viewport.utility import get_viewport_from_window_name
    viewport_manager = container.viewport_manager()
    viewport_manager.register_viewport(name="Viewport", viewport_obj=get_viewport_from_window_name("Viewport"))
    viewport_manager.change_viewport(camera_prim_path=semantic_camera_prim_path, viewport_name="Viewport")

    rate = Rate(1.0 / world.get_rendering_dt())
    grid_size = int(math.ceil(math.sqrt(len(drone_ctxs))))
    for i, ctx in enumerate(drone_ctxs):
        row, col = i // grid_size, i % grid_size
        x_pos = (col - grid_size // 2) * 4.0
        y_pos = (row - grid_size // 2) * 4.0 + 5
        ctx.des_pose = DronePose(pos=Gf.Vec3d(x_pos, y_pos, 2.0), quat=Gf.Quatf.GetIdentity())
        ctx.is_pose_dirty = True
    while simulation_app.is_running():
        rate.sleep()

        for ctx in drone_ctxs:
            rclpy.spin_once(ctx.ros_node, timeout_sec=0.01)

        t_now = drone_ctxs[0].ros_node.get_clock().now()

        for ctx in drone_ctxs:
            if ctx.is_pose_dirty and ctx.des_pose:
                ctx.drone_prim.GetAttribute("xformOp:translate").Set(ctx.des_pose.pos)
                ctx.drone_prim.GetAttribute("xformOp:orient").Set(ctx.des_pose.quat)
                ctx.is_pose_dirty = False

        world.tick()

        for ctx in drone_ctxs:
            header = Header()
            header.stamp = t_now.to_msg()
            header.frame_id = "map"

            msg = ContactsState(header=header)
            if scene_manager.check_prim_collision(prim_path=ctx.prim_path):
                msg.states.append(ContactState(collision1_name=f"drone_{ctx.namespace}"))
            ctx.pubs["collision"].publish(msg)

            if ctx.lidar_list:
                pc_lfr, pc_ubd = ctx.lidar_list[0].get_pointcloud(), ctx.lidar_list[1].get_pointcloud()
                header = Header(stamp=t_now.to_msg(), frame_id="map")
                
                ctx.pubs["lfr_pc"].publish(create_pc2_msg(header, pc_lfr))
                ctx.pubs["ubd_pc"].publish(create_pc2_msg(header, pc_ubd))
                ctx.pubs["lfr_img"].publish(create_image_msg(header, ctx.lidar_list[0].get_depth()))
                ctx.pubs["ubd_img"].publish(create_image_msg(header, ctx.lidar_list[1].get_depth()))

        try:
            import omni.kit.viewport.utility
            viewport = omni.kit.viewport.utility.get_active_viewport()
            if viewport and viewport.updates_enabled:
                update_viewer_camera(scene_manager.stage)
        except:
            pass
