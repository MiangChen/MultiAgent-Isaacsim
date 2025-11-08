try:
    import pydevd_pycharm
    pydevd_pycharm.settrace("localhost", port=12345, stdout_to_server=True, stderr_to_server=True)
except:
    pass

import math
import time
import rclpy
from gazebo_msgs.msg import ContactsState, ContactState
from std_msgs.msg import Header


class Rate:
    def __init__(self, frequency):
        self.period = 1.0 / frequency
        self.start_time = time.time()

    # @carb.profiler.profile
    def sleep(self):
        elapsed = time.time() - self.start_time
        sleep_duration = self.period - elapsed
        if sleep_duration > 0:
            time.sleep(sleep_duration)
        self.start_time = time.time()


def run_simulation_loop(simulation_app, world, drone_ctxs, semantic_camera, 
                       semantic_camera_prim_path, semantic_map, scene_manager, viewport_manager):
    world.reset()
    for _ in range(10):
        simulation_app.update()

    semantic_camera.initialize()

    from omni.kit.viewport.utility import get_viewport_from_window_name
    viewport_manager.register_viewport(name="Viewport", viewport_obj=get_viewport_from_window_name("Viewport"))
    viewport_manager.change_viewport(camera_prim_path=semantic_camera_prim_path, viewport_name="Viewport")

    rate = Rate(1.0 / world.get_rendering_dt())

    grid_size = int(math.ceil(math.sqrt(len(drone_ctxs))))

    from physics_engine.pxr_utils import Gf

    from robot.robot_drone_autel import DronePose
    for i, ctx in enumerate(drone_ctxs):
        row, col = i // grid_size, i % grid_size
        x_pos = (col - grid_size // 2) * 4.0
        y_pos = (row - grid_size // 2) * 4.0 + 5
        ctx.des_pose = DronePose(pos=Gf.Vec3d(x_pos, y_pos, 2.0), quat=Gf.Quatd.GetIdentity())
        ctx.is_pose_dirty = True

    while simulation_app.is_running():
        rate.sleep()

        for ctx in drone_ctxs:
            rclpy.spin_once(ctx.ros_node, timeout_sec=0.01)

        t_now = drone_ctxs[0].ros_node.get_clock().now()

        for ctx in drone_ctxs:
            if ctx.is_pose_dirty and ctx.des_pose and ctx.drone_prim:
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

                from simulation_utils.message_convert import create_pc2_msg, create_image_msg
                ctx.pubs["lfr_pc"].publish(create_pc2_msg(header, pc_lfr))
                ctx.pubs["ubd_pc"].publish(create_pc2_msg(header, pc_ubd))
                ctx.pubs["lfr_img"].publish(create_image_msg(header, ctx.lidar_list[0].get_depth()))
                ctx.pubs["ubd_img"].publish(create_image_msg(header, ctx.lidar_list[1].get_depth()))





def create_car_objects(scene_manager, map_semantic):
    scale = [2, 5, 1.0]
    cubes_config = {
        "car0": {
            "shape_type": "cuboid",
            "prim_path": "/World/car0",
            "scale": scale,
            "name": "car0_LKN1111_pink",
            "position": [11.6, 3.5, 0],
            "color": [255, 255, 255],
            "entity_type": "visual",
        },
        "car1": {
            "shape_type": "cuboid",
            "prim_path": "/World/car1",
            "scale": scale,
            "name": "car1_ZN3J3W_blue",
            "position": [0.3, 3.5, 0],
            "color": [255, 255, 255],
            "entity_type": "visual",
        },
        "car2": {
            "shape_type": "cuboid",
            "prim_path": "/World/car2",
            "scale": scale,
            "name": "car2_JN3839_yellow",
            "position": [-13.2, 3.5, 0],
            "color": [255, 255, 255],
            "entity_type": "visual",
        },
        "car3": {
            "shape_type": "cuboid",
            "prim_path": "/World/car3",
            "name": "car3_QBZ666_black",
            "scale": scale,
            "position": [-7.1, 10, 0],
            "color": [255, 255, 255],
            "entity_type": "visual",
        },
        "car4": {
            "shape_type": "cuboid",
            "prim_path": "/World/car4",
            "name": "car4_PN3S39_white",
            "scale": scale,
            "position": [-0.9, 30, 0],
            "orientation": [0.707, 0, 0, 0.707],
            "color": [255, 255, 255],
            "entity_type": "visual",
        },
    }

    for config in cubes_config.values():
        result = scene_manager.create_shape_unified(**config)
        if result.get("status") == "success":
            map_semantic.add_semantic(prim_path=result.get("result"), semantic_label="car")


def main():
    from containers import get_container, reset_container
    from config.config_manager import config_manager
    from log.log_manager import LogManager
    import rclpy
    
    rclpy.init(args=None)
    logger = LogManager.get_logger(__name__)

    reset_container()
    container = get_container()
    container.wire(modules=[__name__])


    config_manager = container.config_manager()
    log_manager = container.log_manager()
    server = container.server()
    ros_manager = container.ros_manager()
    scene_manager = container.scene_manager()
    grid_map = container.grid_map()
    semantic_map = container.semantic_map()
    viewport_manager = container.viewport_manager()
    
    world = container.world_configured()
    simulation_app = server.get_simulation_app()
    
    ros_manager.start()

    scene_manager.load_scene(usd_path=config_manager.get("world_usd_path"), prim_path_root="/World/Scene")
    create_car_objects(scene_manager, semantic_map)

    result = scene_manager.add_camera(translation=[1, 4, 2], orientation=[1, 0, 0, 0])
    camera_result = result.get("result")

    from simulation_utils.ros_bridge import setup_ros
    
    # Create drones using blueprint - CARLA style (same as main_example.py)
    blueprint_library = world.get_blueprint_library()
    drone_bp = blueprint_library.find('robot.autel')
    
    drone_ctxs = []
    for idx, ns in enumerate(config_manager.get("namespace")):
        drone_bp.set_attribute('type', 'autel')
        drone_bp.set_attribute('id', idx)
        drone_bp.set_attribute('namespace', ns)
        drone_bp.set_attribute('color_scheme_id', idx)
        drone_bp.set_attribute('disable_gravity', True)
        
        drone = world.spawn_actor(drone_bp)
        drone.setup_lidar_and_ros(setup_ros)
        drone.initialize()
        drone_ctxs.append(drone)

    # Run simulation loop
    run_simulation_loop(simulation_app, world, drone_ctxs, camera_result.get("camera_instance"),
                       camera_result.get("prim_path"), semantic_map, scene_manager, viewport_manager)


if __name__ == "__main__":
    main()
