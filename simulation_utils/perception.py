import numpy as np

import carb

from map.map_semantic_map import MapSemantic


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


def process_semantic_detection(semantic_camera, map_semantic: MapSemantic) -> None:
    """
    Process semantic detection and car pose extraction using injected dependencies.

    Args:
        semantic_camera: Semantic camera instance
        map_semantic: Injected semantic map instance
    """
    try:
        current_frame = semantic_camera.get_current_frame()
        if current_frame and "bounding_box_2d_loose" in current_frame:
            result = current_frame["bounding_box_2d_loose"]
            print("get bounding box 2d loose", result)
            if result:
                car_prim, car_pose = map_semantic.get_prim_and_pose_by_semantic(
                    result, "car"
                )
                if car_prim is not None and car_pose is not None:
                    print("get car prim and pose\n", car_prim, "\n", car_pose)
                else:
                    print("No car detected in current frame")
            else:
                print("No bounding box data available")
        else:
            print("No frame data or bounding box key available")
    except Exception as e:
        print(f"Error getting semantic camera data: {e}")
