# =============================================================================
# Perception Module - Perception Processing Utilities
# =============================================================================
#
# This module provides perception processing utilities to semantic mapping integration.
#
# =============================================================================

# Third-party library imports
import carb

# Local project imports
from map.map_semantic_map import MapSemantic


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
