# from isaacsim.core.utils.semantics import add_update_semantics, remove_all_semantics, get_semantics, \
#     check_missing_semantics, check_incorrect_semantics, count_semantics_in_scene
# import isaacsim.core.utils.prims as prims_utils

# Apply a semantic label to a prim or update an existing label

# add_update_semantics(
# prim: pxr.Usd.Prim,
# semantic_label: str,
# type_label: str = 'class',
# suffix='',
# ) → None
#
# prim_path = '/World/simple_city/city/Cube_01'
# prim = prims_utils.get_prim_at_path(prim_path)
#
# remove_all_semantics(prim)
#
# semantic_label = 'house'
# type_label = 'class'
# add_update_semantics(prim, semantic_label, type_label, suffix=type_label)
#
# semantic_label = '1'
# type_label = 'instance'
# add_update_semantics(prim, semantic_label, type_label, suffix=type_label)
#
# print(get_semantics(prim))
# print(type(prim))
# from pxr import Usd, UsdGeom, Gf, Sdf
#
#
# def get_world_pose(prim: Usd.Prim, time_code=Usd.TimeCode.Default()) -> Gf.Matrix4d:
#     # 检查 Prim 是否有效
#     if not prim.IsValid():
#         raise ValueError("Invalid Prim")
#
#     # 转换为 Xformable
#     xform = UsdGeom.Xformable(prim)
#     if not xform:
#         # 如果 Prim 不支持变换，返回单位矩阵或抛出异常
#         print("no xform")
#         return Gf.Matrix4d(1.0)
#
#     # 计算并返回世界变换矩阵
#     return xform.ComputeLocalToWorldTransform(time_code)
#
#
# world_matrix = get_world_pose(prim)
# translation = world_matrix.ExtractTranslation()
# print(world_matrix)
# print(translation)
# # prim.get_world_poses()

from typing import List

import numpy as np
import omni
# from isaacsim.asset.gen.omap.bindings import _omap


from typing import Dict, Any, Optional, List, Tuple
# Isaac Sim / Omniverse related imports
from isaacsim.core.prims import XFormPrim
from pxr import Usd

# Other standard library imports
import yaml


class MapSemantic():
    def __init__(self):
        """
        Initializes the MapSemantic class by loading a semantic mapping from a config file.
        """
        try:
            with open('files/env_cfg.yaml', 'r') as f:
                cfg = yaml.safe_load(f)
            # Assumes the config file contains the 'map_semantic' key
            self.map_semantic = cfg['map_semantic']
        except FileNotFoundError:
            raise FileNotFoundError("Error: Configuration file 'files/env_cfg.yaml' not found.")
        except KeyError:
            raise KeyError("Error: The key 'map_semantic' is missing from the config file 'files/env_cfg.yaml'.")
        except Exception as e:
            raise RuntimeError(f"An unexpected error occurred while loading or parsing the config file: {e}") from e

    def get_prim_and_pose_by_semantic(
            self,
            sensor_result: Dict[str, Any],
            target_semantic_class: str
    ) -> Tuple[Usd.Prim, Dict[str, List[float]]]:
        """
        Parses a sensor result for a specific semantic label and returns its Prim and world pose.

        This function iterates through the detection results, finds the first object that matches
        the `target_semantic_class`, then extracts its prim path to retrieve its USD Prim
        and its pose (position and orientation) in the world frame.

        Args:
            sensor_result (Dict[str, Any]): The result dictionary returned by a sensor,
                                            containing 'data' and 'info' keys.
            target_semantic_class (str): The target semantic class to search for, e.g., 'car'.

        Raises:
            KeyError: If the input `sensor_result` dictionary has a missing or incomplete
                      structure (e.g., missing 'info', 'data' keys).
            ValueError: If the target semantic class is not found in the result's label map,
                        or if no instance of the target class is found among the detected objects.
            RuntimeError: If the pose of the found prim cannot be retrieved.

        Returns:
            Tuple[Usd.Prim, Dict[str, List[float]]]: A tuple containing two elements:
                - Pxr.Usd.Prim: The underlying USD Prim object of the found entity.
                - Dict[str, List[float]]: The pose dictionary of the object,
                                          in the format {'position': [...], 'orientation': [...]}.
        """
        try:
            # Extract necessary data structures from the sensor result
            info = sensor_result['info']
            data = sensor_result['data']
            id_to_labels = info['idToLabels']
            prim_paths = info['primPaths']
        except KeyError as e:
            raise KeyError(f"The input sensor_result dictionary is missing a required key: {e}") from e

        # --- Step 1: Find the semanticId corresponding to the target class ---
        target_semantic_id = None
        for semantic_id, label_data in id_to_labels.items():
            if label_data.get('class') == target_semantic_class:
                target_semantic_id = int(semantic_id)
                break

        if target_semantic_id is None:
            return ValueError(
                f"The semantic class '{target_semantic_class}' was not found in the sensor result's label map.")

        # --- Step 2: Iterate through detected objects to find a matching semanticId ---
        for i, detected_object in enumerate(data):
            if detected_object['semanticId'] == target_semantic_id:
                try:
                    prim_path = prim_paths[i]
                except IndexError:
                    raise IndexError(
                        f"Found object with semantic class '{target_semantic_class}', but its index {i} is out of bounds for the prim_paths list.")

                # --- Step 3: Get the Prim and its pose ---
                try:
                    # Use the high-level API to easily get the prim and its pose
                    xform_prim = XFormPrim(prim_path)
                    position, orientation = xform_prim.get_world_poses()
                    # Get the underlying Usd.Prim from the high-level wrapper
                    usd_prim = xform_prim.prims[0]
                    if not usd_prim.IsValid():
                        return RuntimeError(
                            "Successfully created an XFormPrim, but could not retrieve a valid underlying Usd.Prim.")

                    pose = {
                        "position": position.tolist(),
                        "orientation": orientation.tolist()  # orientation is [w, x, y, z]
                    }
                    # If successful, return the prim and pose, then exit the function
                    return usd_prim, pose

                except Exception as e:
                    # Wrap any exception that occurs during pose retrieval and raise it
                    return RuntimeError(
                        f"Successfully found Prim '{prim_path}', but failed to retrieve its pose: {e}")

        # If the loop completes without finding any matching instance
        return ValueError(
            f"Found the semantic class '{target_semantic_class}' in the label map, but no instance of it was detected.")
