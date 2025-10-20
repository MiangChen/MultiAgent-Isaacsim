from typing import List

from typing import Dict, Any, Tuple, Optional

from physics_engine.isaacsim_utils import XFormPrim
from physics_engine.pxr_utils import Usd


class MapSemantic:
    def __init__(self):
        """
        Initializes the MapSemantic class by loading a semantic mapping from a config file.
        """
        try:
            from config.config_manager import config_manager

            self.map_semantic = config_manager.get("map_semantic")
        except Exception as e:
            raise RuntimeError(
                f"An unexpected error occurred while loading or parsing the config file: {e}"
            ) from e

    def get_prim_and_pose_by_semantic(
        self, sensor_result: Dict[str, Any], target_semantic_class: str
    ) -> Tuple[Optional[Usd.Prim], Optional[Dict[str, List[float]]]]:
        """
        Parses a sensor result for a specific semantic label and returns its Prim and world pose.

        This function iterates through the detection results, finds the first object that matches
        the `target_semantic_class`, then extracts its prim path to retrieve its USD Prim
        and its pose (position and orientation) in the world frame.

        Args:
            sensor_result (Dict[str, Any]): The result dictionary returned by a sensor,
                                            containing 'data' and 'info' keys.
            target_semantic_class (str): The target semantic class to search for, e.g., 'car'.

        Returns:
            Tuple[Optional[Usd.Prim], Optional[Dict[str, List[float]]]]: A tuple containing two elements:
                - Optional[Pxr.Usd.Prim]: The underlying USD Prim object of the found entity, or None if not found.
                - Optional[Dict[str, List[float]]]: The pose dictionary of the object,
                                          in the format {'position': [...], 'orientation': [...]}, or None if not found.
        """

        # Extract necessary data structures from the sensor result
        info = sensor_result["info"]
        data = sensor_result["data"]
        id_to_labels = info["idToLabels"]
        prim_paths = info["primPaths"]

        # --- Step 1: Find the semanticId corresponding to the target class ---
        target_semantic_id = None
        for semantic_id, label_data in id_to_labels.items():
            if label_data.get("class") == target_semantic_class:
                target_semantic_id = int(semantic_id)
                break

        if target_semantic_id is None:
            print(
                f"Warning: The semantic class '{target_semantic_class}' was not found in the sensor result's label map."
            )
            return None, None

        # --- Step 2: Iterate through detected objects to find a matching semanticId ---
        for i, detected_object in enumerate(data):
            if detected_object["semanticId"] == target_semantic_id:
                try:
                    prim_path = prim_paths[i]
                except IndexError:
                    print(
                        f"Error: Found object with semantic class '{target_semantic_class}', but its index {i} is out of bounds for the prim_paths list."
                    )
                    return None, None

                # --- Step 3: Get the Prim and its pose ---
                try:
                    # Use the high-level API to easily get the prim and its pose
                    xform_prim = XFormPrim(prim_path)
                    position, orientation = xform_prim.get_world_poses()
                    # Get the underlying Usd.Prim from the high-level wrapper
                    usd_prim = xform_prim.prims[0]
                    if not usd_prim.IsValid():
                        print(
                            f"Error: Successfully created an XFormPrim, but could not retrieve a valid underlying Usd.Prim."
                        )
                        return None, None

                    pose = {
                        "position": position.tolist(),
                        "orientation": orientation.tolist(),  # orientation is [w, x, y, z]
                    }
                    # If successful, return the prim and pose, then exit the function
                    return usd_prim, pose

                except Exception as e:
                    # Handle any exception that occurs during pose retrieval
                    print(
                        f"Error: Successfully found Prim '{prim_path}', but failed to retrieve its pose: {e}"
                    )
                    return None, None

        # If the loop completes without finding any matching instance
        print(
            f"Warning: Found the semantic class '{target_semantic_class}' in the label map, but no instance of it was detected."
        )
        return None, None
