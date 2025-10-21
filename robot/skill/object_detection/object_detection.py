def object_detection_skill(semantic_camera, map_semantic, target_class: str):
    result = {"success": False, "message": "", "data": None}

    
    current_frame = semantic_camera.get_current_frame()
    bounding_box_result = current_frame["bounding_box_2d_loose"]
    prim_target, target_pose = map_semantic.get_prim_and_pose_by_semantic(
        bounding_box_result, target_semantic_class=target_class
    )
    
    if prim_target and target_pose:
        result["success"] = True
        result["message"] = f"Detected {target_class}"
        result["data"] = {"prim_path": prim_target, "pose": target_pose}
    else:
        result["message"] = "No detection"
    
    return result
