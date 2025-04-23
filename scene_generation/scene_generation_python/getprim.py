import omni.usd

class SceneInfo:
    def __init__(self):
        self.prim_info = []

    def get_prim_info(self):
        """
        Traverse Current Stage. Obtain all basic info of Prims in the stage
        """
        try:
            stage = omni.usd.get_context().get_stage()
            if not stage:
                print("[Scene Info] No USD stage is open.")
                return []

            for prim in stage.Traverse():
                try:
                    info = {
                        "path": prim.GetPath().pathString,
                        "name": prim.GetName(),
                        "type": prim.GetTypeName(),
                        "is_active": prim.IsActive(),
                        "is_defined": prim.IsDefined(),
                        "has_children": prim.HasChildren(),
                        "world_transform": omni.usd.get_world_transform_matrix(prim),
                        "doc": prim.GetMetadata("doc"),
                    }
                    self.prim_info.append(info)
                except Exception as e:
                    print(f"[Scene Info] Failed to extract info for prim: {prim.GetPath()} — {e}")
                    continue

            return self.prim_info

        except Exception as e:
            print(f"[Scene Info] Unexpected error while getting prims: {e}")
            return []