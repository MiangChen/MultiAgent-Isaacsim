from isaacsim.core.utils.semantics import add_update_semantics, remove_all_semantics, get_semantics, \
    check_missing_semantics, check_incorrect_semantics, count_semantics_in_scene
import isaacsim.core.utils.prims as prims_utils

# Apply a semantic label to a prim or update an existing label

# add_update_semantics(
# prim: pxr.Usd.Prim,
# semantic_label: str,
# type_label: str = 'class',
# suffix='',
# ) → None

prim_path = '/World/simple_city/city/Cube_01'
prim = prims_utils.get_prim_at_path(prim_path)

remove_all_semantics(prim)

semantic_label = 'house'
type_label = 'class'
add_update_semantics(prim, semantic_label, type_label, suffix=type_label)

semantic_label = '1'
type_label = 'instance'
add_update_semantics(prim, semantic_label, type_label, suffix=type_label)

print(get_semantics(prim))
print(type(prim))
from pxr import Usd, UsdGeom, Gf, Sdf


def get_world_pose(prim: Usd.Prim, time_code=Usd.TimeCode.Default()) -> Gf.Matrix4d:
    # 检查 Prim 是否有效
    if not prim.IsValid():
        raise ValueError("Invalid Prim")

    # 转换为 Xformable
    xform = UsdGeom.Xformable(prim)
    if not xform:
        # 如果 Prim 不支持变换，返回单位矩阵或抛出异常
        print("no xform")
        return Gf.Matrix4d(1.0)

    # 计算并返回世界变换矩阵
    return xform.ComputeLocalToWorldTransform(time_code)


world_matrix = get_world_pose(prim)
translation = world_matrix.ExtractTranslation()
print(world_matrix)
print(translation)
# prim.get_world_poses()
