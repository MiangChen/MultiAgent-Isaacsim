import numpy as np
from scipy.spatial.transform import Rotation as R


def quat_to_euler(quaternion, format="wxyz", euler_order="xyz", degrees=True):
    q = np.array(quaternion)

    # 转换为scipy期望的格式 (x, y, z, w)
    if format.lower() == "wxyz":
        scipy_quat = [q[1], q[2], q[3], q[0]]  # wxyz -> xyzw
    elif format.lower() == "xyzw":
        scipy_quat = q
    else:
        raise ValueError("格式必须是 'wxyz' 或 'xyzw'")

    rotation = R.from_quat(scipy_quat)
    angles = rotation.as_euler(euler_order.lower(), degrees=degrees)

    return angles[0], angles[1], angles[2]


if __name__ == "__main__":
    quat = [
        0.003503232728689909,
        0.8555620908737183,
        0.22366458177566528,
        -0.46687784790992737,
    ]
    quat = [-0.028, 0.688, 0.150, -0.71]
    quat = [-0.030, 0.679, 0.148, -0.719]  # xyzw格式的quat
    for i in [
        "xyz",
    ]:
        print("in ", i)
        # print(quaternion_to_euler(quat, format='xyzw', euler_order=i))
        print(quat_to_euler(quat, "xyzw", euler_order=i))
