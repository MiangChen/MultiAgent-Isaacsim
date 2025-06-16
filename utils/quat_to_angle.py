import numpy as np
# 使用scipy的简化版本（推荐）
from scipy.spatial.transform import Rotation as R


def quaternion_to_euler_scipy(quaternion, format='xyzw', euler_order='xyz', degrees=True):
    """
    使用scipy实现的四元数到欧拉角转换（更稳定）
    """
    q = np.array(quaternion)

    # 转换为scipy期望的格式 (x, y, z, w)
    if format.lower() == 'wxyz':
        scipy_quat = [q[1], q[2], q[3], q[0]]  # wxyz -> xyzw
    elif format.lower() == 'xyzw':
        scipy_quat = q
    else:
        raise ValueError("格式必须是 'wxyz' 或 'xyzw'")

    # 创建旋转对象
    rotation = R.from_quat(scipy_quat)

    # 转换为欧拉角
    angles = rotation.as_euler(euler_order.lower(), degrees=degrees)

    return angles[0], angles[1], angles[2]

if __name__ == '__main__':
    quat = [0.003503232728689909, 0.8555620908737183, 0.22366458177566528, -0.46687784790992737]
    quat = [-0.028, 0.688, 0.150, -0.71]
    quat = [-0.030, 0.679, 0.148, -0.719]  # xyzw格式的quat
    for i in ['xyz', ]:
        print("in ", i)
        # print(quaternion_to_euler(quat, format='xyzw', euler_order=i))
        print(quaternion_to_euler_scipy(quat, 'xyzw', euler_order=i))
