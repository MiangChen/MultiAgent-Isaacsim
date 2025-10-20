from typing import List

import numpy as np
from scipy.spatial.transform import Rotation as R


def quat_to_yaw(quat_xyzw: List[List[float]]) -> np.ndarray:
    """
    Args:
        quat_xyzw: 四元数, 顺序是 wxyz
    Returns:
        yaw: 绕着z轴的旋转角度
    """

    if not quat_xyzw:
        return np.array([])

    quat_array = np.array(quat_xyzw)

    rotations = R.from_quat(quat_array)
    euler_angles = rotations.as_euler("zyx", degrees=False)
    raw_yaw_angles = euler_angles[:, 0]

    yaw = np.unwrap(raw_yaw_angles)

    return yaw
