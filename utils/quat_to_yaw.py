from typing import Tuple

from math import atan2

from isaacsim.core.utils.rotations import quat_to_rot_matrix


def quat_to_yaw(quaternion: Tuple[float, float, float, float]) -> float:
    """
    Args:
        quaternion: 四元数, 顺序是 wxyz
    Returns:
        yaw: 绕着z轴的旋转角度
    """

    matrix = quat_to_rot_matrix(quaternion)
    yaw = atan2(matrix[1, 0], matrix[0, 0])
    return yaw
