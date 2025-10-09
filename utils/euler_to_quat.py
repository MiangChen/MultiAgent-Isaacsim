from typing import List
from scipy.spatial.transform import Rotation as R


def euler_to_quat(
    roll: float = 0.0,
    pitch: float = 0.0,
    yaw: float = 0.0,
    degrees: bool = True,
    order: str = "xyz",
    order_quat: str = "wxyz",
) -> List[float]:

    rotation = R.from_euler(order, [roll, pitch, yaw], degrees=degrees)

    quat_xyzw = rotation.as_quat()
    if order_quat == "wxyz":
        w = quat_xyzw[3]
        x = quat_xyzw[0]
        y = quat_xyzw[1]
        z = quat_xyzw[2]
        return [w, x, y, z]
    elif order_quat == "xyzw":
        return quat_xyzw
