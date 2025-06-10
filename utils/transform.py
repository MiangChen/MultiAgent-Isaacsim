# --- 输入参数 ---
# 原始坐标和角度
x_orig = 9.033  # 示例值, 你可以替换成你的实际值
y_orig = -3.929
z_orig = 15.292
quat = [-0.030, 0.679, 0.148, -0.719]  # xyzw

# 坐标系变化的增量
delta_x = 0.0
delta_y = 0.0
delta_z = 0.81696
rotate_angle_x = -104.637  # 度
rotate_angle_y = -7.276  # 度
rotate_angle_z = -1.894  # 度

import numpy as np
import math


def quaternion_to_euler(quaternion, format='xyzw', euler_order='xyz', degrees=True):
    """
    将四元数转换为欧拉角

    参数:
    - quaternion: 四元数，格式为列表或numpy数组
    - format: 四元数格式，'wxyz' 或 'xyzw'
    - euler_order: 欧拉角旋转顺序，默认'xyz'，也可以是'zyx', 'yxz'等
    - degrees: 是否返回角度制，True为角度，False为弧度

    返回:
    - (angle_x, angle_y, angle_z): 欧拉角
    """

    # 确保输入是numpy数组
    q = np.array(quaternion)

    # 根据格式提取w, x, y, z
    if format.lower() == 'wxyz':
        w, x, y, z = q[0], q[1], q[2], q[3]
    elif format.lower() == 'xyzw':
        x, y, z, w = q[0], q[1], q[2], q[3]
    else:
        raise ValueError("格式必须是 'wxyz' 或 'xyzw'")

    # 归一化四元数
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm == 0:
        raise ValueError("四元数不能为零向量")

    w, x, y, z = w / norm, x / norm, y / norm, z / norm

    # 根据欧拉角顺序进行转换
    if euler_order.lower() == 'xyz':
        angle_x, angle_y, angle_z = _quat_to_euler_xyz(w, x, y, z)
    else:
        raise ValueError("不支持的欧拉角顺序")

    # 转换为角度制（如果需要）
    if degrees:
        angle_x = math.degrees(angle_x)
        angle_y = math.degrees(angle_y)
        angle_z = math.degrees(angle_z)

    return angle_x, angle_y, angle_z


def _quat_to_euler_xyz(w, x, y, z):
    """四元数转欧拉角 - XYZ顺序"""
    # Roll (X轴旋转)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (Y轴旋转)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # 使用90度如果超出范围
    else:
        pitch = math.asin(sinp)

    # Yaw (Z轴旋转)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def euler_to_rotation_matrix(angle_x, angle_y, angle_z, degrees=True):
    """
    将欧拉角转换为旋转矩阵 (XYZ顺序)
    """
    if degrees:
        angle_x = math.radians(angle_x)
        angle_y = math.radians(angle_y)
        angle_z = math.radians(angle_z)

    # 创建各轴的旋转矩阵
    # X轴旋转矩阵
    Rx = np.array([
        [1, 0, 0],
        [0, math.cos(angle_x), -math.sin(angle_x)],
        [0, math.sin(angle_x), math.cos(angle_x)]
    ])

    # Y轴旋转矩阵
    Ry = np.array([
        [math.cos(angle_y), 0, math.sin(angle_y)],
        [0, 1, 0],
        [-math.sin(angle_y), 0, math.cos(angle_y)]
    ])

    # Z轴旋转矩阵
    Rz = np.array([
        [math.cos(angle_z), -math.sin(angle_z), 0],
        [math.sin(angle_z), math.cos(angle_z), 0],
        [0, 0, 1]
    ])

    # 组合旋转矩阵 (XYZ顺序): R = Rz * Ry * Rx
    R = np.dot(Rz, np.dot(Ry, Rx))
    return R


def rotation_matrix_to_euler(R, degrees=True):
    """
    将旋转矩阵转换为欧拉角 (XYZ顺序)
    """
    # 提取欧拉角
    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    if degrees:
        x = math.degrees(x)
        y = math.degrees(y)
        z = math.degrees(z)

    return x, y, z


def coordinate_transform(x, y, z, angle_x, angle_y, angle_z,
                         delta_x, delta_y, delta_z,
                         rotate_angle_x, rotate_angle_y, rotate_angle_z):
    """
    进行坐标系变换：先平移，再旋转

    参数:
    - x, y, z: 原始位置坐标
    - angle_x, angle_y, angle_z: 原始角度 (度)
    - delta_x, delta_y, delta_z: 平移量
    - rotate_angle_x, rotate_angle_y, rotate_angle_z: 旋转角度 (度)

    返回:
    - new_x, new_y, new_z: 新的位置坐标
    - new_angle_x, new_angle_y, new_angle_z: 新的角度
    """

    # 步骤1: 先进行平移
    translated_x = x + delta_x
    translated_y = y + delta_y
    translated_z = z + delta_z

    # 步骤2: 创建旋转矩阵
    rotation_matrix = euler_to_rotation_matrix(rotate_angle_x, rotate_angle_y, rotate_angle_z)

    # 步骤3: 对平移后的位置进行旋转
    position_vector = np.array([translated_x, translated_y, translated_z])
    rotated_position = np.dot(rotation_matrix, position_vector)

    new_x, new_y, new_z = rotated_position[0], rotated_position[1], rotated_position[2]

    # 步骤4: 处理角度变换
    # 获取原始角度的旋转矩阵
    original_rotation_matrix = euler_to_rotation_matrix(angle_x, angle_y, angle_z)

    # 应用新的旋转
    combined_rotation_matrix = np.dot(rotation_matrix, original_rotation_matrix)

    # 转换回欧拉角
    new_angle_x, new_angle_y, new_angle_z = rotation_matrix_to_euler(combined_rotation_matrix)

    return new_x, new_y, new_z, new_angle_x, new_angle_y, new_angle_z


# 示例使用
if __name__ == "__main__":
    # 原始坐标和角度
    x, y, z = x_orig, y_orig, z_orig

    angle_x, angle_y, angle_z = quaternion_to_euler(quaternion=quat)

    print(f"原始坐标: ({x:.3f}, {y:.3f}, {z:.3f})")
    print(f"原始角度: ({angle_x:.3f}°, {angle_y:.3f}°, {angle_z:.3f}°)")
    print()
    print(f"变换参数:")
    print(f"  平移: ({delta_x:.3f}, {delta_y:.3f}, {delta_z:.3f})")
    print(f"  旋转: ({rotate_angle_x:.3f}°, {rotate_angle_y:.3f}°, {rotate_angle_z:.3f}°)")
    print()

    # 进行坐标变换
    new_x, new_y, new_z, new_angle_x, new_angle_y, new_angle_z = coordinate_transform(
        x, y, z, angle_x, angle_y, angle_z,
        delta_x, delta_y, delta_z,
        rotate_angle_x, rotate_angle_y, rotate_angle_z
    )

    print(f"变换后坐标: ({new_x:.3f}, {new_y:.3f}, {new_z:.3f})")
    print(f"变换后角度: ({new_angle_x:.3f}°, {new_angle_y:.3f}°, {new_angle_z:.3f}°)")
    # print()
    # print(f"坐标变化量: ({new_x - x:.3f}, {new_y - y:.3f}, {new_z - z:.3f})")
    # print(f"角度变化量: ({new_angle_x - angle_x:.3f}°, {new_angle_y - angle_y:.3f}°, {new_angle_z - angle_z:.3f}°)")
