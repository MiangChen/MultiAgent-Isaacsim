from typing import List

from quat_to_angle import quaternion_to_euler_scipy  # 可以将quat变成角度, 能输入xyz/zxy等顺序
import math
import numpy as np
from scipy.spatial.transform import Rotation as R

"""
这个程序涉及到 slam到仿真 到具体机器人的角度信息; 
没必要看懂怎么处理的, 主要知道几个点, 一个是从quat到angle是否, 建议用scipy计算
第二个是, 角度的叠加要使用旋转矩阵, 并且要确认顺序, 是xyz, zyz还是别的
isaacsim中默认的顺序是zyx

"""

def transform_object_rotate_then_translate(origin_pos, origin_angle,
                                           translate_pos,
                                           rotate,
                                           rotation_order='xyz'):
    """
    对物体进行变换：先旋转，再平移

    返回:
    - new_x, new_y, new_z: 新的位置坐标
    """

    # === 步骤1: 位置变换 - 先旋转原始位置，再平移 ===

    # 创建旋转变换
    rotation_transform = R.from_euler(rotation_order, rotate, degrees=True)

    # 对原始位置进行旋转
    rotated_position = rotation_transform.apply(origin_pos)

    # 再进行平移
    new_x = rotated_position[0] + translate_pos[0]
    new_y = rotated_position[1] + translate_pos[1]
    new_z = rotated_position[2] + translate_pos[2]

    # === 步骤2: 姿态变换 - 组合旋转 ===

    # 创建原始旋转对象
    original_rotation = R.from_euler('xyz', origin_angle, degrees=True)

    # 组合旋转：先应用变换旋转，再应用原始旋转
    combined_rotation = rotation_transform * original_rotation
    angle = combined_rotation.as_euler(rotation_order, degrees=True)
    return [new_x, new_y, new_z], angle


def extract_yaw_from_rotation(rotation):
    """从旋转对象中提取yaw角"""
    R_matrix = rotation.as_matrix()
    yaw = np.arctan2(R_matrix[1, 0], R_matrix[0, 0])
    return np.degrees(yaw)


def calculate_robot_yaw(rotate_map: List = None, orien_robot: List = None) -> float:
    # 地图校正变换
    map_correction = R.from_euler('xyz', rotate_map, degrees=True)

    # 机器人原始姿态
    robot_original = R.from_euler('xyz', orien_robot, degrees=True)

    # 校正机器人姿态
    robot_corrected = map_correction * robot_original

    # 提取yaw角
    yaw = extract_yaw_from_rotation(robot_corrected)

    return yaw


def calculate_angle_with_y_axis(robot_pose: list, angle_unit='degrees'):
    """
    通过向量计算与y轴的夹角

    Args:
        robot_pose: 机器人的位置坐标 [x, y, z] 或 [x, y]
        angle_unit: 角度单位，'degrees' 或 'radians'

    Returns:
        angle: 与y轴正方向的夹角，顺时针为正，逆时针为负
    """
    # 将机器人位置向量转换为2D向量 (只考虑x, y分量)
    robot_vector = np.array([robot_pose[0], robot_pose[1]])

    # 检查是否为零向量
    if np.linalg.norm(robot_vector) == 0:
        return 0.0  # 零向量情况下返回0

    # 归一化机器人位置向量
    robot_direction = robot_vector / np.linalg.norm(robot_vector)

    # y轴正方向向量
    y_axis = np.array([0, 1])

    # 计算叉积来判断旋转方向 (2D叉积返回标量)
    cross_product = np.cross(y_axis, robot_direction)

    # 计算点积来获取角度大小
    dot_product = np.dot(y_axis, robot_direction)

    # 使用反余弦函数计算角度
    angle_rad = math.acos(np.clip(dot_product, -1, 1))

    # 根据叉积的符号确定旋转方向
    # 从y轴到robot_direction的旋转：叉积>0表示逆时针，叉积<0表示顺时针
    if cross_product < 0:
        angle_rad = -angle_rad  # 逆时针为负
    else:
        angle_rad = angle_rad  # 顺时针为正

    if angle_unit == 'degrees':
        return math.degrees(angle_rad)
    else:
        return angle_rad


# === 使用示例 ===
# --- 输入参数 ---
# 机器人的原始信息
pos = [10.518024444580078, -3.9358742237091064, 15.300127029418945]
pos = [0, 0, 0]
quat = [-0.015559980645775795, 0.8029360175132751, 0.21828925609588623, 0.5544393658638]  # xyzw格式的quat
# quat = [-0.010639474727213383, 0.6564741134643555, 0.17408470809459686, 0.7339093089103699]
quat = [0, 0, 0, 1]  # xyzw格式的quat

# 坐标系变化的数值
translate_pos = [-4, -6, 0.7]
# 旋转为xyz顺序
rotate = [-104, 0, 8]

if __name__ == "__main__":
    # === 方法1: 只返回新的位置和四元数 ===
    angle = quaternion_to_euler_scipy(quat, 'xyzw', euler_order='xyz')

    new_pos, new_angle = transform_object_rotate_then_translate(
        pos, angle,
        translate_pos,
        rotate,
        rotation_order='xyz',
    )

    yaw = calculate_robot_yaw(rotate_map=rotate, orien_robot=angle)

    target_pos = [-10, 0, 0]
    vector = [target_pos[0] - new_pos[0], target_pos[1] - new_pos[1]]
    vector_angle = calculate_angle_with_y_axis(vector)
    print("=== 变换结果 ===")
    print(f"新位置:   ({new_pos[0]:.3f}, {new_pos[1]:.3f}, {new_pos[2]:.3f})")
    print(f"新的角度: ({new_angle[0]:.3f}, {new_angle[1]:.3f}, {new_angle[2]:.3f})")
    print("yaw", yaw)
    print("vector", vector_angle)
