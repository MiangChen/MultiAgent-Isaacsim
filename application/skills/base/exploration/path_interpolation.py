# =============================================================================
# Path Interpolation Module - Advanced Path Interpolation for Coverage Planning
# =============================================================================
#
# This module provides various path interpolation methods to ensure smooth
# and dense waypoint generation for robot navigation and coverage tasks.
#
# =============================================================================

import numpy as np
import math
from typing import List, Tuple, Optional
from scipy.interpolate import interp1d, splprep, splev
from log.log_manager import LogManager

logger = LogManager.get_logger(__name__)


def interpolate_path_linear(
    waypoints: List[Tuple[float, float]], target_distance: float = 0.05
) -> List[Tuple[float, float]]:
    """
    线性插值路径，确保相邻点间距约为指定距离

    Args:
        waypoints: 原始路径点列表 [(x1, y1), (x2, y2), ...]
        target_distance: 目标间距（米），默认5cm

    Returns:
        插值后的路径点列表
    """
    if len(waypoints) < 2:
        return waypoints

    interpolated_points = [waypoints[0]]  # 起始点

    for i in range(1, len(waypoints)):
        start_point = waypoints[i - 1]
        end_point = waypoints[i]

        # 计算线段长度
        dx = end_point[0] - start_point[0]
        dy = end_point[1] - start_point[1]
        segment_length = math.sqrt(dx * dx + dy * dy)

        # 如果线段长度小于等于目标距离，直接添加终点
        if segment_length <= target_distance:
            interpolated_points.append(end_point)
            continue

        # 计算需要插入的点数
        num_segments = int(math.ceil(segment_length / target_distance))

        # 插入中间点
        for j in range(1, num_segments):
            t = j / num_segments  # 插值参数 [0, 1]
            interp_x = start_point[0] + t * dx
            interp_y = start_point[1] + t * dy
            interpolated_points.append((interp_x, interp_y))

        # 添加线段终点
        interpolated_points.append(end_point)

    return interpolated_points


def interpolate_path_spline(
    waypoints: List[Tuple[float, float]],
    target_distance: float = 0.05,
    smoothing: float = 0.0,
) -> List[Tuple[float, float]]:
    """
    样条插值路径，生成更平滑的轨迹

    Args:
        waypoints: 原始路径点列表
        target_distance: 目标间距（米）
        smoothing: 平滑参数，0为无平滑

    Returns:
        插值后的路径点列表
    """
    if len(waypoints) < 3:  # 样条插值至少需要3个点
        return interpolate_path_linear(waypoints, target_distance)

    try:
        # 提取x, y坐标
        x_coords = [p[0] for p in waypoints]
        y_coords = [p[1] for p in waypoints]

        # 计算总路径长度
        total_length = 0
        for i in range(1, len(waypoints)):
            dx = waypoints[i][0] - waypoints[i - 1][0]
            dy = waypoints[i][1] - waypoints[i - 1][1]
            total_length += math.sqrt(dx * dx + dy * dy)

        # 计算需要的总点数
        num_points = int(math.ceil(total_length / target_distance)) + 1

        # 样条插值
        tck, u = splprep(
            [x_coords, y_coords], s=smoothing, k=min(3, len(waypoints) - 1)
        )
        u_new = np.linspace(0, 1, num_points)
        x_new, y_new = splev(u_new, tck)

        # 转换为坐标对列表
        interpolated_points = [(float(x), float(y)) for x, y in zip(x_new, y_new)]

        logger.debug(
            f"Spline interpolation: {len(waypoints)} -> {len(interpolated_points)} points"
        )
        return interpolated_points

    except Exception as e:
        logger.warning(f"Spline interpolation failed: {e}, falling back to linear")
        return interpolate_path_linear(waypoints, target_distance)


def interpolate_path_adaptive(
    waypoints: List[Tuple[float, float]],
    target_distance: float = 0.05,
    curvature_threshold: float = 0.1,
) -> List[Tuple[float, float]]:
    """
    自适应插值，在弯曲处增加更多点

    Args:
        waypoints: 原始路径点列表
        target_distance: 目标间距（米）
        curvature_threshold: 曲率阈值，超过此值增加点密度

    Returns:
        插值后的路径点列表
    """
    if len(waypoints) < 3:
        return interpolate_path_linear(waypoints, target_distance)

    interpolated_points = [waypoints[0]]

    for i in range(1, len(waypoints) - 1):
        prev_point = waypoints[i - 1]
        curr_point = waypoints[i]
        next_point = waypoints[i + 1]

        # 计算曲率（简化版本）
        curvature = calculate_curvature(prev_point, curr_point, next_point)

        # 根据曲率调整目标距离
        if curvature > curvature_threshold:
            adaptive_distance = target_distance * 0.5  # 弯曲处密度加倍
        else:
            adaptive_distance = target_distance

        # 插值当前线段
        segment_points = interpolate_segment(prev_point, curr_point, adaptive_distance)
        interpolated_points.extend(segment_points[1:])  # 跳过起始点避免重复

    # 处理最后一段
    if len(waypoints) > 1:
        last_segment = interpolate_segment(
            waypoints[-2], waypoints[-1], target_distance
        )
        interpolated_points.extend(last_segment[1:])

    return interpolated_points


def calculate_curvature(
    p1: Tuple[float, float], p2: Tuple[float, float], p3: Tuple[float, float]
) -> float:
    """
    计算三点间的曲率

    Args:
        p1, p2, p3: 连续的三个点

    Returns:
        曲率值
    """
    # 向量
    v1 = (p2[0] - p1[0], p2[1] - p1[1])
    v2 = (p3[0] - p2[0], p3[1] - p2[1])

    # 向量长度
    len1 = math.sqrt(v1[0] ** 2 + v1[1] ** 2)
    len2 = math.sqrt(v2[0] ** 2 + v2[1] ** 2)

    if len1 < 1e-6 or len2 < 1e-6:
        return 0.0

    # 单位向量
    u1 = (v1[0] / len1, v1[1] / len1)
    u2 = (v2[0] / len2, v2[1] / len2)

    # 计算角度变化（简化的曲率）
    dot_product = u1[0] * u2[0] + u1[1] * u2[1]
    dot_product = max(-1.0, min(1.0, dot_product))  # 限制范围

    angle_change = math.acos(abs(dot_product))
    avg_length = (len1 + len2) / 2

    return angle_change / avg_length if avg_length > 0 else 0.0


def interpolate_segment(
    start: Tuple[float, float], end: Tuple[float, float], target_distance: float
) -> List[Tuple[float, float]]:
    """
    插值单个线段

    Args:
        start: 起始点
        end: 终止点
        target_distance: 目标间距

    Returns:
        插值后的点列表（包含起始和终止点）
    """
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    segment_length = math.sqrt(dx * dx + dy * dy)

    if segment_length <= target_distance:
        return [start, end]

    num_segments = int(math.ceil(segment_length / target_distance))
    points = [start]

    for i in range(1, num_segments):
        t = i / num_segments
        x = start[0] + t * dx
        y = start[1] + t * dy
        points.append((x, y))

    points.append(end)
    return points


def interpolate_path_with_fixed_distance(
    waypoints: List[Tuple[float, float]],
    target_distance: float = 0.05,
    method: str = "linear",
) -> List[Tuple[float, float]]:
    """
    路径插值的统一接口

    Args:
        waypoints: 原始路径点
        target_distance: 目标间距
        method: 插值方法 ("linear", "spline", "adaptive")

    Returns:
        插值后的路径点
    """
    if method == "spline":
        return interpolate_path_spline(waypoints, target_distance)
    elif method == "adaptive":
        return interpolate_path_adaptive(waypoints, target_distance)
    else:  # default to linear
        return interpolate_path_linear(waypoints, target_distance)
