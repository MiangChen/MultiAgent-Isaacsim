def plan_exploration_waypoints(**kwargs):
    robot = kwargs.get("robot")
    polygon_coords = kwargs.get("polygon_coords")
    holes = kwargs.get("holes", [])
    lane_width = kwargs.get("lane_width", 1.0)
    robot_radius = kwargs.get("robot_radius", 0.2)
    sweep_deg = kwargs.get("sweep_deg", 0.0)
    min_seg = kwargs.get("min_seg", 0.5)
    z_out = kwargs.get("z_out", 0.0)
    frame_id = kwargs.get("frame_id", "map")

    # 插值参数
    interpolation_distance = kwargs.get("interpolation_distance", 0.05)  # 5cm
    interpolation_method = kwargs.get(
        "interpolation_method", "linear"
    )  # linear, spline, adaptive

    import numpy as np
    from shapely.geometry import Polygon, LineString, MultiLineString
    from shapely.affinity import rotate
    from nav_msgs.msg import Odometry, Path
    from nav2_msgs.action import ComputePathToPose
    from geometry_msgs.msg import PoseStamped, Quaternion
    from .path_interpolation import interpolate_path_with_fixed_distance

    def _to_xy(seq):
        """把输入统一转换为二维点序列 [(x,y), ...]，兼容 list/np/tensor。"""
        if seq is None:
            return None

        # 兼容 torch.Tensor 但不强依赖 torch
        if hasattr(seq, "detach") and hasattr(seq, "cpu"):
            try:
                seq = seq.detach().cpu().numpy()
            except Exception:
                pass

        if isinstance(seq, np.ndarray):
            seq = np.asarray(seq)
            if seq.ndim == 1:
                if seq.shape[0] >= 2:
                    return [(float(seq[0]), float(seq[1]))]
                raise ValueError("输入点维度不足 2")
            elif seq.ndim == 2:
                return [(float(p[0]), float(p[1])) for p in seq]
            else:
                raise ValueError(f"numpy 输入形状不支持: {seq.shape}")

        out = []
        for p in seq:
            if hasattr(p, "__len__"):
                x, y = float(p[0]), float(p[1])
                out.append((x, y))
            else:
                raise ValueError(f"无法解析点: {p}")
        return out

    # ---- 1) 输入统一成 2D ----
    outer_xy = _to_xy(polygon_coords)
    if not outer_xy or len(outer_xy) < 3:
        raise ValueError("外边界必须至少是三点多边形。")

    # 注意：原代码里不小心把 holes 清空了；这里修正为保留 holes
    holes_xy = []
    if holes:
        holes_xy = [_to_xy(h) for h in holes if _to_xy(h) and len(_to_xy(h)) >= 3]

    poly = Polygon(outer_xy, holes=holes_xy)
    shrunk = poly.buffer(-robot_radius)
    if shrunk.is_empty:
        raise ValueError("区域太窄或 robot_radius 过大，收缩后为空。")

    # ---- 2) 扫掠坐标系旋转 ----
    rot = rotate(shrunk, -sweep_deg, origin="centroid", use_radians=False)
    minx, miny, maxx, maxy = rot.bounds

    # ---- 3) 竖直切片 ----
    segments_per_strip = []
    x = minx
    ray = max(maxy - miny, maxx - minx) * 2.0

    while x <= maxx + 1e-9:
        cutter = LineString([(x, miny - ray), (x, maxy + ray)])
        inter = rot.intersection(cutter)

        lines = []
        if isinstance(inter, LineString):
            lines = [inter]
        elif isinstance(inter, MultiLineString):
            lines = list(inter.geoms)
        else:
            try:
                lines = [g for g in inter.geoms if isinstance(g, LineString)]
            except Exception:
                lines = []

        lines = [ln for ln in lines if ln.length >= min_seg]
        lines.sort(key=lambda l: (l.coords[0][1] + l.coords[-1][1]) / 2.0)
        segments_per_strip.append(lines)
        x += lane_width

    # ---- 4) 蛇形串接 ----
    path_pts_2d = []
    reverse = False
    for lines in segments_per_strip:
        col_pts = []
        for ln in lines:
            p0, p1 = list(ln.coords)[0], list(ln.coords)[-1]
            col_pts.extend([p1, p0] if reverse else [p0, p1])
        if col_pts:
            path_pts_2d.extend(col_pts)
        reverse = not reverse

    if not path_pts_2d:
        raise ValueError(
            "没有生成任何覆盖航点，请检查 lane_width/robot_radius/区域大小。"
        )

    # ---- 5) 旋回原坐标系并去抖 ----
    back = rotate(
        LineString(path_pts_2d), sweep_deg, origin=poly.centroid, use_radians=False
    )
    waypoints_2d = list(back.coords)

    simplified_2d = [waypoints_2d[0]]
    for p in waypoints_2d[1:]:
        if np.hypot(p[0] - simplified_2d[-1][0], p[1] - simplified_2d[-1][1]) > 1e-3:
            simplified_2d.append(p)

    # ---- 6) 路径插值（确保点间距约为指定距离）----

    # 执行插值
    interpolated_2d = interpolate_path_with_fixed_distance(
        simplified_2d,
        target_distance=interpolation_distance,
        method=interpolation_method,
    )

    # ---- 7) 构造 Path 消息（z = z_out，默认 0；朝向为单位四元数）----
    path_msg = Path()
    now = robot.get_clock().now().to_msg() if hasattr(robot, "get_clock") else None
    path_msg.header.frame_id = frame_id
    if now:
        path_msg.header.stamp = now

    for xv, yv in interpolated_2d:
        ps = PoseStamped()
        ps.header.frame_id = frame_id
        if now:
            ps.header.stamp = now
        ps.pose.position.x = float(xv)
        ps.pose.position.y = float(yv)
        ps.pose.position.z = float(z_out)  # 默认 0

        # 朝向：xyzw = (0,0,0,1)
        ps.pose.orientation.x = 0.0
        ps.pose.orientation.y = 0.0
        ps.pose.orientation.z = 0.0
        ps.pose.orientation.w = 1.0

        path_msg.poses.append(ps)

    return path_msg
