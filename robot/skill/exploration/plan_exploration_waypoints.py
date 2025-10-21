def plan_exploration_waypoints_skill(**kwargs):
    robot = kwargs.get("robot")
    polygon_coords = kwargs.get("polygon_coords")
    holes = kwargs.get("holes")
    lane_width = kwargs.get("lane_width", 1.0)
    robot_radius = kwargs.get("robot_radius", 0.2)
    sweep_deg = kwargs.get("sweep_deg", 0.0)
    min_seg = kwargs.get("min_seg", 0.5)
    z_out = kwargs.get("z_out", 0.0)

    result = {"success": False, "message": "", "data": None}

    import torch
    import numpy as np
    from shapely.geometry import Polygon, LineString, MultiLineString
    from shapely.affinity import rotate

    def _to_xy(seq):
        """
        把输入统一转换为二维点序列 [(x,y), ...]
        兼容 list / tuple / np.ndarray / torch.Tensor / warp.array。
        第三维(例如z)会被忽略。
        """
        if seq is None:
            return None

        # torch / warp / numpy 自动转为 ndarray
        if isinstance(seq, torch.Tensor):
            seq = seq.detach().cpu().numpy()
        # numpy array 情况
        if isinstance(seq, np.ndarray):
            seq = np.asarray(seq)
            if seq.ndim == 1:
                # 单个点
                if seq.shape[0] >= 2:
                    return [(float(seq[0]), float(seq[1]))]
                else:
                    raise ValueError("输入点维度不足 2")
            elif seq.ndim == 2:
                # N×2 或 N×3
                return [(float(p[0]), float(p[1])) for p in seq]
            else:
                raise ValueError("numpy 输入形状不支持: " + str(seq.shape))

        # 普通 list / tuple
        out = []
        for p in seq:
            # p 可能是 list / tuple / np.array / tensor / warp.array
            if hasattr(p, "__len__"):
                x, y = float(p[0]), float(p[1])
                out.append((x, y))
            else:
                raise ValueError(f"无法解析点: {p}")
        return out

    # ---- 1) 输入统一成 2D ----
    outer_xy = _to_xy(polygon_coords)
    if not holes:
        holes = []
    else:
        holes = []
    holes_xy = [_to_xy(h) for h in holes]

    poly = Polygon(outer_xy, holes=holes_xy)
    shrunk = poly.buffer(-robot_radius)
    if shrunk.is_empty:
        result["message"] = "区域太窄或 robot_radius 过大，收缩后为空"
        return result

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

        # 过滤碎片并按中心 y 排序
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
        result["message"] = (
            "没有生成任何覆盖航点，请检查 lane_width/robot_radius/区域大小"
        )
        return result

    # ---- 5) 旋回原坐标系并转 3D ----
    back = rotate(
        LineString(path_pts_2d), sweep_deg, origin=poly.centroid, use_radians=False
    )
    waypoints_2d = list(back.coords)

    # 去抖
    simplified_2d = [waypoints_2d[0]]
    for p in waypoints_2d[1:]:
        if np.hypot(p[0] - simplified_2d[-1][0], p[1] - simplified_2d[-1][1]) > 1e-3:
            simplified_2d.append(p)

    # 输出 3D（z 固定为 z_out，默认 0）
    waypoints = [(x, y, float(z_out)) for (x, y) in simplified_2d]

    result["success"] = True
    result["message"] = f"Generated {len(waypoints)} waypoints"
    result["data"] = waypoints

    return result
