# =============================================================================
# Node Scene Monitor Module - Scene State Monitoring and Publishing
# =============================================================================
#
# This module provides ROS2 node implementation for monitoring scene state
# changes and publishing scene modifications and robot feedback.
#
# =============================================================================

# Local project imports
from log.log_manager import LogManager
from physics_engine.omni_utils import omni
from physics_engine.pxr_utils import Tf, Gf, Usd, UsdGeom, Sdf

# ROS2 imports
from rclpy.node import Node
from geometry_msgs.msg import Transform as RosTransform

# Custom ROS message imports
from gsi_msgs.gsi_msgs_helper import (
    PrimTransform,
    SceneModifications,
    RobotFeedback,
    VelTwistPose,
    RobotSkill,
    PlanExecution,
    SkillExecution,
    SkillFeedback,
)

logger = LogManager.get_logger(__name__)


class NodeSceneMonitor(Node):
    def __init__(self):
        super().__init__("node_scene_monitor")
        self.modification_publisher = self.create_publisher(
            SceneModifications, "/SceneMonitor", 10
        )

        self._stage = omni.usd.get_context().get_stage()

        root = self._stage.GetDefaultPrim()
        if not root or not root.IsValid():
            root = self._stage.GetPrimAtPath("/World")  # Isaac 常见 root
            if not root or not root.IsValid():
                root = self._stage.GetPseudoRoot()  # 兜底

        self._prev_paths = {
            str(p.GetPath()) for p in Usd.PrimRange(root)  # 遍历 root 子树（含 root）
        }
        self._prev_xforms = {p: self._get_local_xform(p) for p in self._prev_paths}

        self._prev_transforms = {
            path: self._get_local_xform(path) for path in self._prev_paths
        }

        self._key = Tf.Notice.Register(
            Usd.Notice.ObjectsChanged, self.on_usd_objects_changed, self._stage
        )

    def _get_local_xform(self, prim_or_path) -> Gf.Matrix4d:
        # 统一成 Usd.Prim
        if isinstance(prim_or_path, Usd.Prim):
            prim = prim_or_path
        else:
            # 支持 str / Sdf.Path
            path = (
                str(prim_or_path)
                if not hasattr(prim_or_path, "pathString")
                else prim_or_path.pathString
            )
            prim = self._stage.GetPrimAtPath(path)

        if not prim or not prim.IsValid():
            return Gf.Matrix4d(1.0)

        xformable = UsdGeom.Xformable(prim)
        if not xformable:
            return Gf.Matrix4d(1.0)

        res = xformable.GetLocalTransformation()
        if isinstance(res, tuple):
            # (matrix, resets) 或 (status, matrix, resets)
            mat = res[0] if len(res) == 3 else res[0]
        else:
            mat = res
        return Gf.Matrix4d(mat)

    def _mat_to_ros(self, mat):
        ros_t = RosTransform()
        if mat is None:
            # 平移保持 0，旋转保持单位四元数(默认)
            return ros_t

        # 平移
        try:
            t = mat.ExtractTranslation()
            ros_t.translation.x = float(t[0])
            ros_t.translation.y = float(t[1])
            ros_t.translation.z = float(t[2])
        except Exception:
            # 极端情况也不抛错
            pass

        # 旋转 —— 多策略尝试，任一成功即可
        q_wxyz = None
        try:
            # 策略 A：有些版本提供直接提取四元数的接口
            q = mat.ExtractRotationQuat()  # 如果不存在会抛异常
            q_wxyz = (float(q.GetReal()),) + tuple(map(float, q.GetImaginary()))
        except Exception:
            try:
                # 策略 B：先拿 3x3，再构造 Gf.Rotation，最后取 Quat
                R3 = mat.ExtractRotationMatrix()  # Gf.Matrix3d
                rot = Gf.Rotation(R3)  # 有些版本支持
                q = rot.GetQuat()
                q_wxyz = (float(q.GetReal()),) + tuple(map(float, q.GetImaginary()))
            except Exception:
                try:
                    # 策略 C：有的绑定是 mat.ExtractRotation() -> Gf.Quatd 或 Gf.Rotation
                    r = mat.ExtractRotation()
                    # 兼容两种返回
                    if hasattr(r, "GetQuat"):
                        q = r.GetQuat()
                    else:
                        q = r
                    q_wxyz = (float(q.GetReal()),) + tuple(map(float, q.GetImaginary()))
                except Exception:
                    # 策略 D：完全兜底——单位四元数，避免整条管线被中断
                    q_wxyz = (1.0, 0.0, 0.0, 0.0)

        ros_t.rotation.w, ros_t.rotation.x, ros_t.rotation.y, ros_t.rotation.z = q_wxyz
        return ros_t

    def _matrices_differ(self, m1, m2, tol=1):
        """简单对比两矩阵是否不同。"""
        if m1 is None or m2 is None:
            return False
        return not m1.AlmostEqual(m2, tol)

    def _pose_delta_is_significant(
        self,
        prev: Gf.Matrix4d,
        curr: Gf.Matrix4d,
        trans_eps: float = 100,  # 2 cm
        rot_eps_deg: float = 100,  # 2°
    ) -> bool:
        """稳健比较：只用位移差和四元数差，任何一步失败都当 0 差，不误报。"""
        if prev is None or curr is None:
            return False

        # --- 位移差：直接比较 prev/curr 的 translation，避免矩阵相减/求逆 ---
        def _safe_trans(m):
            try:
                t = m.ExtractTranslation()
                return float(t[0]), float(t[1]), float(t[2])
            except Exception:
                return 0.0, 0.0, 0.0  # 失败当作 0（不触发）

        px, py, pz = _safe_trans(prev)
        cx, cy, cz = _safe_trans(curr)
        dx = cx - px
        dy = cy - py
        dz = cz - pz
        trans_norm = (dx * dx + dy * dy + dz * dz) ** 0.5

        # --- 旋转差：优先用 ExtractRotationQuat；失败则当 0 度 ---
        def _safe_quat(m):
            # 返回 (w, x, y, z)；失败返回单位 quat
            try:
                q = m.ExtractRotationQuat()  # 有些版本提供
                w = float(q.GetReal())
                vx, vy, vz = q.GetImaginary()
                return w, float(vx), float(vy), float(vz)
            except Exception:
                # 退路：有些版本 ExtractRotation() 直接给 Gf.Quatd 或 Gf.Rotation
                try:
                    r = m.ExtractRotation()
                    if hasattr(r, "GetQuat"):  # Gf.Rotation
                        q = r.GetQuat()
                    else:  # Gf.Quatd
                        q = r
                    w = float(q.GetReal())
                    vx, vy, vz = q.GetImaginary()
                    return w, float(vx), float(vy), float(vz)
                except Exception:
                    # 再不行就单位四元数（表示“无差”）
                    return 1.0, 0.0, 0.0, 0.0

        w1, x1, y1, z1 = _safe_quat(prev)
        w2, x2, y2, z2 = _safe_quat(curr)

        # 相对四元数角度：q_rel = q1^{-1} * q2；角度 = 2*acos(clamp(w_rel, -1, 1))
        # 这里直接用点积等价计算：cos(theta/2) = |dot(q1, q2)|
        dot = w1 * w2 + x1 * x2 + y1 * y2 + z1 * z2
        # 四元数双覆盖：dot 取绝对值
        if dot < 0.0:
            dot = -dot
        # 数值安全
        if dot > 1.0:
            dot = 1.0
        half_angle = 2.0 * (dot**2 - 0.5) ** 0.5 if dot >= (2**-0.5) else None
        # 更稳定：直接用 acos
        import math

        half = math.acos(max(-1.0, min(1.0, dot)))
        rot_deg = 2.0 * half * 180.0 / math.pi

        # 判定（任一超过阈值才算“显著变化”）
        return (trans_norm >= trans_eps) or (rot_deg >= rot_eps_deg)

    def on_usd_objects_changed(self, notice, sender_stage):

        #    print("Changed!!")

        def _to_prim_path_str(p):
            # p 可能是 Sdf.Path 或字符串；属性路径需要转成 Prim 路径
            if isinstance(p, Sdf.Path):
                p = p.GetPrimPath()
            else:
                p = Sdf.Path(str(p)).GetPrimPath()
            return str(p)

        # 1) 归一化路径为 Prim 路径字符串
        resynced = {_to_prim_path_str(p) for p in notice.GetResyncedPaths()}
        info_changed = {_to_prim_path_str(p) for p in notice.GetChangedInfoOnlyPaths()}

        # 2) 选择 root
        root = self._stage.GetDefaultPrim()
        if not root or not root.IsValid():
            root = self._stage.GetPrimAtPath("/")
            if not root or not root.IsValid():
                root = self._stage.GetPseudoRoot()

        # 3) 构建当前 Prim 快照：路径字符串 -> Usd.Prim
        curr_prims = {str(pr.GetPath()): pr for pr in Usd.PrimRange(root)}
        self._curr_paths = set(curr_prims.keys())

        # 4) 计算当前 xform：给 _get_local_xform 传 Prim（不是字符串）
        curr_xforms = {}
        for path, prim in curr_prims.items():
            try:
                xf = self._get_local_xform(prim)  # 注意传 Prim
            except Exception:
                xf = None  # 某些 Prim 没有几何/不支持，忽略
            curr_xforms[path] = xf

        # 5) 取前一帧快照
        prev_paths = getattr(self, "_prev_paths", set())
        prev_xforms = getattr(self, "_prev_xforms", {})

        # 6) 集合差异（都是 Prim 路径字符串）
        added = [p for p in resynced if (p in self._curr_paths and p not in prev_paths)]
        deleted = [
            p for p in resynced if (p not in self._curr_paths and p in prev_paths)
        ]

        # 只对 Prim 路径做 xform 检查
        xform_changed = []
        for p in info_changed:
            cx = curr_xforms.get(p)
            px = prev_xforms.get(p)
            if cx is None or px is None:
                continue
            # 用“相对位姿 + 阈值”来判定；异常内部已处理，不会误报
            if self._pose_delta_is_significant(px, cx, trans_eps=5, rot_eps_deg=1000):
                xform_changed.append(p)

        # 7) 没有任何修改就只更新快照并返回
        if not (added or deleted or xform_changed):
            self._prev_paths = self._curr_paths
            self._prev_xforms = curr_xforms
            return

        # 8) 构建消息（仅非空时发布）
        msg = SceneModifications()

        for p in added:
            item = PrimTransform()
            item.change_type = PrimTransform.ADD
            item.prim_path = p
            item.transform = self._mat_to_ros(curr_xforms.get(p))
            msg.modifications.append(item)

        for p in deleted:
            item = PrimTransform()
            item.change_type = PrimTransform.DELETE
            item.prim_path = p
            item.transform = self._mat_to_ros(prev_xforms.get(p))  # 可能为 None
            msg.modifications.append(item)

        for p in xform_changed:
            item = PrimTransform()
            item.change_type = PrimTransform.TRANSFORM_CHANGED
            item.prim_path = p
            item.transform = self._mat_to_ros(curr_xforms.get(p))
            msg.modifications.append(item)

        if msg.modifications:
            self.modification_publisher.publish(msg)
            print(msg)

        # 9) 更新快照
        self._prev_paths = self._curr_paths
        self._prev_xforms = curr_xforms
