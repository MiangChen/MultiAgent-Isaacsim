import asyncio
from typing import Dict
import traceback
import threading

from geometry_msgs.msg import Transform as RosTransform
from rclpy.action import ActionServer, ActionClient, GoalResponse
from rclpy.node import Node
from rclpy.task import Future as RclpyFuture

import omni.usd
from pxr import Tf, Gf

from action_msgs.msg import GoalStatus
from gsi2isaacsim.gsi_msgs_helper import (
    PrimTransform,
    SceneModifications,
    RobotFeedback,
    VelTwistPose,
    RobotSkill,
    PlanExecution,
    SkillExecution,
    SkillFeedback,
)
from log.log_manager import LogManager

logger = LogManager.get_logger(__name__)


class PlanNode(Node):
    def __int__(self):
        super().__init__("PlanNode")


class SceneMonitorNode(Node):
    def __init__(self):
        super().__init__("SceneMonitor")
        self.modification_publisher = self.create_publisher(
            SceneModifications, "/SceneModification", 10
        )

        self._stage = omni.usd.get_context().get_stage()

        from pxr import Usd  # 放到文件顶部的 imports

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
        from pxr import UsdGeom, Gf, Usd

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
        from pxr import Gf

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

        from pxr import Usd, Sdf

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


class SwarmNode(Node):

    def __init__(self):
        super().__init__("swarm")
        # 结构: { robot_class: { robot_id: { "motion": publisher, "feedback": publisher } } }
        self.publisher_dict: dict[str, dict[int, dict[str, any]]] = {}
        self.subscriber_dict: dict[str, dict[int, dict[str, any]]] = {}

    def register_feedback_publisher(self, robot_class: str, robot_id: int, qos=10):
        """
        注册一个 feedback publisher
        topic 规则: /feedback/<robot_class>_<robot_id>
        """
        # 如果类别不存在，先建字典
        if robot_class not in self.publisher_dict:
            self.publisher_dict[robot_class] = {}
        # 如果 robot_id 不存在，先建子字典
        if robot_id not in self.publisher_dict[robot_class]:
            self.publisher_dict[robot_class][robot_id] = {}

        topic = f"/feedback/{robot_class}_{robot_id}"
        pub = self.create_publisher(RobotFeedback, topic, qos)

        # 存到子字典里
        self.publisher_dict[robot_class][robot_id]["feedback"] = pub

        #        self.get_logger().info(
        #            f"Registered feedback publisher for {robot_class}[{robot_id}] on topic {topic}"
        #        )
        return pub

    def register_motion_publisher(self, robot_class: str, robot_id: int, qos=50):
        """
        注册一个 motion publisher
        topic 规则: /motion/<robot_class>_<robot_id>
        """
        if robot_class not in self.publisher_dict:
            self.publisher_dict[robot_class] = {}
        if robot_id not in self.publisher_dict[robot_class]:
            self.publisher_dict[robot_class][robot_id] = {}

        topic = f"/motion/{robot_class}_{robot_id}"
        pub = self.create_publisher(VelTwistPose, topic, qos)

        self.publisher_dict[robot_class][robot_id]["motion"] = pub

        #        self.get_logger().info(
        #            f"Registered motion publisher for {robot_class}[{robot_id}] on topic {topic}"
        #        )
        return pub

    def register_cmd_subscriber(
        self, robot_class: str, robot_id: int, callback=None, qos=50
    ):
        """
        注册一个cmd subscriber
        topic 规则： /cmd/<robot_class>_<robot_id>
        """
        if robot_class not in self.subscriber_dict:
            self.subscriber_dict[robot_class] = {}
        # 如果 robot_id 不存在，先建子字典
        if robot_id not in self.subscriber_dict[robot_class]:
            self.subscriber_dict[robot_class][robot_id] = {}

        topic = f"/cmd/{robot_class}_{robot_id}"
        sub = self.create_subscription(VelTwistPose, topic, callback, qos)

        self.subscriber_dict[robot_class][robot_id]["cmd"] = sub

        #        self.get_logger().info(
        #            f"Registered cmd subscriber for {robot_class}[{robot_id}] on topic {topic}"
        #        )
        return sub

    def publish_feedback(self, robot_class: str, robot_id: int, msg: RobotFeedback):
        self.publisher_dict[robot_class][robot_id]["feedback"].publish(msg)

    def publish_motion(self, robot_class: str, robot_id: int, msg: VelTwistPose):
        self.publisher_dict[robot_class][robot_id]["motion"].publish(msg)

    def update_swarm_data(self):
        self.shared_data["robot_count"] = len(self.robot_positions)
        self.shared_data["formation_type"] = "triangle"
        self.publish_data("robot_count", len(self.robot_positions))
        self.publish_data("formation_type", "triangle")

    def get_robot_positions(self):
        return self.robot_positions

    def request_map_info(self):
        """请求地图信息"""

        def handle_map_response(value):
            logger.info(f"Received map size: {value}")

        self.query_node_data("map", "map_size", handle_map_response)


class PlanExecutionServer(Node):
    """
    并行的任务调度动作服务器（调度器/Orchestrator）。
    """

    def __init__(self, loop):
        super().__init__("plan_execution_server")
        self.loop = loop  # Store the main event loop
        self._action_server = ActionServer(
            self,
            PlanExecution,
            "/isaac_sim/plan_execution",
            execute_callback=self.execute_callback_wrapper,
        )

        self._skill_clients: Dict[str, ActionClient] = {}
        self._client_lock = threading.Lock()
        logger.info("✅ Parallel Plan Dispatch Server is ready.")

    # This function is called by the ROS executor in a worker thread.
    def execute_callback_wrapper(self, goal_handle):
        """
        Schedules the async implementation on the main event loop and waits for the result.
        """

        # Schedule the coroutine on the main event loop
        future = asyncio.run_coroutine_threadsafe(
            self.async_execute_callback(goal_handle), self.loop
        )

        # Block and wait for the result from the asyncio thread
        return future.result()

    async def async_execute_callback(self, goal_handle):
        """
        This is the original async logic, now guaranteed to run in the main thread.
        """
        plan = goal_handle.request.plan
        logger.info(f"Dispatching plan with {len(plan.steps)} timesteps...")

        for step in plan.steps:
            logger.info(f"--- Starting Timestep {step.timestep} ---")

            # Initialize feedback state for this timestep
            timestep_feedback_state: Dict[str, SkillFeedback] = {}
            for robot_skill in step.robots:
                skill_id = f"{robot_skill.robot_id}-{robot_skill.skill_list[0].skill}-{id(robot_skill)}"
                timestep_feedback_state[skill_id] = SkillFeedback(
                    skill_id=skill_id, status="Pending"
                )

            # Concurrently execute all skills for the current timestep
            tasks = [
                self.dispatch_skill(
                    goal_handle,
                    robot_skill,
                    step.timestep,
                    f"{robot_skill.robot_id}-{robot_skill.skill_list[0].skill}-{id(robot_skill)}",
                    timestep_feedback_state,
                )
                for robot_skill in step.robots
            ]
            results = await asyncio.gather(*tasks, return_exceptions=True)

            # Check if any task failed or raised an exception
            if not all(res is True for res in results):
                # Log any exceptions that occurred
                for i, res in enumerate(results):
                    if isinstance(res, Exception):
                        logger.error(f"Exception during skill dispatch: {res}")
                        traceback.print_exc()

                error_msg = f"Execution failed in timestep {step.timestep}."
                logger.error(error_msg)
                goal_handle.abort()
                return PlanExecution.Result(success=False, message=error_msg)

            logger.info(f"--- Timestep {step.timestep} Completed Successfully ---")

        goal_handle.succeed()
        logger.info("✅ Plan dispatched and executed successfully.")
        return PlanExecution.Result(success=True, message="Plan executed successfully.")

    async def dispatch_skill(
        self,
        plan_goal_handle,
        robot_skill: RobotSkill,
        current_timestep: int,
        skill_id: str,
        feedback_state: dict,
    ):
        skill_name = robot_skill.skill_list[0].skill
        action_name = f"/{skill_name}"
        logger.info(f"Dispatching skill '{skill_id}' on action '{action_name}'")

        try:
            action_client = self._get_or_create_client(action_name)
            if not await self._wait_for_server(action_client, timeout_sec=3.0):
                raise RuntimeError(f"Action server '{action_name}' not available.")

            skill_goal = SkillExecution.Goal(skill_request=robot_skill)

            def skill_feedback_callback(feedback_msg):
                status = feedback_msg.feedback.status
                if skill_id in feedback_state:
                    feedback_state[skill_id].status = status
                    self._publish_aggregated_feedback(
                        plan_goal_handle, current_timestep, feedback_state
                    )

            send_goal_future = action_client.send_goal_async(
                skill_goal, feedback_callback=skill_feedback_callback
            )
            goal_handle = await self._wrap_ros_future(send_goal_future)

            if not goal_handle.accepted:
                raise RuntimeError(f"Skill '{skill_id}' was rejected by the server.")

            logger.info(f"Skill '{skill_id}' accepted by server.")
            feedback_state[skill_id].status = "Executing"
            self._publish_aggregated_feedback(
                plan_goal_handle, current_timestep, feedback_state
            )

            result_wrapper = await self._wrap_ros_future(goal_handle.get_result_async())

            if result_wrapper.status != GoalStatus.STATUS_SUCCEEDED:
                raise RuntimeError(
                    f"Skill '{skill_id}' did not succeed. Status: {result_wrapper.status}"
                )

            if result_wrapper.result.success:
                logger.info(f"Skill '{skill_id}' completed successfully.")
                feedback_state[skill_id].status = "Success"
                self._publish_aggregated_feedback(
                    plan_goal_handle, current_timestep, feedback_state
                )
                return True
            else:
                raise RuntimeError(
                    f"Skill '{skill_id}' failed with message: {result_wrapper.result.message}"
                )

        except Exception as e:
            logger.error(f"Exception for skill '{skill_id}': {type(e).__name__} - {e}")
            feedback_state[skill_id].status = f"Error: {e}"
            self._publish_aggregated_feedback(
                plan_goal_handle, current_timestep, feedback_state
            )
            # Re-raise the exception so asyncio.gather can catch it
            raise

    def _get_or_create_client(self, action_name: str) -> ActionClient:
        with self._client_lock:
            if action_name not in self._skill_clients:
                logger.info(f"Creating new action client for '{action_name}'...")
                self._skill_clients[action_name] = ActionClient(
                    self, SkillExecution, action_name
                )
            return self._skill_clients[action_name]

    def _publish_aggregated_feedback(
        self, plan_goal_handle, current_timestep, feedback_state
    ):
        agg_feedback = PlanExecution.Feedback()
        agg_feedback.current_timestep = current_timestep
        agg_feedback.skill_statuses = list(feedback_state.values())
        if plan_goal_handle.is_active:
            plan_goal_handle.publish_feedback(agg_feedback)

    async def _wait_for_server(
        self, action_client: ActionClient, timeout_sec=2.0
    ) -> bool:
        """Asynchronously wait for an Action Server to be available."""
        # This function is now called from the main asyncio thread, so we can run
        # the blocking wait_for_server call in the loop's default executor (a thread pool).
        return await self.loop.run_in_executor(
            None, lambda: action_client.wait_for_server(timeout_sec=timeout_sec)
        )

    async def _wrap_ros_future(self, rclpy_future: RclpyFuture):
        """Wrap an rclpy.Future in an asyncio.Future."""
        # This function is also now called from the main asyncio thread.
        aio_future = self.loop.create_future()

        def on_done(ros_future):
            try:
                result = ros_future.result()
                self.loop.call_soon_threadsafe(aio_future.set_result, result)
            except Exception as e:
                self.loop.call_soon_threadsafe(aio_future.set_exception, e)

        rclpy_future.add_done_callback(on_done)
        return await aio_future


class SkillServerNode(Node):
    """
    这个节点模拟了多个技能的执行。
    它会为每个指定的技能名称启动一个专用的Action Server。
    """

    def __init__(self):
        super().__init__("skill_server_node")
        self.skill_servers = []

        # 定义我们想要模拟的技能列表
        skill_names = ["navigate", "load_object", "unload_object", "take_picture"]

        for skill_name in skill_names:
            server = ActionServer(
                self,
                SkillExecution,
                f"/{skill_name}",  # Action topic is named after the skill
                self.execute_callback,
            )
            self.skill_servers.append(server)
            logger.info(f"✅ Mock Action Server for '/{skill_name}' is ready.")

    def execute_callback(self, goal_handle):
        """
        模拟技能执行的通用回调函数。
        """
        try:
            skill_req = goal_handle.request.skill_request
            skill_name = skill_req.skill_list[0].skill
            params_list = skill_req.skill_list[0].params
            params_dict = {param.key: param.value for param in params_list}
            robot = skill_req.robot_id
            robot_name, robot_id = robot.split("-")
            robot_id = int(robot_id)
            logger.info(
                f"[{skill_name.upper()}] Robot '{robot}' received request. Simulating execution..."
            )

            feedback_msg = SkillExecution.Feedback()

            # 使用skill manager
            from containers import get_container

            container = get_container()

            skill_manager = container.skill_manager()

            if robot_name.lower() in ["ugv", "jetbot"]:
                skill_manager._SKILL_TABLE.get(skill_name)(
                    rc="jetbot", rid=robot_id, params=params_dict
                )

            elif robot_name.lower() in ["uav", "drone", "cf2x"]:
                skill_manager._SKILL_TABLE.get(skill_name)(
                    rc="cf2x", rid=robot_id, params=params_dict
                )

            feedback_msg.status = f"Robot '{robot_id}' executing '{skill_name}"
            logger.info(
                f"Feedback from [{skill_name.upper()}|{robot_id}]: {feedback_msg.status}"
            )
            goal_handle.publish_feedback(feedback_msg)

            goal_handle.succeed()
            logger.info(
                f"[{skill_name.upper()}] Robot '{robot_id}' finished successfully ✅"
            )
            result = SkillExecution.Result()
            result.success = True
            result.message = f"'{skill_name}' completed successfully by '{robot_id}'."

            logger.info(
                f"✅ [{skill_name.upper()}] Execution finished for robot '{robot_id}'."
            )
            return result
        except Exception as e:
            logger.error(f"!!!!!! EXCEPTION IN SKILL EXECUTION CALLBACK !!!!!!")
            logger.error(f"Skill: {skill_req.skill_list[0].skill}, Robot: {skill_req.robot_id}")
            logger.error(f"Error Type: {type(e).__name__}")
            logger.error(f"Error Message: {e}")
