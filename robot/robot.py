# =============================================================================
# Robot Module - Core Robot Implementation
# =============================================================================
#
# This module provides the base Robot class and related functionality for
# robotic simulation and control within the Isaac Sim environment.
#
# =============================================================================

# Standard library imports
import threading
from typing import Tuple, Dict, Any, List

# Third-party library imports
import numpy as np
import torch
from shapely.geometry import Polygon, LineString, MultiLineString
from shapely.affinity import rotate

# Local project imports
from log.log_manager import LogManager
from navigation.node_path_planner_ompl import NodePlannerOmpl
from navigation.node_trajectory_generator import NodeTrajectoryGenerator
from navigation.node_controller_mpc import NodeMpcController
from physics_engine.isaacsim_utils import (
    Scene,
    RigidPrim,
    prims_utils,
    create_viewport_for_camera,
    set_camera_view,
    Camera,
)
from physics_engine.pxr_utils import UsdPhysics
from robot.sensor.camera import CfgCamera, CfgCameraThird, Camera
from robot.cfg import CfgRobot
from robot.body import BodyRobot
from robot.robot_trajectory import Trajectory
from ros.node_robot import NodeRobot
from scene.scene_manager import SceneManager
from utils import to_torch

# ROS2 imports
from rclpy.executors import MultiThreadedExecutor
from nav_msgs.msg import Odometry

# Custom ROS message imports
from gsi_msgs.gsi_msgs_helper import (
    RobotFeedback,
    SkillExecution,
    SkillInfo,
    Parameter,
    VelTwistPose,
)

logger = LogManager.get_logger(__name__)


def _get_viewport_manager_from_container():
    """
    Get viewport manager from dependency injection container.

    This function provides a fallback mechanism to obtain the viewport manager
    when it's not directly injected into the constructor.

    Returns:
        ViewportManager: The viewport manager instance from the container
    """
    try:
        from containers import get_container

        container = get_container()
        return container.viewport_manager()
    except Exception as e:
        raise Exception("ViewportManager not found in container") from e


class Robot:
    def __init__(
        self,
        scene: Scene = None,
        scene_manager: SceneManager = None,
    ):

        self.cfg_dict_camera = self.cfg_robot.cfg_dict_camera
        self.cfg_dict_camera_third = self.cfg_robot.cfg_dict_camera_third
        self.camera = {}
        self.camera_third = {}

        self.scene = scene
        self.scene_manager = scene_manager
        self.viewport_manager = (
            _get_viewport_manager_from_container()
        )  # 通过依赖注入获取viewport_manager

        # 通用的机器人本体初始化代码
        self.cfg_robot.path_prim_robot = (
            self.cfg_robot.path_prim_swarm
            + f"/{self.cfg_robot.type}"
            + f"/{self.cfg_robot.type}_{self.cfg_robot.id}"
        )
        self.cfg_robot.namespace = self.cfg_robot.type + f"_{self.cfg_robot.id}"
        self.namespace = self.cfg_robot.namespace

        self.body: BodyRobot = None

        # 机器人的历史轨迹
        self.trajectory: Trajectory = None
        # 机器人的控制器
        self.controllers: dict = {}  # 用于存储多个控制器, 'controller name': function
        self.control_mode: str = (
            ""  # 'joint_efforts', 'joint_velocities', 'joint_positions', 'joint_indices', 'joint_names'
        )
        self.action: np.ndarray = None
        # 机器人的技能
        self.skills: dict = {}  # 用于记录其技能: 'skill name': function
        # 用于回调函数中
        self.flag_active = False
        self.flag_world_reset: bool = False  # 用来记录下世界是不是被初始化了
        self.flag_action_navigation: bool = False  # 用来记录是不是启动导航了
        # 用于PDDL, 记录当前用的 skill, 之所以用skill, 是为了和action区分, action一般是底层的关节动作
        self.state_skill: str = ""
        self.state_skill_complete: bool = True  # 默认状态, 没有skill要做, 所以是True

        # 布置机器人的相机传感器, 可以有多个相机
        self.cameras: dict = {}
        self.view_angle: float = 2 * np.pi / 3  # 感知视野 弧度
        self.view_radius: float = 2  # 感知半径 米

        # 第三视角相机 一个机器人只有一个
        # self.cfg_camera_third_person = cfg_camera_third_person
        self.viewport_name = None  # 存储viewport名称
        self.relative_camera_pos = np.array([0, 0, 0])  # 默认为0向量
        self.transform_camera_pos = np.array([0, 0, 0])

        # 机器人的ros node
        self.node = NodeRobot(namespace=self.namespace)
        self.node.set_robot_instance(self)
        self.node.callback_execute_skill = self.callback_execute_skill

        self.node_planner_ompl = NodePlannerOmpl(namespace=self.namespace)
        self.node_trajectory_generator = NodeTrajectoryGenerator(
            namespace=self.namespace
        )
        self.node_controller_mpc = NodeMpcController(namespace=self.namespace)

        self.executor = MultiThreadedExecutor()
        self.ros_thread = None
        self.stop_event = threading.Event()
        self._setup_executor()
        self.start_ros()

        # 机器人的任务状态
        self.current_task_id = "0"
        self.current_task_name = "n"

        # 以下三个变量用于记录navigate开始时的机器人位置，用于计算feedback里的progress
        self.nav_begin = None
        self.nav_end = None
        self.nav_dist = 1.0

        # 机器人探索区域状态
        self.is_detecting = False
        self.target_prim = None

        self.track_waypoints = []
        self.track_waypoint_index = 0
        self.is_tracking = False
        self.track_waypoints_sub = None
        self.track_waypoints_sub

    ########################## Skill Action Server ############################
    def callback_execute_skill(self, goal_handle):
        if self.state_skill_complete is False:
            logger.error(
                f"Robot is busy with skill '{self.state_skill}'. Rejecting new goal."
            )
            goal_handle.abort()
            return SkillExecution.Result(success=False, message="Robot is busy.")

        skill_name = goal_handle.request.skill_request.skill_list[0].skill
        self.state_skill_complete = False
        feedback_msg = SkillExecution.Feedback()

        # 模拟一个需要10个步骤的任务
        for i in range(10):
            # 检查是否收到了取消请求
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.state_skill_complete = True  # 重置状态
                logger.info(f"Skill '{skill_name}' was canceled.")
                return SkillExecution.Result(
                    success=False, message="Skill execution was canceled by client."
                )
            # 模拟工作
            import time

            time.sleep(2)
            # 发送反馈
            feedback_msg.status = (
                f"Executing step {i + 1} of 10 for skill '{skill_name}'."
            )
            goal_handle.publish_feedback(feedback_msg)
            logger.info(f"Feedback: {feedback_msg.status}")

        self.state_skill_complete = True
        goal_handle.succeed()
        result = SkillExecution.Result()
        result.success = True
        result.message = f"Skill '{skill_name}' executed successfully."
        return result

    ########################## Publisher Odom  ############################
    def publish_robot_state(self):
        """发布机器人状态到/odom话题"""
        pos, quat = self.body.get_world_pose()  # quat: wxyz
        vel_linear, vel_angular = self.body.get_world_vel()

        pos = pos.detach().cpu().numpy()
        quat = quat.detach().cpu().numpy()
        vel_linear = vel_linear.detach().cpu().numpy()
        vel_angular = vel_angular.detach().cpu().numpy()

        odom_msg = Odometry()
        odom_msg.header.stamp = self.node.get_clock().now().to_msg()
        odom_msg.header.frame_id = "map"  # or "odom"
        odom_msg.child_frame_id = self.namespace

        odom_msg.pose.pose.position.x = float(pos[0])
        odom_msg.pose.pose.position.y = float(pos[1])
        odom_msg.pose.pose.position.z = float(pos[2])

        odom_msg.pose.pose.orientation.x = float(quat[1])
        odom_msg.pose.pose.orientation.y = float(quat[2])
        odom_msg.pose.pose.orientation.z = float(quat[3])
        odom_msg.pose.pose.orientation.w = float(quat[0])

        odom_msg.twist.twist.linear.x = float(vel_linear[0])
        odom_msg.twist.twist.linear.y = float(vel_linear[1])
        odom_msg.twist.twist.linear.z = float(vel_linear[2])

        odom_msg.twist.twist.angular.x = float(vel_angular[0])
        odom_msg.twist.twist.angular.y = float(vel_angular[1])
        odom_msg.twist.twist.angular.z = float(vel_angular[2])

        self.node.publisher_odom.publish(odom_msg)

    ########################## Subscriber Velocity  ############################
    def callback_cmd_vel(self, msg):
        """处理来自ROS的速度命令"""
        linear_vel = torch.tensor([msg.linear.x, msg.linear.y, msg.linear.z])
        angular_vel = torch.tensor([msg.angular.x, msg.angular.y, msg.angular.z])
        print("linear_vel", linear_vel)
        self.controller_simplified(linear_vel, angular_vel)

    ########################## Start ROS  ############################

    def _setup_executor(self):
        self.executor.add_node(self.node)
        self.executor.add_node(self.node_planner_ompl)
        self.executor.add_node(self.node_trajectory_generator)
        self.executor.add_node(self.node_controller_mpc)

    def start_ros(self):
        if self.ros_thread is None or not self.ros_thread.is_alive():
            self.ros_thread = threading.Thread(
                target=self._spin_ros, daemon=True, name=f"ROS_{self.namespace}"
            )
            self.ros_thread.start()
            logger.info(f"Robot {self.namespace} ROS thread started")

    def _spin_ros(self):
        try:
            while not self.stop_event.is_set():
                self.executor.spin_once(timeout_sec=0.05)
        except Exception as e:
            logger.error(f"Robot {self.namespace} ROS thread error: {e}")
        finally:
            logger.info(f"Robot {self.namespace} ROS thread stopped")

    def stop_ros(self):
        if self.ros_thread and self.ros_thread.is_alive():
            self.stop_event.set()
            self.ros_thread.join(timeout=2.0)
            if self.ros_thread.is_alive():
                logger.warning(
                    f"Robot {self.namespace} ROS thread did not stop gracefully"
                )

    def cleanup(self):
        self.stop_ros()

        try:
            self.executor.remove_node(self.node)
            self.executor.remove_node(self.node_planner_ompl)
            self.executor.remove_node(self.node_trajectory_generator)
            self.executor.remove_node(self.node_controller_mpc)

            self.node.destroy_node()
            self.node_planner_ompl.destroy_node()
            self.node_trajectory_generator.destroy_node()
            self.node_controller_mpc.destroy_node()
        except Exception as e:
            logger.error(f"Error cleaning up robot {self.name}: {e}")

    def initialize(self) -> None:
        for camera_name, camera_cfg in self.cfg_robot.cfg_dict_camera.items():
            camera_instance = Camera(
                path_prim_parent=self.cfg_robot.path_prim_robot, cfg_camera=camera_cfg
            )
            self.camera[camera_name] = camera_instance
        for cam_name, cam_cfg in self.cfg_robot.cfg_dict_camera_third.items():
            camera_instance = Camera(
                path_prim_parent=self.cfg_robot.path_prim_robot, cfg_camera=cam_cfg
            )
            self.camera_third[cam_name] = camera_instance
        for camera in self.camera.values():
            camera.initialize()

        # 第三视角相机初始化 - 使用ViewportManager
        # if self.cfg_camera_third_person and self.cfg_camera_third_person.enabled:
        #     self._initialize_third_person_camera()

    def on_physics_step(self, step_size) -> None:
        """
        Args:
            step_size:  dt 时间间隔
        """
        # 更新相机的视野
        self._update_camera_view()
        # publish robot position
        self.publish_robot_state()
        return

    def post_reset(self) -> None:
        """Set up things that happen after the world resets."""
        for sensor in self.cameras.values():
            sensor.post_reset()
        return

    def put_down(self, robot_hand_prim_path: str, object_prim_path: str) -> dict:
        """
        通过删除关节并恢复物理属性来“放下”一个被抓取的物体。
        """

        hand_prim = RigidPrim(prim_paths_expr=robot_hand_prim_path)
        object_prim = RigidPrim(prim_paths_expr=object_prim_path)

        # 定位并删除用于抓取的固定关节

        joint_path = f"/World/grasp_joint_{object_prim.name}"
        stage = self.scene_manager._stage
        joint_prim = stage.GetPrimAtPath(joint_path)

        if joint_prim.IsValid():
            joint = UsdPhysics.Joint(joint_prim)
            joint.GetJointEnabledAttr().Set(False)  # <-- 关键的状态切换！
            # 碰撞属性
            self.scene_manager.set_collision_enabled(
                object_prim_path, collision_enabled=True
            )
            # 模拟惯性
            self._publish_feedback(
                params=[
                    Parameter(key="status", value="normal"),
                    Parameter(key="reason", value=""),
                ],
                progress=50,
            )
            hand_lin_vel = hand_prim.get_linear_velocities()
            hand_ang_vel = hand_prim.get_angular_velocities()
            object_prim.set_linear_velocities(hand_lin_vel)
            object_prim.set_angular_velocities(hand_ang_vel)
        else:
            self._publish_feedback(
                params=[
                    Parameter(key="status", value="failed"),
                    Parameter(key="reason", value="Joint not found"),
                ],
                progress=0,
            )
            logger.info(f"WARNING: Joint '{joint_path}' not found, cannot disable.")

        logger.info(f"INFO: Object '{object_prim.name}' dropped and physics restored.")
        self._publish_feedback(
            params=[
                Parameter(key="status", value="normal"),
                Parameter(key="reason", value=""),
            ],
            progress=100,
        )
        return {"status": "success", "message": "Object dropped successfully."}

    def pickup_object_if_close_unified(
        self,
        robot_hand_prim_path: str,
        object_prim_path: str,
        distance_threshold: float = 2.0,
    ) -> dict:
        """
        检查机器人手部与物体的距离，如果小于阈值，则执行抓取。
        """
        hand_prim = RigidPrim(prim_paths_expr=robot_hand_prim_path)
        object_prim = RigidPrim(prim_paths_expr=object_prim_path)

        hand_pos, _ = hand_prim.get_world_poses()
        object_pos, _ = object_prim.get_world_poses()

        distance = np.linalg.norm(hand_pos - object_pos)

        if distance <= distance_threshold:

            self._publish_feedback(
                params=[
                    Parameter(key="status", value="normal"),
                    Parameter(key="reason", value=""),
                ],
                progress=30,
            )
            # 停止物体当前的任何运动，清除惯性
            object_prim.set_linear_velocities(np.zeros(3))
            object_prim.set_angular_velocities(np.zeros(3))

            # 将物体传送到精确的抓取位置
            object_prim.set_world_poses(positions=hand_pos)

            # 禁用碰撞属性
            self.scene_manager.set_collision_enabled(
                prim_path=object_prim_path, collision_enabled=False
            )

            # 创建物理连接 (关节)
            joint_path = f"/World/grasp_joint_{object_prim.name}"
            stage = self.scene_manager._stage  # 假设可以从SceneManager获取stage
            joint_prim = stage.GetPrimAtPath(joint_path)

            self._publish_feedback(
                params=[
                    Parameter(key="status", value="normal"),
                    Parameter(key="reason", value=""),
                ],
                progress=60,
            )

            # 如果关节不存在，就创建一个。这通常只在第一次抓取时发生。
            if not joint_prim.IsValid():
                logger.info(
                    f"INFO: Joint '{joint_path}' not found, creating it for the first time."
                )
                self.scene_manager.create_joint(
                    joint_path=joint_path,
                    joint_type="fixed",
                    body0=robot_hand_prim_path,
                    body1=object_prim_path,
                    local_pos0=[0, 0, 1],
                    local_pos1=[0, 0, 0],
                    axis=[0, 0, 1],
                )
                joint_prim = stage.GetPrimAtPath(joint_path)
            joint = UsdPhysics.Joint(joint_prim)
            joint.GetJointEnabledAttr().Set(True)  # <-- 关键的状态切换！

            self._publish_feedback(
                params=[
                    Parameter(key="status", value="normal"),
                    Parameter(key="reason", value=""),
                ],
                progress=100,
            )

            return {"status": "success"}
        else:
            self._publish_feedback(
                params=[
                    Parameter(key="status", value="failed"),
                    Parameter(key="reason", value="Object is too far to pick up"),
                ],
                progress=0,
            )
            return {
                "status": "skipped",
                "message": f"Object is too far to pick up ({distance:.2f}m > {distance_threshold}m).",
            }

    def take_photo(self, file_path: str = None):
        if self.camera is not None:
            rgb = self.camera.get_rgb()
            if rgb != None and file_path is not None:
                self.camera.save_rgb_to_file(rgb_tensor_gpu=rgb, file_path=file_path)
            return rgb
        else:
            return None

    def return_home(self):
        self.navigate_to(self.body.cfg_robot.base)

    def detect(self, target_prim: str = None):
        pos, quat = self.body.get_world_pose()
        result = self.scene_manager.overlap_hits_target_ancestor(target_prim)
        logger.info(
            f"[Skill] {self.body.cfg_robot.name} is detecting for {target_prim}. The result is {result}"
        )
        return result

    def broadcast(self, params: Dict[str, Any]):
        content = params.get("content")
        logger.info(
            f"[Skill] {self.body.cfg_robot.name} executing broadcasting. The content is {content}"
        )
        return True

    def explore(
        self,
        boundary: List[tuple] = None,
        holes: List[tuple] = None,
        target_prim: str = "/TARGET_PRIM_NOT_SPECIFIED",
    ):
        waypoints = self.plan_exploration_waypoints(
            boundary,
            holes,
            lane_width=self.body.cfg_robot.detection_radius,
            robot_radius=self.body.cfg_robot.robot_radius,
        )
        self.is_detecting = True  # TODO:detect的反馈机制
        self.target_prim = target_prim
        self.move_along_path(waypoints, flag_reset=True)
        return True

    def plan_exploration_waypoints(
        self,
        polygon_coords,
        holes=None,
        lane_width=1.0,
        robot_radius=0.2,
        sweep_deg=0.0,
        min_seg=0.5,
        z_out=0.0,  # 输出航点的 z 值（默认 0.0）
    ):
        """
        polygon_coords: [(x,y,z=0), ...] 或 [(x,y), ...]
        holes: [ [(x,y,z=0),...], ... ] 或 2D

        lane_width: 覆盖间距(传感器/工具有效宽度)
        robot_radius: 机体/安全半径，用于收缩边界
        sweep_deg: 扫掠角度(度)，0 表示沿 y 方向来回
        min_seg: 航段最小长度过滤（避免碎片段）
        """

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
            raise ValueError(
                "没有生成任何覆盖航点，请检查 lane_width/robot_radius/区域大小。"
            )

        # ---- 5) 旋回原坐标系并转 3D ----
        back = rotate(
            LineString(path_pts_2d), sweep_deg, origin=poly.centroid, use_radians=False
        )
        waypoints_2d = list(back.coords)

        # 去抖
        simplified_2d = [waypoints_2d[0]]
        for p in waypoints_2d[1:]:
            if (
                np.hypot(p[0] - simplified_2d[-1][0], p[1] - simplified_2d[-1][1])
                > 1e-3
            ):
                simplified_2d.append(p)

        # 输出 3D（z 固定为 z_out，默认 0）
        waypoints = [(x, y, float(z_out)) for (x, y) in simplified_2d]
        return waypoints

    def controller_simplified(
        self, linear_velocity: torch.Tensor, angular_velocity: torch.Tensor
    ) -> None:
        if self.body.robot_articulation.is_physics_handle_valid():
            self.body.robot_articulation.set_linear_velocities(linear_velocity)
            self.body.robot_articulation.set_angular_velocities(angular_velocity)
        return None

    def track_callback(self, msg):
        pos = (
            msg.pos.position.x,
            msg.pos.position.y,
            msg.pos.position.z,
        )
        self.track_waypoints.append(pos)

    def start_tracking(self, target_prim: str = None):
        self.is_tracking = True
        self.track_waypoints = self.node.create_subscription(
            VelTwistPose,
            "/target/motion",
            self.track_callback,
            50,
        )

    def track(self):
        if self.track_waypoint_index < len(
            self.track_waypoints
        ):  # 当index==len的时候, 就已经到达目标了
            flag_reach = self.move_to(self.track_waypoints[self.track_waypoint_index])
            if flag_reach == True:
                self.track_waypoint_index += 1  # TODO：Step里应用这个技能

    def stop_tracking(self):
        self.is_tracking = False
        self.track_waypoints = []
        self.track_waypoint_index = 0
        self.node.destroy_subscription(self.track_waypoints)
        self.track_waypoints_sub = None

    def _initialize_third_person_camera(self):
        """初始化第三人称相机并注册到ViewportManager"""
        logger.info(f"Creating third-person camera for robot {self.cfg_robot.id}...")

        # 1. 为相机创建唯一的prim路径和视口名称
        self.camera_prim_path = f"/World/Robot_{self.cfg_robot.id}_Camera"
        self.viewport_name = f"Viewport_Robot_{self.cfg_robot.id}"

        # 如果prim已存在，先删除（可选，用于热重载）
        if prims_utils.is_prim_path_valid(self.camera_prim_path):
            prims_utils.delete_prim(self.camera_prim_path)

        # 2. 创建相机Prim

        camera = Camera(prim_path=self.camera_prim_path)
        camera.set_focal_length(2)

        # 3. 创建视口
        viewport_obj = create_viewport_for_camera(
            viewport_name=self.viewport_name,
            camera_prim_path=self.camera_prim_path,
            width=self.cfg_camera_third_person.viewport_size[0],
            height=self.cfg_camera_third_person.viewport_size[1],
            position_x=self.cfg_camera_third_person.viewport_position[0],
            position_y=self.cfg_camera_third_person.viewport_position[1],
        )

        # 4. 注册viewport到ViewportManager
        if viewport_obj and self.viewport_manager:
            success = self.viewport_manager.register_viewport(
                self.viewport_name, viewport_obj
            )
            if success:
                self.viewport_manager.map_camera(
                    self.viewport_name, self.camera_prim_path
                )
                logger.info(
                    f"Robot {self.cfg_robot.id} viewport registered to ViewportManager"
                )
            else:
                raise RuntimeError(
                    f"Failed to register viewport for robot {self.cfg_robot.id}"
                )

        # 5. 将相对位置转换为numpy数组以便后续计算
        self.relative_camera_pos = np.array(
            self.cfg_camera_third_person.relative_position
        )
        self.transform_camera_pos = np.array(
            self.cfg_camera_third_person.transform_position
        )

    def _update_camera_view(self):
        """更新第三人称相机的位置和朝向"""
        if self.camera_third:
            # 1. 获取机器人的当前位置
            robot_position, _ = self.get_world_poses()  # torch.tensor

            # 2. 计算相机的位置 (eye) 和目标位置 (target)
            camera_eye_position = (
                robot_position + self.relative_camera_pos + self.transform_camera_pos
            )
            camera_target_position = robot_position

            # 3. 设置相机视图
            set_camera_view(
                eye=camera_eye_position,
                target=camera_target_position,
                camera_prim_path=self.camera_prim_path,
            )

    def get_viewport_info(self):
        """
        获取robot的viewport信息，供ViewportManager批量操作使用

        Returns:
            dict: 包含viewport_name, camera_path等信息的字典
        """
        return {
            "viewport_name": self.viewport_name,
            "camera_path": self.camera_prim_path,
            "robot_id": self.cfg_robot.id if self.cfg_robot else None,
            "enabled": (
                self.cfg_camera_third_person.enabled
                if self.cfg_camera_third_person
                else False
            ),
        }

    def _params_from_pose(self, pos: np.ndarray, quat: np.ndarray) -> list[Parameter]:
        # TODO:对齐这个param的类型
        p = np.asarray(pos, dtype=float)
        q = np.asarray(quat, dtype=float)
        if p.ndim >= 2:
            p = p[0]
        if q.ndim >= 2:
            q = q[0]

        # 现在期望 p.shape == (3,), q.shape == (4,)
        if p.size < 3 or q.size < 4:
            raise ValueError(f"Invalid pose shapes: pos={p.shape}, quat={q.shape}")

        px, py, pz = (float(v) for v in p)
        # 这里假定四元数顺序为 [x, y, z, w]
        qx, qy, qz, qw = (float(v) for v in q)

        base_return = [
            Parameter(key="pos_x", value=str(px)),
            Parameter(key="pos_y", value=str(py)),
            Parameter(key="pos_z", value=str(pz)),
            Parameter(key="quat_x", value=str(qx)),
            Parameter(key="quat_y", value=str(qy)),
            Parameter(key="quat_z", value=str(qz)),
            Parameter(key="quat_w", value=str(qw)),
        ]

        print(base_return)

        normal_return = [Parameter(key="status", value="normal"), *base_return]
        abnormal_return = [Parameter(key="status", value="abnormal"), *base_return]

        if self.previous_pos:
            if (
                np.sqrt(
                    (self.previous_pos[0] - px) ** 2
                    + (self.previous_pos[1] - py) ** 2
                    + (self.previous_pos[2] - pz) ** 2
                )
                < self.movement_threshold
            ):
                self.previous_pos = [px, py, pz]
                return abnormal_return
            else:
                self.previous_pos = [px, py, pz]
                return normal_return

        else:
            self.previous_pos = [px, py, pz]
            return normal_return

    def _publish_feedback(self, params, progress: int = 1, object_id: str = ""):

        pos, quat = self.body.get_world_pose()
        params = self._params_from_pose(pos, quat)

        skill = SkillInfo(
            skill=self.current_task_name,
            params=params,
            object_id=object_id,
            task_id=self.current_task_id,
            progress=int(
                progress
            ),  # progress 是当前技能的执行进度，progress是从0到100的一个 int32
        )

        msg = RobotFeedback(
            robot_id=f"{self.cfg_robot.type}_{self.cfg_robot.id}",
            skill_feedback=skill,
        )

        # self.node.publish_feedback(self.cfg_robot.type, self.cfg_robot.id, msg)

    def _publish_status_pose(self):

        import numpy as np

        pos, orn = self.body.get_world_pose()

        # 这些 API 返回 (N, 3) 的数组/张量；这里只取第 0 个
        lin_v = self.body.robot_articulation.get_linear_velocities(
            indices=[0], clone=True
        )
        ang_v = self.body.robot_articulation.get_angular_velocities(
            indices=[0], clone=True
        )

        # 统一成 numpy，做个健壮性兜底
        lin_v0 = np.asarray(lin_v)[0] if np.size(lin_v) else np.zeros(3, dtype=float)
        ang_v0 = np.asarray(ang_v)[0] if np.size(ang_v) else np.zeros(3, dtype=float)

        msg = VelTwistPose()

        # vel 字段：保持与你原逻辑一致，全部置 0
        msg.vel.x, msg.vel.y, msg.vel.z = [0.0, 0.0, 0.0]

        # twist：使用仿真里的实时速度
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = (
            float(v) for v in lin_v0
        )
        msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = (
            float(v) for v in ang_v0
        )
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = (
            float(v) for v in pos
        )

        # 兼容 ndarray（常见返回）与带属性的四元数对象两种情况
        if hasattr(orn, "x"):
            quat_xyzw = orn.x, orn.y, orn.z, orn.w
        else:
            # 约定顺序为 [x, y, z, w]；若你的资源是 wxyz，请按需调整
            quat_xyzw = float(orn[0]), float(orn[1]), float(orn[2]), float(orn[3])

        (
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ) = (float(v) for v in quat_xyzw)

        # self.node.publish_motion(
        #     robot_class=self.cfg_robot.type,
        #     robot_id=self.cfg_robot.id,
        #     msg=msg
        # )


if __name__ == "__main__":
    pass
