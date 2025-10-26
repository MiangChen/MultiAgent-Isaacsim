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
from typing import Any, Dict
import time

# Third-party library imports
import numpy as np
import py_trees
import torch

# Local project imports
from log.log_manager import LogManager
from physics_engine.isaacsim_utils import (
    Scene,
    prims_utils,
    create_viewport_for_camera,
    set_camera_view,
    Camera,
)
from robot.sensor.camera import Camera
from robot.body import BodyRobot
from robot.robot_trajectory import Trajectory
from robot.skill.base.navigation import NodePlannerOmpl
from robot.skill.base.navigation import NodeTrajectoryGenerator
from robot.skill.base.navigation import NodeMpcController
from robot.skill.behavior_tree_manager import BehaviorTreeManager
from ros.node_robot import NodeRobot
from scene.scene_manager import SceneManager

# ROS2 imports
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from nav_msgs.msg import Odometry
from nav2_msgs.action import ComputePathToPose

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

        # robot physics state
        self.vel_linear = torch.tensor([0.0, 0.0, 0.0])
        self.vel_angular = torch.tensor([0.0, 0.0, 0.0])

        # 机器人的技能
        # self.skills: dict = {}  # 用于记录其技能: 'skill name': function
        # self.skill_manager = SkillManager(self)
        self.behaviour_tree = None       # 存储当前执行的行为树
        self.behavior_tree_manager = BehaviorTreeManager(self)
        self.active_goal_handle = None   # 存储当前活动的Action Goal句柄
        self.action_timer = None         # 用于周期性 tick 行为树的定时器

        # 用于回调函数中
        self.flag_active = False
        self.flag_world_reset: bool = False  # 用来记录下世界是不是被初始化了
        # self.flag_action_navigation: bool = False  # 用来记录是不是启动导航了
        # 用于PDDL, 记录当前用的 skill, 之所以用skill, 是为了和action区分, action一般是底层的关节动作
        # self.state_skill: str = ""
        # self.state_skill_complete: bool = True  # 默认状态, 没有skill要做, 所以是True

        # 布置机器人的相机传感器, 可以有多个相机
        self.cameras: dict = {}
        self.view_angle: float = 2 * np.pi / 3  # 感知视野 弧度
        self.view_radius: float = 2  # 感知半径 米

        # 第三视角相机 一个机器人只有一个
        # self.cfg_camera_third_person = cfg_camera_third_person
        self.viewport_name = None  # 存储viewport名称
        self.relative_camera_pos = np.array([0, 0, 0])  # 默认为0向量
        self.transform_camera_pos = np.array([0, 0, 0])

        # 初始化基础ROS组件
        self._init_ros()

        # self.node.start_spinning()
        # self.node_planner_ompl.start_spinning()
        # self.node_trajectory_generator.start_spinning()
        # self.node_controller_mpc.start_spinning()

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

        self.track_waypoint_list = []
        self.track_waypoint_index = 0
        self.is_tracking = False
        self.track_waypoint_sub = None
        self.is_planning = False

        #测试锁
        self._test_lock = threading.Lock()

    def callback_task_execution(self, goal_handle):
        """
        新的非阻塞式 Action Server 回调。
        当接收到新的 Action Goal 时被调用。
        """
        self.node.get_logger().info('接收到新的任务目标...')

        if self.active_goal_handle and self.active_goal_handle.is_active:
            self.node.get_logger().error('机器人正忙，拒绝新任务！')
            goal_handle.abort()
            return SkillExecution.Result(success=False, message="机器人正忙")

        request = goal_handle.request.skill_request
        task_name = request.skill_list[0].skill  # todo 返回的是一个技能列表, 目前我们先处理第一个
        params = {p.key: p.value for p in request.skill_list[0].params}

        try:
            self.behaviour_tree = self.behavior_tree_manager.create_tree_for_task(task_name, params)
            if self.behaviour_tree is None:
                raise ValueError(f"无法为任务 '{task_name}' 创建行为树")
        except Exception as e:
            self.node.get_logger().error(f"构建行为树失败: {e}")
            goal_handle.abort()
            return SkillExecution.Result(success=False, message=f"构建行为树失败: {e}")

        self.active_goal_handle = goal_handle
        goal_handle.accept()


        tick_frequency = 10.0
        self.node.get_logger().info(f"创建定时器，频率: {tick_frequency} Hz")
        self.action_timer = self.node.create_timer(1.0 / tick_frequency, self.tick_the_tree)
        self.node.get_logger().info(f"定时器创建成功: {self.action_timer is not None}")
        self.node.get_logger().info(f"任务 '{task_name}' 已接受并开始执行。")

        return SkillExecution.Result()

    def tick_the_tree(self):
        """
        定时器回调函数，这是您的新“主执行循环”。
        它会周期性地驱动行为树并发送反馈。
        """
        self.node.get_logger().info("tick_the_tree() 被调用")
        
        if not self.active_goal_handle or not self.active_goal_handle.is_active:
            self.node.get_logger().warn("当前任务句柄无效或非活动，停止执行。")
            self.cleanup_action()
            return

        try:
            self.node.get_logger().info("开始执行 behaviour_tree.tick()")
            self.behaviour_tree.tick()

            feedback_msg = SkillExecution.Feedback()
            feedback_msg.status = self.behaviour_tree.root.status.value # RUNNING, SUCCESS, FAILURE
            active_node = next((node for node in self.behaviour_tree.root.iterate() if node.status == py_trees.common.Status.RUNNING), None)
            feedback_msg.reason = active_node.name if active_node else "任务完成中"
            progress = self.behaviour_tree.blackboard_client.get("progress")
            feedback_msg.progress = int(progress) if progress is not None else 0
            self.active_goal_handle.publish_feedback(feedback_msg)

            tree_status = self.behaviour_tree.root.status
            if tree_status == py_trees.common.Status.SUCCESS:
                self.node.get_logger().info('任务成功完成！')
                result = SkillExecution.Result(success=True, message="任务成功")
                self.active_goal_handle.succeed()
                self.cleanup_action()
            elif tree_status == py_trees.common.Status.FAILURE:
                self.node.get_logger().warn('任务失败！')
                result = SkillExecution.Result(success=False, message="任务执行失败")
                self.active_goal_handle.abort()
                self.cleanup_action()
        except Exception as e:
            self.node.get_logger().error(f"行为树 tick 期间发生异常: {e}", exc_info=True)
            result = SkillExecution.Result(success=False, message=f"执行异常: {e}")
            self.active_goal_handle.abort()
            self.cleanup_action()

    def cleanup_action(self):
        """任务结束后（成功、失败或取消），清理所有相关资源。"""
        if self.action_timer:
            self.action_timer.cancel()
            self.action_timer = None
        self.behaviour_tree = None
        self.node.get_logger().info("任务已清理。")
        self.active_goal_handle = None # 在最后重置，允许新任务进入

    def run_skill_for_test(
            self,
            task_name: str,
            params: Dict[str, str] = None,
            *,
            tick_hz: float = 10.0,
            timeout_sec: float = None,
            progress_cb = None,
            verbose: bool = True,
    ) -> Dict[str, object]:
        """
        从外部直接调用以测试一个技能(任务)：
        - 创建相应的行为树并以固定频率tick直至完成/失败/超时。
        - 不依赖Action Server，不发布ROS Action反馈，仅通过返回值/回调暴露进度与结果。

        参数:
            task_name: 技能/任务名
            params: 任务参数字典，如 {"target": "kitchen"}
            tick_hz: tick频率(Hz)
            timeout_sec: 超时时间(秒)。None表示不限时。
            progress_cb: 进度回调，签名为 `callback(dict)`；见下方结构说明。
            verbose: 是否打印基本日志到 self.node.get_logger()

        调用示例：
            res = swarm_manager.robot_active['jetbot'][0].run_skill_for_test(
                task_name="simple_navigation",
                params={"goal_pos":[10.0, 5.0, 1.0], "goal_quat_wxyz":[0.0, 0.0, 0.0, 1.0]},
                tick_hz=10.0,
                timeout_sec=60.0,
                progress_cb=lambda fb: print(f"[progress] {fb}"),
            )
            print(res)

        返回:
            dict: {
                "success": bool,
                "message": str,
                "status": "SUCCESS"|"FAILURE"|"TIMEOUT"|"EXCEPTION",
                "elapsed_sec": float
            }
        """
        with self._test_lock:
            # 如果你的Action正在执行，保护性地拒绝测试，避免同一份self.behaviour_tree被并发修改
            if getattr(self, "active_goal_handle", None) and self.active_goal_handle.is_active:
                msg = "机器人正忙（存在活动的Action任务），测试被拒绝"
                if verbose:
                    self.node.get_logger().error(msg)
                return {"success": False, "message": msg, "status": "BUSY", "elapsed_sec": 0.0}

            start_t = time.time()
            dt = 1.0 / max(tick_hz, 1e-3)
            result = {"success": False, "message": "", "status": "UNKNOWN", "elapsed_sec": 0.0}

            # 构建行为树
            try:
                if verbose:
                    self.node.get_logger().info(f"[TEST] 创建行为树: task='{task_name}', params={params or {} }")
                self.behaviour_tree = self.behavior_tree_manager.create_tree_for_task(task_name, params or {})
                if self.behaviour_tree is None:
                    raise ValueError(f"无法为任务 '{task_name}' 创建行为树")
            except Exception as e:
                msg = f"[TEST] 构建行为树失败: {e}"
                if verbose:
                    self.node.get_logger().error(msg)
                return {"success": False, "message": msg, "status": "EXCEPTION", "elapsed_sec": 0.0}

            try:
                # 主测试循环：直至SUCCESS/FAILURE/超时
                while True:
                    # 执行一个tick
                    self.behaviour_tree.tick()

                    root = self.behaviour_tree.root
                    tree_status = root.status  # py_trees.common.Status
                    # 找到第一个RUNNING的活动节点，用于“reason”
                    active_node = next(
                        (node for node in root.iterate() if node.status == py_trees.common.Status.RUNNING),
                        None
                    )

                    # 从黑板(如果有)拿一个通用的整数型进度
                    progress = 0
                    bb_client = getattr(self.behaviour_tree, "blackboard_client", None)
                    if bb_client is not None:
                        p = bb_client.get("progress")
                        if p is not None:
                            try:
                                progress = int(p)
                            except Exception:
                                progress = 0

                    # 进度回调（可选）
                    if progress_cb:
                        progress_cb({
                            "status": tree_status.value,  # "RUNNING"|"SUCCESS"|"FAILURE"
                            "reason": active_node.name if active_node else (
                                "完成中" if tree_status == py_trees.common.Status.RUNNING else ""),
                            "progress": progress,
                            "timestamp": time.time()
                        })

                    # 结束条件判断
                    if tree_status == py_trees.common.Status.SUCCESS:
                        result.update({
                            "success": True,
                            "message": "任务成功",
                            "status": "SUCCESS",
                        })
                        if verbose:
                            self.node.get_logger().info("[TEST] 任务成功完成")
                        break
                    elif tree_status == py_trees.common.Status.FAILURE:
                        result.update({
                            "success": False,
                            "message": "任务执行失败",
                            "status": "FAILURE",
                        })
                        if verbose:
                            self.node.get_logger().warn("[TEST] 任务失败")
                        break

                    # 超时判断
                    if timeout_sec is not None and (time.time() - start_t) > timeout_sec:
                        result.update({
                            "success": False,
                            "message": f"测试执行超时({timeout_sec}s)",
                            "status": "TIMEOUT",
                        })
                        if verbose:
                            self.node.get_logger().warn(f"[TEST] 任务超时({timeout_sec}s)")
                        break

                    # 简单sleep控制tick频率（测试场景，不用ROS定时器）
                    time.sleep(dt)

            except Exception as e:
                msg = f"[TEST] 执行异常: {e}"
                if verbose:
                    self.node.get_logger().error(msg, exc_info=True)
                result.update({
                    "success": False,
                    "message": msg,
                    "status": "EXCEPTION",
                })
            finally:
                # 与Action通道复用相同的清理逻辑
                try:
                    self.cleanup_action()
                except Exception as _:
                    pass
                result["elapsed_sec"] = round(time.time() - start_t, 3)

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

        odom_msg.pose.pose.orientation.x = float(quat[1])  # xyzw <- wxyz
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
        self.vel_linear = linear_vel
        self.vel_angular = angular_vel

    ########################## Infrastructure Initialization ############################
    
    def _init_ros(self):
        # ROS节点基础设施
        self.node = NodeRobot(namespace=self.namespace)
        self.node.set_robot_instance(self)
        # self.node.callback_execute_skill = self.callback_task_execution  # 不可以用这个方法, 回调函数无法在后期修改

        # 导航基础设施节点
        self.node_planner_ompl = NodePlannerOmpl(namespace=self.namespace)
        self.node_trajectory_generator = NodeTrajectoryGenerator(namespace=self.namespace)
        self.node_controller_mpc = NodeMpcController(namespace=self.namespace)
        
        # Action客户端
        self.action_client_path_planner = ActionClient(
            self.node, ComputePathToPose, "action_compute_path_to_pose"
        )

        # 执行器和线程管理
        self.executor = MultiThreadedExecutor()
        self.ros_thread = None
        self.stop_event = threading.Event()
        
        # 设置执行器并启动ROS
        self.executor.add_node(self.node)
        self.executor.add_node(self.node_planner_ompl)
        self.executor.add_node(self.node_trajectory_generator)
        self.executor.add_node(self.node_controller_mpc)

        self.start_ros()

    ########################## Start ROS  ############################

    def start_ros(self):
        if self.ros_thread is None or not self.ros_thread.is_alive():
            self.ros_thread = threading.Thread(
                target=self._spin_ros, daemon=True, name=f"ROS_{self.namespace}"
            )
            self.ros_thread.start()
            logger.info(f"Robot {self.namespace} ROS thread started")
            # 等待一小段时间确保线程启动
            import time
            time.sleep(0.1)
            logger.info(f"Robot {self.namespace} ROS thread is_alive: {self.ros_thread.is_alive()}")

    def _spin_ros(self):
        logger.info(f"Robot {self.namespace} ROS thread started spinning")
        try:
            spin_count = 0
            while not self.stop_event.is_set():
                self.executor.spin_once(timeout_sec=0.05)
                spin_count += 1
                # 每1000次spin记录一次，避免日志过多
                if spin_count % 1000 == 0:
                    logger.debug(f"Robot {self.namespace} ROS thread spinning... count: {spin_count}")
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
            logger.error(f"Error cleaning up robot {self.namespace}: {e}")

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

    def form_feedback(self, status: str = "processing", reason: str = "none", progress: int = 100) -> Dict[str, Any]:
        return dict(
            status=status,
            reason=reason,
            progress=progress, )

    def on_physics_step(self, step_size) -> None:
        """
        Args:
            step_size:  dt 时间间隔
        """
        self.execute_frame_skill()
        # update robot velocity
        self.controller_simplified()
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

    ############ controller ######################
    def controller_simplified(
        self,
    ) -> None:
        """
        this function can only be used in on_physics_step
        """
        if self.body.robot_articulation.is_physics_handle_valid():
            self.body.robot_articulation.set_linear_velocities(self.vel_linear)
            self.body.robot_articulation.set_angular_velocities(self.vel_angular)
        return None

    def execute_frame_skill(
            self,
    ) -> None:
        """
        需要周期性执行的技能，如拍照，检测，喊话
        """
        if self.is_detecting:
            self.detect(self.target_prim)
        if (self.is_tracking
                and self.node_controller_mpc.has_reached_goal
                and self.track_waypoint_index < len(self.track_waypoint_list)
        ):
            self.navigate_to(self.track_waypoint_list[self.track_waypoint_index])
            self.track_waypoint_index += 1

    def track_callback(self, msg):
        pos = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        )
        self.track_counter += 1
        if self.track_counter % self.track_period == 0:
            self.track_waypoint_list.append(pos)

    def start_tracking(self, target_prim: str = None):
        self.is_tracking = True
        self.track_waypoint_list = self.node.create_subscription(
            VelTwistPose,
            "/target/odom",
            self.track_callback,
            50,
        )
        return self.form_feedback(status="normal")

    def stop_tracking(self):
        self.is_tracking = False
        self.track_waypoint_list = []
        self.track_waypoint_index = 0
        self.node.destroy_subscription(self.track_waypoint_sub)
        self.track_waypoint_sub = None
        return self.form_feedback(status="finished")

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
