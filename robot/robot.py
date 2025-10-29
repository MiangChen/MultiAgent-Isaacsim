# =============================================================================
# Robot Module - Core Robot Implementation
# =============================================================================
#
# This module provides the base Robot class and related functionality for
# robotic simulation and control within the Isaac Sim environment.
#
# =============================================================================

# Standard library imports
import json
import threading
from typing import Any, Dict

# Third-party library imports
import numpy as np
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
from robot.skill.base.detection import detect_skill
from robot.skill.base.navigation import navigate_to_skill
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
        self.active_goal_handle = None  # 存储当前活动的Action Goal句柄

        # 用于回调函数中
        self.flag_active = False
        self.flag_world_reset: bool = False  # 用来记录下世界是不是被初始化了

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
        self.track_counter = 0
        self.track_period = 300

        # 测试锁
        self._test_lock = threading.Lock()

        self.skill_generator = None

    def callback_skill_execution(self, goal_handle):
        if self.active_goal_handle:
            goal_handle.abort()
            return SkillExecution.Result(success=False, message="机器人正忙")

        try:
            request = goal_handle.request.skill_request
            task_name = request.skill_list[0].skill
            params = {p.key: p.value for p in request.skill_list[0].params}
            params["robot"] = self

            self.prepare_skill_execution(goal_handle, task_name, params)
            return SkillExecution.Result(success=True, message=f"技能 {task_name} 启动成功")
        except Exception as e:
            goal_handle.abort()
            return SkillExecution.Result(success=False, message=f"启动失败: {str(e)}")

    def prepare_skill_execution(self, goal_handle, task_name, params) -> None:
        """准备技能执行，在 physics step 中执行"""
        from robot.skill.base.navigation.navigate_to import navigate_to_skill
        from robot.skill.base.manipulation.pick_up import pick_up_skill
        from robot.skill.base.manipulation.put_down import put_down_skill

        SKILL_TABLE = {
            "navigation": navigate_to_skill,
            "pickup": pick_up_skill,
            "putdown": put_down_skill,
        }

        try:
            # 创建技能生成器，但不开始执行
            self.skill_generator = SKILL_TABLE[task_name](**params)
            self.active_goal_handle = goal_handle
            self.skill_feedback_msg = SkillExecution.Feedback()

            logger.info(
                f"技能 {task_name} 已准备就绪，将在下一个 physics step 开始执行"
            )

        except Exception as e:
            logger.error(f"准备技能失败: {e}")
            goal_handle.abort()

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
        logger.debug(f"get linear vel: {linear_vel}, angular vel: {angular_vel}")

    ########################## Infrastructure Initialization ############################

    def _init_ros(self):
        # ROS节点基础设施
        self.node = NodeRobot(namespace=self.namespace)
        self.node.set_robot_instance(self)

        # 导航基础设施节点
        self.action_client_path_planner = ActionClient(
            self.node, ComputePathToPose, "action_compute_path_to_pose"
        )  # 用于发布消息
        self.node_planner_ompl = NodePlannerOmpl(namespace=self.namespace)
        self.node_trajectory_generator = NodeTrajectoryGenerator(
            namespace=self.namespace
        )
        self.node_controller_mpc = NodeMpcController(namespace=self.namespace)

        # 技能状态发布器
        from std_msgs.msg import String
        self.skill_status_pub = self.node.create_publisher(String, "skill_status", 10)

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
            logger.info(
                f"Robot {self.namespace} ROS thread is_alive: {self.ros_thread.is_alive()}"
            )

    def _spin_ros(self):
        logger.info(f"Robot {self.namespace} ROS thread started spinning")
        try:
            spin_count = 0
            while not self.stop_event.is_set():
                self.executor.spin_once(timeout_sec=0.05)
                spin_count += 1
                # 每1000次spin记录一次，避免日志过多
                if spin_count % 20 == 0:
                    logger.debug(
                        f"Robot {self.namespace} ROS thread spinning... count: {spin_count}"
                    )
                spin_count += 1
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

    def form_feedback(
        self, status: str = "processing", reason: str = "none", progress: int = 100
    ) -> Dict[str, Any]:
        return dict(
            status=status,
            reason=reason,
            progress=progress,
        )

    def on_physics_step(self, step_size) -> None:
        """
        Args:
            step_size:  dt 时间间隔
        """
        # 执行技能步骤
        self.execute_frame_skill()
        self.execute_skill_step()
        # update robot velocity
        self.controller_simplified()
        # 更新相机的视野
        self._update_camera_view()
        # publish robot position
        self.publish_robot_state()
        # on control loop
        self.node_controller_mpc.control_loop()
        return

    def execute_skill_step(self):
        if not self.skill_generator:
            return

        try:
            feedback = next(self.skill_generator)
            self.publish_skill_status(feedback)
        except StopIteration as e:
            final_result = e.value if hasattr(e, 'value') and e.value else {}
            logger.info(f"技能执行完成: {final_result}")
            self.publish_skill_status(final_result)
            self.cleanup_skill()
        except Exception as e:
            logger.error(f"技能执行失败: {e}")
            self.publish_skill_status({"status": "failed", "reason": str(e)})
            self.cleanup_skill()


    def cleanup_skill(self):
        self.active_goal_handle = None
        self.skill_generator = None
        self.skill_feedback_msg = None

    def publish_skill_status(self, status_dict):
        if hasattr(self, 'skill_status_pub'):
            from std_msgs.msg import String
            msg = String()
            msg.data = json.dumps(status_dict)
            self.skill_status_pub.publish(msg)

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
            logger.debug(f"Robot Articulation vel, {self.vel_linear}")
        return None

    def execute_frame_skill(
            self,
    ) -> None:
        """
        需要周期性执行的技能，如拍照，检测，喊话
        """
        if self.is_detecting:
            detect_skill(self, self.target_prim)
        if (
                self.is_tracking
                and self.node_controller_mpc.has_reached_goal
                and self.track_waypoint_index < len(self.track_waypoint_list)
        ):
            self.skill_generator = navigate_to_skill(
                robot=self, goal_pos=self.track_waypoint_list[self.track_waypoint_index]
            )
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

        # # 如果prim已存在，先删除（可选，用于热重载）
        # if prims_utils.is_prim_path_valid(self.camera_prim_path):
        #     prims_utils.delete_prim(self.camera_prim_path)

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
