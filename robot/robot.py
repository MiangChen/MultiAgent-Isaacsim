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
        self.camera_dict = {}
        self.camera_third_dict = {}

        self.scene = scene
        self.scene_manager = scene_manager
        self.viewport_manager = _get_viewport_manager_from_container()
        self.cfg_robot.path_prim_robot = (
            self.cfg_robot.path_prim_swarm
            + f"/{self.cfg_robot.type}"
            + f"/{self.cfg_robot.type}_{self.cfg_robot.id}"
        )
        self.cfg_robot.namespace = self.cfg_robot.type + f"_{self.cfg_robot.id}"
        self.namespace = self.cfg_robot.namespace

        self.body: BodyRobot = None

        # 机器人的控制器
        self.controllers: dict = {}  # 用于存储多个控制器, 'controller name': function
        self.control_mode: str = (
            ""  # 'joint_efforts', 'joint_velocities', 'joint_positions', 'joint_indices', 'joint_names'
        )
        self.action: np.ndarray = None

        # robot physics state
        self.vel_linear = torch.tensor([0.0, 0.0, 0.0])
        self.vel_angular = torch.tensor([0.0, 0.0, 0.0])
        self.pos = torch.tensor([0.0, 0.0, 0.0])
        self.quat = torch.tensor([0.0, 0.0, 0.0, 1.0])
        self.sim_time = 0.0

        # 字典化的技能管理 - 以函数名为key
        self.skill_states = (
            {}
        )  # {"navigate_to_skill": "EXECUTING", "take_off_skill": "COMPLETED", ...}
        self.skill_functions = (
            {}
        )  # {"navigate_to_skill": navigate_to_skill, "take_off_skill": take_off_skill, ...}
        self.skill_params = (
            {}
        )  # {"navigate_to_skill": {...params...}, "take_off_skill": {...params...}, ...}
        self.skill_errors = (
            {}
        )  # {"navigate_to_skill": "error_message", "take_off_skill": None, ...}
        self.skill_feedbacks = (
            {}
        )  # {"navigate_to_skill": {...feedback...}, "take_off_skill": {...feedback...}, ...}

        # 技能私有数据存储
        self.skill_data = (
            {}
        )  # {"navigate_to_skill": {...private_data...}, "take_off_skill": {...private_data...}, ...}

        self.view_angle: float = 2 * np.pi / 3  # 感知视野 弧度
        self.view_radius: float = 2  # 感知半径 米

        # 第三视角相机 一个机器人只有一个
        # self.cfg_camera_third_person = cfg_camera_third_person
        self.viewport_name = None  # 存储viewport名称
        self.relative_camera_pos = np.array([0, 0, 0])  # 默认为0向量
        self.transform_camera_pos = np.array([0, 0, 0])
        self._init_ros()
        self.is_detecting = False
        self.target_prim = None

        self.track_waypoint_list = []
        self.track_waypoint_index = 0
        self.is_tracking = False
        self.track_waypoint_sub = None

        self.track_counter = 0
        self.track_period = 300

    ########################## Publisher Odom  ############################
    def publish_robot_state(self):
        pos, quat = self.body.get_world_pose()
        vel_linear, vel_angular = self.body.get_world_vel()

        self.pos = pos
        self.quta = quat

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

        self.node.publisher_dict["odom"].publish(odom_msg)

    ########################## Subscriber Velocity  ############################
    def set_velocity_command(self, linear_vel, angular_vel):
        """设置机器人速度命令 - 业务逻辑接口"""
        self.vel_linear = torch.tensor(linear_vel)
        self.vel_angular = torch.tensor(angular_vel)
        logger.debug(f"set linear vel: {linear_vel}, angular vel: {angular_vel}")

    ########################## Infrastructure Initialization ############################

    def _init_ros(self):
        # ROS节点基础设施
        self.node = NodeRobot(namespace=self.namespace, topics=self.cfg_robot.topics)
        self.node.set_robot_instance(self)

        # 导航基础设施节点
        self.action_client_path_planner = ActionClient(
            self.node, ComputePathToPose, "action_compute_path_to_pose"
        )
        self.node_planner_ompl = NodePlannerOmpl(namespace=self.namespace)
        self.node_trajectory_generator = NodeTrajectoryGenerator(
            namespace=self.namespace
        )
        self.node_controller_mpc = NodeMpcController(namespace=self.namespace)

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
            import time

            time.sleep(0.1)

    def _spin_ros(self):
        logger.info(f"Robot {self.namespace} ROS thread started spinning")
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
            logger.error(f"Error cleaning up robot {self.namespace}: {e}")

    def initialize(self) -> None:
        for camera_name, camera_cfg in self.cfg_robot.cfg_dict_camera.items():
            camera_instance = Camera(
                path_prim_parent=self.cfg_robot.path_prim_robot, cfg_camera=camera_cfg
            )
            self.camera_dict[camera_name] = camera_instance
        for cam_name, cam_cfg in self.cfg_robot.cfg_dict_camera_third.items():
            camera_instance = Camera(
                path_prim_parent=self.cfg_robot.path_prim_robot, cfg_camera=cam_cfg
            )
            self.camera_third_dict[cam_name] = camera_instance
        for camera in self.camera_dict.values():
            camera.initialize()

        # 第三视角相机初始化 - 使用ViewportManager
        # if self.cfg_camera_third_person and self.cfg_camera_third_person.enabled:
        #     self._initialize_third_person_camera()

    def form_feedback(
        self, status: str = "processing", message: str = "none", progress: int = 100
    ) -> Dict[str, Any]:
        return dict(
            status=str(status),
            message=str(message),
            progress=progress,
        )

    def on_physics_step(self, step_size) -> None:
        """
        Args:
            step_size:  dt 时间间隔
        """
        # publish robot position
        self.publish_robot_state()
        # 更新相机的视野
        self._update_camera_view()
        # 执行技能步骤
        self.execute_skill_step()
        # calculate robot velocity
        self.node_controller_mpc.control_loop()
        # update robot velocity
        self.controller_simplified()
        return

    def execute_skill_step(self):
        """执行所有活跃技能的步骤"""
        if self.skill_functions:
            completed_skills = []

            for skill_name, skill_function in list(self.skill_functions.items()):

                # 执行技能
                params = self.skill_params.get(skill_name, {})
                feedback = skill_function(**params)
                self.skill_feedbacks[skill_name] = feedback

                # 检查技能是否完成
                if feedback and feedback.get("status") in ["completed", "failed"]:
                    completed_skills.append(skill_name)

                # except Exception as e:
                #     self.skill_errors[skill_name] = str(e)
                #     self.skill_states[skill_name] = "FAILED"
                #     completed_skills.append(skill_name)

            # 清理完成的技能
            for skill_name in completed_skills:
                self._cleanup_skill(skill_name)

            return

    def start_skill(self, skill_function: callable, **params):
        """启动技能 - 使用函数名作为技能标识"""
        skill_name = skill_function.__name__
        params["robot"] = self

        self.skill_params[skill_name] = params
        self.skill_states[skill_name] = None
        self.skill_errors[skill_name] = {}
        self.skill_feedbacks[skill_name] = {}
        self.skill_data[skill_name] = {}
        self.skill_functions[skill_name] = skill_function

        return skill_name

    def _cleanup_skill(self, skill_name: str):
        """内部清理方法"""
        self.skill_functions.pop(skill_name, None)
        self.skill_params.pop(skill_name, None)
        self.skill_states.pop(skill_name, None)
        self.skill_errors.pop(skill_name, None)
        # self.skill_feedbacks.pop(skill_name, None)  # 因为需要在ros robot中反馈机器人的执行结果, 所以暂时不可以清除
        self.skill_data.pop(skill_name, None)

    def set_skill_data(self, skill_name: str, key: str, value):
        """设置技能私有数据"""
        if skill_name not in self.skill_data:
            self.skill_data[skill_name] = {}
        self.skill_data[skill_name][key] = value

    def get_skill_data(self, skill_name: str, key: str, default=None):
        """获取技能私有数据"""
        return self.skill_data.get(skill_name, {}).get(key, default)

    def post_reset(self) -> None:
        for sensor in self.cameras.values():
            sensor.post_reset()

    def controller_simplified(self) -> None:
        if self.body.robot_articulation.is_physics_handle_valid():
            self.body.robot_articulation.set_linear_velocities(self.vel_linear)
            self.body.robot_articulation.set_angular_velocities(self.vel_angular)
            logger.debug(f"Robot Articulation vel, {self.vel_linear}")

    def execute_frame_skill(self) -> None:
        if self.is_detecting:
            detect_skill(self, self.target_prim)
        if (
            self.is_tracking
            and self.node_controller_mpc.has_reached_goal
            and self.track_waypoint_index < len(self.track_waypoint_list)
        ):
            self.skill_function = navigate_to_skill
            self.skill_params = {
                "robot": self,
                "goal_pos": self.track_waypoint_list[self.track_waypoint_index],
            }
            self.track_waypoint_index += 1

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
        if self.camera_third_dict:
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

    def update_sim_time(self, sim_time):
        """更新仿真时间"""
        self.sim_time = sim_time

    def get_available_skills(self):
        """获取当前机器人类型支持的所有技能"""
        from robot.skill.skill_registry import SkillRegistry

        return SkillRegistry.get_skill_names_for_robot(self.cfg_robot.type)
