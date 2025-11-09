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
from scene.scene_manager import SceneManager

# Custom ROS message imports (for type hints only)
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
            scene_manager: SceneManager = None,
    ):

        self.cfg_dict_camera = self.cfg_robot.cfg_dict_camera
        self.cfg_dict_camera_third = self.cfg_robot.cfg_dict_camera_third
        self.camera_dict = {}
        self.camera_third_dict = {}

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

        self.view_angle: float = 2 * np.pi / 3  # 感知视野 弧度
        self.view_radius: float = 2  # 感知半径 米

        # 第三视角相机 一个机器人只有一个
        # self.cfg_camera_third_person = cfg_camera_third_person
        self.viewport_name = None  # 存储viewport名称
        self.relative_camera_pos = np.array([0, 0, 0])  # 默认为0向量
        self.transform_camera_pos = np.array([0, 0, 0])
        
        # ROS manager (optional, injected from outside)
        self.ros_manager = None
        
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
        """Publish robot state (if ROS is enabled)"""
        pos, quat = self.body.get_world_pose()
        vel_linear, vel_angular = self.body.get_world_vel()

        self.pos = pos
        self.quat = quat

        # Publish to ROS if available
        if self.has_ros():
            pos = pos.detach().cpu().numpy()
            quat = quat.detach().cpu().numpy()
            vel_linear = vel_linear.detach().cpu().numpy()
            vel_angular = vel_angular.detach().cpu().numpy()
            
            self.ros_manager.publish_odometry(pos, quat, vel_linear, vel_angular)

    ########################## Subscriber Velocity  ############################
    def set_velocity_command(self, linear_vel, angular_vel):
        """设置机器人速度命令 - 业务逻辑接口"""
        self.vel_linear = torch.tensor(linear_vel)
        self.vel_angular = torch.tensor(angular_vel)
        logger.debug(f"set linear vel: {linear_vel}, angular vel: {angular_vel}")

    ########################## ROS Manager Interface ############################
    
    def set_ros_manager(self, ros_manager):
        """Set ROS manager (dependency injection)"""
        self.ros_manager = ros_manager
    
    def get_ros_manager(self):
        """Get ROS manager"""
        return self.ros_manager
    
    def has_ros(self):
        """Check if ROS is enabled"""
        return self.ros_manager is not None
    
    def cleanup(self):
        """Cleanup robot resources"""
        if self.has_ros():
            self.ros_manager.stop()

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

    def apply_control(self, control):
        """Apply CARLA-style control: Control object -> velocity command"""
        from simulation.control import RobotControl
        if isinstance(control, RobotControl):
            self._current_control = control  # Cache for get_control()
            self.set_velocity_command(control.linear_velocity, control.angular_velocity)
    
    def get_control(self):
        """Get current control (CARLA-style)"""
        from simulation.control import RobotControl
        if not hasattr(self, '_current_control'):
            # Return default control if none applied yet
            control = RobotControl()
            control.linear_velocity = self.vel_linear.tolist()
            control.angular_velocity = self.vel_angular.tolist()
            return control
        return self._current_control

    def on_physics_step(self, step_size) -> None:
        """
        Args:
            step_size:  dt 时间间隔
        """
        # publish robot position
        self.publish_robot_state()
        # 更新相机的视野
        self._update_camera_view()
        
        # Calculate robot velocity (if ROS is enabled)
        if self.has_ros():
            self.ros_manager.get_node_controller_mpc().control_loop()
        
        # Update robot velocity
        self.controller_simplified()
        return

    def post_reset(self) -> None:
        for sensor in self.cameras.values():
            sensor.post_reset()

    def controller_simplified(self) -> None:
        if self.body.robot_articulation.is_physics_handle_valid():
            self.body.robot_articulation.set_linear_velocities(self.vel_linear)
            self.body.robot_articulation.set_angular_velocities(self.vel_angular)
            logger.debug(f"Robot Articulation vel, {self.vel_linear}")

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
