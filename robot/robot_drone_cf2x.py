# =============================================================================
# Robot Drone CF2X Module - Crazyflie 2.X Drone Implementation
# =============================================================================
#
# This module provides the Crazyflie 2.X drone robot implementation with
# specialized flight control, camera sensors, and navigation capabilities.
#
# =============================================================================

# Standard library imports
import threading
from typing import Dict

# Third-party library imports
import numpy as np
import torch
import carb

# Local project imports
from log.log_manager import LogManager
from physics_engine.isaacsim_utils import Scene, Articulation
from physics_engine.omni_utils import omni

from robot.robot import Robot
from robot.robot_trajectory import Trajectory
from robot.cfg import CfgDroneCf2X
from robot.body.body_drone_cf2x import BodyDroneCf2X
from robot.sensor.camera import CfgCamera, CfgCameraThird
from utils import to_torch

logger = LogManager.get_logger(__name__)


class RobotCf2x(Robot):

    def __init__(
        self,
        cfg_robot: Dict = {},
        scene: Scene = None,
        scene_manager=None,
    ) -> None:
        self.cfg_robot = CfgDroneCf2X(**cfg_robot)
        super().__init__(
            scene=scene,
            scene_manager=scene_manager,
        )

        self.body = BodyDroneCf2X(cfg_robot=self.cfg_robot, scene=self.scene)
        self.is_drone = True

        # 无人机基本属性
        self.position = np.array(
            getattr(cfg_robot, "position", [0.0, 0.0, 0.0]), dtype=np.float32
        )
        self.target_position = self.position.copy()
        self.velocity = np.zeros(3, dtype=np.float32)  # 当前速度 [vx, vy, vz]
        self.default_speed = getattr(cfg_robot, "default_speed", 1.0)  # 默认移动速度
        self.takeoff_height = getattr(cfg_robot, "takeoff_height", 1.0)  # 起飞悬停高度
        self.land_height = getattr(cfg_robot, "land_height", 0.0)  # 降落高度

        # 飞行状态
        self.flight_state = "landed"  # 'landed', 'hovering'
        self.hovering_height = self.takeoff_height  # 悬停高度

        # 路径瞬移相关
        self.waypoints = None  # 路径点列表
        self.current_waypoint_index = 0
        self.teleport_mode = True  # 默认使用瞬移模式

        # 键盘控制
        # self.keyboard_control_enabled = True
        # self._movement_command = np.zeros(3, dtype=np.float32)  # 键盘移动命令
        # self._keyboard_sub = None

        # ROS2初始化
        # self.counter = 0
        # self.pub_period = 50
        # self.previous_pos = None
        # self.movement_threshold = (
        #     0.1  # 移动时，如果两次检测之间的移动距离小于这个阈值，那么就会判定其为异常
        # )

        # —— 平滑导航配置（无人机专用）——
        self.nav_target_xy = None  # 目标点的 XY
        self.nav_max_speed = 0.5  # 导航最大水平速度
        self.nav_slow_radius = 3.0  # 减速起始半径（m）
        self.nav_stop_radius = 0.30  # 到点判定半径（m）
        if self.cfg_robot.disable_gravity:
            self.scene_manager.disable_gravity_for_hierarchy(self.cfg_robot.path_prim_robot)

    def initialize(self):
        """初始化无人机"""
        return

    def takeoff(self):
        """起飞到预设高度并悬停"""
        if self.flight_state == "landed":
            # 获取当前位置，只改变高度
            positions, orientations = self.body.get_world_pose()
            current_pos = positions[0]
            # 上升
            self.hovering_height = self.takeoff_height
            self.move_vertically(self.takeoff_height, "hovering")

        elif self.flight_state == "fluctuating":
            print("无人机正在起降")
        else:
            print("无人机已处于悬停状态")

    def land(self):
        """降落到地面"""
        if self.flight_state == "hovering":
            # 获取当前位置，只改变高度
            positions, orientations = self.body.get_world_pose()
            current_pos = positions[0]
            ground_z = self.get_ground_height_with_raycast(current_pos)
            # 下降
            self.flight_state = "fluctuating"
            self.move_vertically(current_pos[2] - ground_z, "landing")

        elif self.flight_state == "fluctuating":
            print("无人机正在起降")
        else:
            print("无人机已在地面状态")

    def get_ground_height_with_raycast(self, current_position: np.ndarray) -> float:
        """
        使用光线投射来精确获取机器人正下方的地面高度。

        Args:
            current_position (np.ndarray): 机器人当前的 [x, y, z] 世界坐标。

        Returns:
            float: 检测到的地面Z坐标。如果未检测到地面，则返回None。
        """
        # 1. 获取 Isaac Sim 的物理场景接口

        physx_interface = omni.physx.get_physx_scene_query_interface()

        # 2. 定义光线投射的起点和方向
        #    起点应该在机器人当前位置的正上方，以避免光线从机器人内部开始
        ray_origin = carb.Float3(
            current_position[0], current_position[1], current_position[2] + 1.0
        )
        #    方向是垂直向下
        ray_direction = carb.Float3(0, 0, -1)

        # 3. 设置最大探测距离
        max_distance = 100.0  # 例如，最大向下探测100米

        # 4. 执行光线投射
        #    hit_report_multiple 会返回所有碰到的物体
        hit = physx_interface.raycast_closest(ray_origin, ray_direction, max_distance)

        # 5. 处理结果
        if hit["hit"]:
            # "position" 字段是光线与物体碰撞点的精确世界坐标
            ground_position = hit["position"]
            print(f"DEBUG: Raycast hit ground at Z={ground_position[2]:.3f}")
            return ground_position[2]
        else:
            print("WARNING: Raycast did not hit any surface below the robot.")
            return None  # 或者返回一个默认的安全高度，比如 0.0

    def update_position_with_velocity(self, dt):
        """基于速度和时间步长更新位置 (ds = v * dt)"""
        if self.flight_state == "hovering":
            # 计算位移: ds = v * dt
            displacement = self.velocity * dt

            # 更新位置
            self.position += displacement

            # 保持悬停高度（只允许水平移动）
            self.position[2] = self.hovering_height

            # 应用新位置到无人机实体
            self.position = to_torch(self.position)
            _, orientations = self.body.get_world_pose()
            orientation = orientations[0]
            self.body.robot_articulation.set_world_poses(self.position, orientation)

    def set_waypoints(self, waypoints):
        """设置路径点列表进行瞬移"""
        self.waypoints = [np.array(wp, dtype=np.float32) for wp in waypoints]
        self.current_waypoint_index = 0
        print(f"设置了 {len(waypoints)} 个路径点")

        # 如果无人机在地面，先起飞
        if self.flight_state == "landed":
            self.takeoff()

    def teleport_to_waypoint(self, index):
        """瞬移到指定路径点"""
        if 0 <= index < len(self.waypoints):
            target = self.waypoints[index].copy()
            # 确保在悬停高度
            target[2] = self.hovering_height

            # 瞬移
            self.position = to_torch(target)
            _, orientations = self.body.get_world_pose()
            orientation = orientations
            self.body.robot_articulation.set_world_poses(self.position, orientation)

            print(f"瞬移到路径点 {index + 1}/{len(self.waypoints)}: {target}")
            return True
        return False

    def execute_waypoint_sequence(self):
        """执行路径点序列"""
        if self.current_waypoint_index < len(self.waypoints):
            success = self.teleport_to_waypoint(self.current_waypoint_index)
            if success:
                self.current_waypoint_index += 1
                if self.current_waypoint_index >= len(self.waypoints):
                    print("所有路径点执行完成")
                    return True
        return False

    def teleport_to_position(self, position):
        """瞬移到指定位置"""
        target_pos = np.array(position, dtype=np.float32)

        # 如果在地面状态，先起飞
        if self.flight_state == "landed":
            self.takeoff()

        # 确保在悬停高度
        target_pos[2] = self.hovering_height

        self.position = target_pos
        _, orientations = self.body.get_world_pose()
        orientation = orientations[0]
        self.body.robot_articulation.set_world_poses([self.position], [orientation])
        print(f"瞬移到位置: {target_pos}")

    def move_vertically(self, target_height, state):

        pos, quat = self.body.get_world_pose()
        self.position = to_torch(pos)
        cur_pos = np.array(pos[0], dtype=np.float32)
        delta_z = target_height - cur_pos[2]

        if delta_z <= 0.05:
            zero_velocity = torch.zeros((1, 3), dtype=torch.float32)
            self.body.robot_articulation.set_linear_velocities(zero_velocity)
            if state == "landing":
                self.flight_state = "landed"
                print(f"无人机降落到高度: {target_height}m")
            elif state == "hovering":
                # 取消重力
                M = 1
                K = self.body.robot_articulation.num_bodies
                values_array = torch.full((M, K), fill_value=1, dtype=torch.uint8)
                self.body.robot_articulation.set_body_disable_gravity(
                    values=values_array
                )
                self.flight_state = "hovering"
                print(f"无人机起飞到高度: {target_height}m")
            return

        linear_velocity = torch.tensor([0, 0, delta_z], dtype=torch.float32)
        self.body.robot_articulation.set_linear_velocities(linear_velocity)
        self.velocity = linear_velocity

    def on_physics_step(self, step_size):
        super().on_physics_step(step_size)

        # self.counter += 1

        # if (
        #     getattr(self, "flag_action_navigation", False)
        #     and self.nav_target_xy is not None
        # ):
        #     pass
        # elif self.flight_state == "fluctuating":
        #     self.move_vertically()
        # else:
        #
        #     if hasattr(self, "waypoints") and self.waypoints:
        #         self.execute_waypoint_sequence()
        #     if self.flight_state == "hovering":
        #         self.update_position_with_velocity(step_size)
