import numpy as np
import torch

import omni
import omni.appwindow
from isaacsim.core.api.scenes import Scene
from isaacsim.core.prims import Articulation

from camera.camera_cfg import CameraCfg
from camera.camera_third_cfg import CameraThirdCfg
from map.map_grid_map import GridMap
from path_planning.path_planning_astar import AStar
from log.log_manager import LogManager
from controller.controller_cf2x import ControllerCf2x
from robot.robot_base import RobotBase
from robot.robot_trajectory import Trajectory
from robot.robot_cfg.robot_cfg_drone_cf2x import RobotCfgCf2x
from utils import to_torch

import threading

logger = LogManager.get_logger(__name__)


class RobotCf2x(RobotBase):
    """
    CF2x无人机控制类

     键盘控制说明:
    - Q键: 起飞到预设高度并保持悬停
    - E键: 降落到地面
    - W键: 向前移动 (+X方向)
    - S键: 向后移动 (-X方向)
    - A键: 向左移动 (+Y方向)
    - D键: 向右移动 (-Y方向)
    """

    def __init__(
        self,
        cfg_body: RobotCfgCf2x,
        cfg_camera: CameraCfg = None,
        cfg_camera_third_person: CameraThirdCfg = None,
        scene: Scene = None,
        map_grid: GridMap = None,
        scene_manager=None,
    ) -> None:
        super().__init__(
            cfg_body,
            cfg_camera,
            cfg_camera_third_person,
            scene,
            map_grid,
            scene_manager=scene_manager,
        )
        self.create_robot_entity()

        self.is_drone = True  # 标记为无人机
        self.controller = ControllerCf2x()
        self.map_grid = map_grid

        # 无人机基本属性
        self.position = np.array(
            getattr(cfg_body, "position", [0.0, 0.0, 0.0]), dtype=np.float32
        )
        self.target_position = self.position.copy()
        self.velocity = np.zeros(3, dtype=np.float32)  # 当前速度 [vx, vy, vz]
        self.default_speed = getattr(cfg_body, "default_speed", 1.0)  # 默认移动速度
        self.takeoff_height = getattr(cfg_body, "takeoff_height", 1.0)  # 起飞悬停高度
        self.land_height = getattr(cfg_body, "land_height", 0.0)  # 降落高度

        # 飞行状态
        self.flight_state = "landed"  # 'landed', 'hovering'
        self.hovering_height = self.takeoff_height  # 悬停高度

        # 路径瞬移相关
        self.waypoints = [[0, 0, 1], [5, 0, 1], [5, 5, 1], [0, 5, 1]]  # 路径点列表
        self.current_waypoint_index = 0
        self.teleport_mode = True  # 默认使用瞬移模式

        # 键盘控制
        self.keyboard_control_enabled = True
        self._movement_command = np.zeros(3, dtype=np.float32)  # 键盘移动命令
        self._keyboard_sub = None

        # ROS2初始化
        self.ros2_initialized = False
        self._ros2_lock = threading.Lock()  # ← 这里初始化了互斥锁

        self.counter = 0
        self.pub_period = 50
        self.previous_pos = None
        self.movement_threshold = (
            0.1  # 移动时，如果两次检测之间的移动距离小于这个阈值，那么就会判定其为异常
        )

        # —— 平滑导航配置（无人机专用）——
        self.nav_target_xy = None  # 目标点的 XY
        self.nav_max_speed = 0.5  # 导航最大水平速度
        self.nav_slow_radius = 3.0  # 减速起始半径（m）
        self.nav_stop_radius = 0.30  # 到点判定半径（m）

        # self.node.register_feedback_publisher(
        #     robot_class=self.cfg_body.type,
        #     robot_id=self.cfg_body.id,
        #     qos=50
        # )
        # self.node.register_motion_publisher(
        #     robot_class=self.cfg_body.type,
        #     robot_id=self.cfg_body.id,
        #     qos=50
        # )
        # self.node.register_cmd_subscriber(
        #     robot_class=self.cfg_body.type,
        #     robot_id=self.cfg_body.id,
        #     callback=self.cmd_vel_callback,
        #     qos=50
        # )

        self.ros2_initialized = True

        # 初始化键盘事件监听
        self._setup_keyboard_events()

    def create_robot_entity(self):
        """
        初始化机器人关节树
        """
        self.robot_entity = Articulation(
            prim_paths_expr=self.cfg_body.prim_path_swarm,
            name=self.cfg_body.name,
            positions=to_torch(self.cfg_body.position).reshape(1, 3),
            orientations=to_torch(self.cfg_body.quat).reshape(1, 4),
        )

    def _setup_keyboard_events(self):
        """设置键盘事件监听"""
        import carb

        try:
            appwindow = omni.appwindow.get_default_app_window()
            input_iface = carb.input.acquire_input_interface()

            self._keyboard_sub = input_iface.subscribe_to_keyboard_events(
                appwindow.get_keyboard(), self._on_keyboard_event
            )
            print("无人机键盘控制设置成功")
            print("控制说明: Q起飞, E降落, WASD移动")
        except Exception as e:
            print(f"键盘事件监听设置失败: {e}")
            self._keyboard_sub = None

    def _on_keyboard_event(self, event, *args, **kwargs):
        """键盘事件回调函数"""
        import carb

        try:
            # 按键按下事件
            if event.type == carb.input.KeyboardEventType.KEY_PRESS:
                if event.input.name == "Q":
                    self.takeoff()
                elif event.input.name == "E":
                    self.land()
                elif event.input.name == "W":
                    self._movement_command[0] = self.default_speed  # 向前
                elif event.input.name == "S":
                    self._movement_command[0] = -self.default_speed  # 向后
                elif event.input.name == "A":
                    self._movement_command[1] = self.default_speed  # 向左
                elif event.input.name == "D":
                    self._movement_command[1] = -self.default_speed  # 向右

            # 按键释放事件
            elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
                if event.input.name in ["W", "S"]:
                    self._movement_command[0] = 0.0
                elif event.input.name in ["A", "D"]:
                    self._movement_command[1] = 0.0

            return True
        except Exception as e:
            print(f"键盘事件处理错误: {e}")
            return False

    def initialize(self):
        """初始化无人机"""
        return

    def apply_action(self, action=None):
        """应用动作"""
        if action is not None:
            self.robot_entity.apply_action(self.controller.velocity(action))
        return

    def forward(self, velocity=None):
        """前进动作"""
        if velocity is not None:
            self.robot_entity.apply_action(self.controller.forward(velocity))
        return

    def takeoff(self):
        """起飞到预设高度并悬停"""
        if self.flight_state == "landed":
            # 获取当前位置，只改变高度
            positions, orientations = self.robot_entity.get_world_poses()
            current_pos = positions[0]

            # 设置新的悬停位置
            self.position = torch.as_tensor(
                [current_pos[0], current_pos[1], self.takeoff_height],
                dtype=torch.float32,
            )
            self.hovering_height = self.takeoff_height

            # 直接设置位置（瞬移到悬停高度）
            orientation = orientations[0]
            self.robot_entity.set_world_poses(self.position, orientation)
            # 设置速度为0
            zero_velocities = torch.zeros((1, 6), dtype=torch.float32)
            self.robot_entity.set_velocities(velocities=zero_velocities)

            # 更新状态
            self.flight_state = "hovering"
            self.velocity = np.zeros(3, dtype=np.float32)  # 悬停时速度为0

            # 取消重力
            M = 1
            K = self.robot_entity.num_bodies
            values_array = torch.full((M, K), fill_value=1, dtype=torch.uint8)
            self.robot_entity.set_body_disable_gravity(values=values_array)

            print(f"无人机起飞到高度: {self.takeoff_height}m，进入悬停状态")
        else:
            print("无人机已在飞行状态")

    def get_ground_height_with_raycast(self, current_position: np.ndarray) -> float:
        """
        使用光线投射来精确获取机器人正下方的地面高度。

        Args:
            current_position (np.ndarray): 机器人当前的 [x, y, z] 世界坐标。

        Returns:
            float: 检测到的地面Z坐标。如果未检测到地面，则返回None。
        """
        # 1. 获取 Isaac Sim 的物理场景接口
        import omni.physx
        import carb

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

    def land(self):
        """降落到地面"""
        if self.flight_state == "hovering":
            # 获取当前位置，只改变高度
            positions, orientations = self.robot_entity.get_world_poses()
            current_pos = positions[0]

            # 获取地面的高度
            ground_z = self.get_ground_height_with_raycast(current_pos)

            # 设置降落位置
            self.position = np.array(
                [current_pos[0], current_pos[1], current_pos[2] - ground_z],
                dtype=np.float32,
            )

            # 直接设置位置（瞬移到地面）
            orientation = orientations[0]
            self.robot_entity.set_world_poses([self.position], [orientation])

            # 设置速度为0
            zero_velocities = np.zeros((1, 6), dtype=np.float32)
            self.robot_entity.set_velocities(velocities=zero_velocities)

            # 立即清零所有运动相关变量
            self.velocity = np.zeros(3, dtype=np.float32)  # 清零速度
            self._movement_command = np.zeros(3, dtype=np.float32)  # 清除移动命令

            # 更新状态
            self.flight_state = "landed"

            print(f"无人机降落到高度: {self.current_pos[2] - ground_z}m")
        else:
            print("无人机已在地面状态")

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
            _, orientations = self.robot_entity.get_world_poses()
            orientation = orientations[0]
            self.robot_entity.set_world_poses(self.position, orientation)

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
            _, orientations = self.robot_entity.get_world_poses()
            orientation = orientations[0]
            self.robot_entity.set_world_poses(self.position, orientation)

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
        _, orientations = self.robot_entity.get_world_poses()
        orientation = orientations[0]
        self.robot_entity.set_world_poses([self.position], [orientation])
        print(f"瞬移到位置: {target_pos}")

    def keyboard_control(self, dt):
        """键盘控制处理"""
        if self.flight_state == "hovering" and self.keyboard_control_enabled:
            # 设置速度为键盘命令
            self.velocity = self._movement_command.copy()
            # 注意：位置更新在 on_physics_step 中统一处理
        elif self.flight_state == "landed":
            # 地面状态下确保速度为0
            self.velocity = np.zeros(3, dtype=np.float32)

    def cmd_vel_callback(self, msg):
        # ROS2回调：接收目标速度，设置无人机移动命令
        with self._ros2_lock:
            self._movement_command[0] = float(msg.linear.x)
            self._movement_command[1] = float(msg.linear.y)
            self._movement_command[2] = float(msg.linear.z)

    def move_to(self):

        import numpy as np

        # 落地就先起飞到悬停高度（一次性瞬移到 hover 高度即可）
        if self.flight_state == "landed":
            self.takeoff()

        # === 读取当前位姿（用仿真里的真实位姿，不再用 self.position 作为“真值”） ===
        positions, orientations = self.robot_entity.get_world_poses()

        if self.counter % self.pub_period == 0:
            self._publish_feedback(
                params=self._params_from_pose(positions, orientations),
                progress=self._calc_dist(positions, self.nav_end) * 100 / self.nav_dist,
            )

        cur_pos = np.array(positions[0], dtype=np.float32)
        pos_xy = cur_pos[:2]

        target_xy = self.nav_target_xy.astype(np.float32)
        delta = target_xy - pos_xy
        dist = float(np.linalg.norm(delta))

        # 到点判定
        stop_r = float(self.nav_stop_radius)

        if dist <= stop_r:
            # 停车：把刚体速度清零
            zero_velocity = torch.zeros((1, 3), dtype=torch.float32)
            self.velocity = torch.zeros((3,), dtype=torch.float32)
            self.nav_target_xy = None

            self.robot_entity.set_linear_velocities(zero_velocity)
            self.robot_entity.set_angular_velocities(zero_velocity)

            self.flag_action_navigation = False
            self.state_skill_complete = True
            self._publish_feedback(
                params=self._params_from_pose(positions, orientations), progress=100
            )
            return True

        # 期望速度：指向目标，近处线性减速
        vmax = float(self.nav_max_speed)
        slow_r = float(self.nav_slow_radius)
        dir_xy = delta / (dist + 1e-6)
        spd = vmax * (dist / slow_r) if dist < slow_r else vmax
        v_world = torch.tensor(
            [dir_xy[0] * spd, dir_xy[1] * spd, 0.0], dtype=torch.float32
        )

        # === 把速度交给“根刚体” ===
        self.robot_entity.set_linear_velocities(v_world)

        # 可选：如果希望机体绕 z 朝速度方向缓慢对齐，也可以给一个角速度：
        # self.robot_entity.set_angular_velocity(np.array([0.0, 0.0, yaw_rate], dtype=np.float32))

        self.velocity = v_world

    def on_physics_step(self, step_size):
        super().on_physics_step(step_size)

        # self._publish_status_pose()
        self.counter += 1

        # 未激活或无目标：什么都不做
        if (
            getattr(self, "flag_action_navigation", False)
            and self.nav_target_xy is not None
            and hasattr(self, "flag_world_reset")
            and self.flag_world_reset
        ):
            self.move_to()
            # if self.counter % self.pub_period == 0:
            #     self._publish_feedback_pose()
        else:
            if self.keyboard_control_enabled:
                self.keyboard_control(step_size)
            else:
                if hasattr(self, "waypoints") and self.waypoints:
                    self.execute_waypoint_sequence()
            if self.flight_state == "hovering":
                self.update_position_with_velocity(step_size)

    def enable_keyboard_control(self, enable=True):
        """启用或禁用键盘控制"""
        self.keyboard_control_enabled = enable
        if not enable:
            # 禁用键盘控制时清零移动命令
            self._movement_command = np.zeros(3, dtype=np.float32)
            self.velocity = np.zeros(3, dtype=np.float32)
        print(f"键盘控制已{'启用' if enable else '禁用'}")

    def __del__(self):
        self._cleanup_keyboard_events()

    def _cleanup_keyboard_events(self):
        if self._keyboard_sub is not None:
            try:
                import carb

                input_iface = carb.input.acquire_input_interface()
                input_iface.unsubscribe_to_keyboard_events(self._keyboard_sub)
                self._keyboard_sub = None
            except Exception as e:
                print(f"键盘事件清理失败: {e}")

    # 以下方法保留用于兼容性，主要用于地面机器人

    def move_along_path(self, path: list = None, flag_reset: bool = False):
        """让机器人沿着路径点运动"""
        return False

    def navigate_to(
        self, pos_target: np.ndarray = None, reset_flag: bool = False, **kwargs
    ):
        """
        导航到目标位置。
        - 无人机: 执行特殊的直线飞行逻辑。
        - 地面车: 调用父类的 A* 路径规划实现。
        """
        if pos_target is None:
            raise ValueError("no target position")

        # 1. --- 处理子类的特殊情况：无人机 ---
        if hasattr(self, "is_drone") and self.is_drone:
            logger.info("Executing drone-specific navigation.")
            # 支持传入 2D 或 3D；若含 Z 则忽略
            self.nav_end = np.array(pos_target, dtype=np.float32)
            self.nav_target_xy = self.nav_end[:2].copy()
            self.nav_active = True

            # 设置通用状态标志
            self.flag_action_navigation = True
            self.state_skill = "navigate_to"
            self.state_skill_complete = False

            if hasattr(self, "keyboard_control_enabled"):
                self.keyboard_control_enabled = False

            return True  # 子类方法返回一个状态

        # 2. --- 对于所有其他情况（地面车），委托给父类 ---
        else:
            logger.info("Delegating to superclass for ground navigation.")
            # 直接调用父类的同名方法，并将所有相关参数传递过去。
            # 父类会处理 A* 规划、状态设置等所有事情。
            super().navigate_to(
                pos_target=self.nav_end,
                reset_flag=reset_flag,
                # 允许传递父类需要的额外参数，如 load_from_file
                **kwargs,
            )

            # 保持与无人机分支一致的返回值
            return True

    def pick_up(self):
        """拾取物品"""
        pass

    def explore_zone(
        self,
        zone_corners: list = None,
        scane_direction: str = "horizontal",
        reset_flag: bool = False,
    ):
        """探索指定区域"""
        min_x = min(corner[0] for corner in zone_corners)
        max_x = max(corner[0] for corner in zone_corners)
        min_y = min(corner[1] for corner in zone_corners)
        max_y = max(corner[1] for corner in zone_corners)

        scan_direction = scane_direction

        import math

        effective_width = 2 * self.view_radius * math.sin(self.viewgraph_angle / 2)
        scan_line_spacing = effective_width * 0.8

        scan_lines = []
        if scan_direction == "horizontal":
            y = min_y
            while y <= max_y:
                scan_lines.append(y)
                y += scan_line_spacing
        else:
            x = min_x
            while x <= max_x:
                scan_lines.append(x)
                x += scan_line_spacing

        path_points = []
        if scan_direction == "horizontal":
            for i, y in enumerate(scan_lines):
                if i % 2 == 0:
                    path_points.append([min_x, y])
                    path_points.append([max_x, y])
                else:
                    path_points.append([max_x, y])
                    path_points.append([min_x, y])
        else:
            for i, x in enumerate(scan_lines):
                if i % 2 == 0:
                    path_points.append([x, min_y])
                    path_points.append([x, max_y])
                else:
                    path_points.append([x, max_y])
                    path_points.append([x, min_y])

        return path_points
