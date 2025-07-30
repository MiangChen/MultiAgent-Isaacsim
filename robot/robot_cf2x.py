import numpy as np
import omni
import omni.appwindow
from isaacsim.core.api.scenes import Scene
from isaacsim.robot.wheeled_robots.robots import WheeledRobot

from controller.controller_pid import ControllerPID
from camera.camera_cfg import CameraCfg
from camera.camera_third_person_cfg import CameraThirdPersonCfg
from map.map_grid_map import GridMap
from path_planning.path_planning_astar import AStar
from controller.controller_cf2x import ControllerCf2x
from robot.robot_base import RobotBase
from robot.robot_trajectory import Trajectory
from robot.robot_cfg_drone_cf2x import RobotCfgCf2x
import rclpy
from geometry_msgs.msg import Pose, Twist
import threading


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

    def __init__(self, cfg_body: RobotCfgCf2x, cfg_camera: CameraCfg = None,
                 cfg_camera_third_person: CameraThirdPersonCfg = None, scene: Scene = None,
                 map_grid: GridMap = None) -> None:
        super().__init__(cfg_body, cfg_camera, cfg_camera_third_person, scene, map_grid)

        self.is_drone = True  # 标记为无人机
        self.controller = ControllerCf2x()
        self.map_grid = map_grid

        # 无人机基本属性
        self.position = np.array(getattr(cfg_body, 'position', [0.0, 0.0, 0.0]), dtype=np.float32)
        self.target_position = self.position.copy()
        self.velocity = np.zeros(3, dtype=np.float32)  # 当前速度 [vx, vy, vz]
        self.default_speed = getattr(cfg_body, 'default_speed', 1.0)  # 默认移动速度
        self.takeoff_height = getattr(cfg_body, 'takeoff_height', 1.0)  # 起飞悬停高度
        self.land_height = getattr(cfg_body, 'land_height', 0.0)  # 降落高度

        # 飞行状态
        self.flight_state = 'landed'  # 'landed', 'hovering'
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
        self._ros2_lock = threading.Lock()
        try:
            rclpy.init(args=None)
            from rclpy.node import Node
            self.ros2_node = rclpy.create_node(f"cf2x_publisher_{getattr(cfg_body, 'id', '0')}")
            self.pose_pub = self.ros2_node.create_publisher(Pose, 'drone/pose', 10)
            self.twist_pub = self.ros2_node.create_publisher(Twist, 'drone/twist', 10)
            # 订阅无人机目标速度
            self.cmd_vel_sub = self.ros2_node.create_subscription(
                Twist, 'drone/cmd_vel', self.cmd_vel_callback, 10)
            self.ros2_initialized = True
        except Exception as e:
            print(f"ROS2初始化失败: {e}")

        # 初始化键盘事件监听
        self._setup_keyboard_events()

        return

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
        if self.flight_state == 'landed':
            # 获取当前位置，只改变高度
            positions, orientations = self.robot_entity.get_world_poses()
            current_pos = positions[0]

            # 设置新的悬停位置
            self.position = np.array([current_pos[0], current_pos[1], self.takeoff_height], dtype=np.float32)
            self.hovering_height = self.takeoff_height

            # 直接设置位置（瞬移到悬停高度）
            orientation = orientations[0]
            self.robot_entity.set_world_poses([self.position], [orientation])

            # 更新状态
            self.flight_state = 'hovering'
            self.velocity = np.zeros(3, dtype=np.float32)  # 悬停时速度为0

            print(f"无人机起飞到高度: {self.takeoff_height}m，进入悬停状态")
        else:
            print("无人机已在飞行状态")

    def land(self):
        """降落到地面"""
        if self.flight_state == 'hovering':
            # 获取当前位置，只改变高度
            positions, orientations = self.robot_entity.get_world_poses()
            current_pos = positions[0]

            # 设置降落位置
            self.position = np.array([current_pos[0], current_pos[1], self.land_height], dtype=np.float32)

            # 直接设置位置（瞬移到地面）
            orientation = orientations[0]
            self.robot_entity.set_world_poses([self.position], [orientation])

            # 立即清零所有运动相关变量
            self.velocity = np.zeros(3, dtype=np.float32)  # 清零速度
            self._movement_command = np.zeros(3, dtype=np.float32)  # 清除移动命令

            # 更新状态
            self.flight_state = 'landed'

            print(f"无人机降落到高度: {self.land_height}m")
        else:
            print("无人机已在地面状态")
        # import omni.physx  # 导入 PhysX 接口
        # from pxr import PhysxSchema, UsdPhysics, Gf  # 如果需要，导入 USD 和 PhysX Schema
        #
        # # 假设在您的类中已经有 PhysX 接口可用，或者在初始化时获取
        # # 示例：获取 PhysX 接口
        # physx_interface = omni.physx.acquire_physx_interface()
        #
        # # 修改降落逻辑
        # if self.flight_state == 'hovering':
        #     # 获取当前位置
        #     positions, orientations = self.robot_entity.get_world_poses()
        #     current_pos = positions[0]
        #
        #     # 执行 raycast 来检测地面高度
        #     # 从当前 x, y 位置向下投射射线，查询 z 方向的碰撞点
        #     ray_origin = Gf.Vec3f(float(current_pos[0]), float(current_pos[1]), float(current_pos[2]))  # 起始点：当前无人机位置
        #     ray_direction = Gf.Vec3f(0, 0, -1)  # 向下方向（负 z 轴）
        #     max_distance = 10.0  # 最大射线长度，单位米，根据场景调整
        #
        #     # 执行 raycast 查询
        #     hit_results = physx_interface.raycast(ray_origin, ray_direction, max_distance)
        #
        #     if hit_results and len(hit_results) > 0:
        #         # hit_results 是一个列表，包含命中信息
        #         # 假设 hit_results[0] 是第一个命中点，获取 hit position
        #         hit_position = hit_results[0].position  # 或者根据实际 API 获取
        #         ground_height = hit_position[2]  # z 坐标是地面高度
        #     else:
        #         # 如果没有命中，假设一个默认高度，例如 0
        #         ground_height = 0.0
        #         print("警告：射线未命中任何物体，假设地面高度为 0")
        #
        #     # 设置降落高度，确保有一个小间隙避免直接碰撞（例如 +0.1m）
        #     land_z = ground_height + 0.1  # 或者根据无人机大小调整
        #
        #     # 设置目标位置，使用 raycast 获取的地面高度
        #     self.position = np.array([current_pos[0], current_pos[1], land_z], dtype=np.float32)
        #
        #     # 为了避免直接瞬移引起的碰撞，最好使用平滑移动
        #     # 这里可以添加一个降落控制循环，或者使用 PhysX 的关节控制
        #     # 但如果必须瞬移，确保位置是安全的
        #     orientation = orientations[0]
        #     self.robot_entity.set_world_poses([self.position], [orientation])
        #
        #     # 立即清零所有运动相关变量
        #     self.velocity = np.zeros(3, dtype=np.float32)  # 清零速度
        #     self._movement_command = np.zeros(3, dtype=np.float32)  # 清除移动命令
        #
        #     # 更新状态
        #     self.flight_state = 'landed'
        #
        #     print(f"无人机降落到高度: {land_z}m，基于 raycast 检测的地面高度")
        # else:
        #     print("无人机已在地面状态")

    def update_position_with_velocity(self, dt):
        """基于速度和时间步长更新位置 (ds = v * dt)"""
        if self.flight_state == 'hovering':
            # 计算位移: ds = v * dt
            displacement = self.velocity * dt

            # 更新位置
            self.position += displacement

            # 保持悬停高度（只允许水平移动）
            self.position[2] = self.hovering_height

            # 应用新位置到无人机实体
            _, orientations = self.robot_entity.get_world_poses()
            orientation = orientations[0]
            self.robot_entity.set_world_poses([self.position], [orientation])

    def set_waypoints(self, waypoints):
        """设置路径点列表进行瞬移"""
        self.waypoints = [np.array(wp, dtype=np.float32) for wp in waypoints]
        self.current_waypoint_index = 0
        print(f"设置了 {len(waypoints)} 个路径点")

        # 如果无人机在地面，先起飞
        if self.flight_state == 'landed':
            self.takeoff()

    def teleport_to_waypoint(self, index):
        """瞬移到指定路径点"""
        if 0 <= index < len(self.waypoints):
            target = self.waypoints[index].copy()
            # 确保在悬停高度
            target[2] = self.hovering_height

            # 瞬移
            self.position = target
            _, orientations = self.robot_entity.get_world_poses()
            orientation = orientations[0]
            self.robot_entity.set_world_poses([self.position], [orientation])

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
        if self.flight_state == 'landed':
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
        if self.flight_state == 'hovering' and self.keyboard_control_enabled:
            # 设置速度为键盘命令
            self.velocity = self._movement_command.copy()
            # 注意：位置更新在 on_physics_step 中统一处理
        elif self.flight_state == 'landed':
            # 地面状态下确保速度为0
            self.velocity = np.zeros(3, dtype=np.float32)

    def cmd_vel_callback(self, msg):
        # ROS2回调：接收目标速度，设置无人机移动命令
        with self._ros2_lock:
            self._movement_command[0] = float(msg.linear.x)
            self._movement_command[1] = float(msg.linear.y)
            self._movement_command[2] = float(msg.linear.z)

    def on_physics_step(self, step_size):
        super().on_physics_step(step_size)
        if self.is_drone:
            if self.keyboard_control_enabled:
                self.keyboard_control(step_size)
            else:
                if hasattr(self, 'waypoints') and self.waypoints:
                    self.execute_waypoint_sequence()
            if self.flight_state == 'hovering':
                self.update_position_with_velocity(step_size)
        # ROS2发布无人机状态，并处理订阅回调
        if getattr(self, 'ros2_initialized', False):
            pose = Pose()
            pose.position.x = float(self.position[0])
            pose.position.y = float(self.position[1])
            pose.position.z = float(self.position[2])
            twist = Twist()
            twist.linear.x = float(self.velocity[0])
            twist.linear.y = float(self.velocity[1])
            twist.linear.z = float(self.velocity[2])
            self.pose_pub.publish(pose)
            self.twist_pub.publish(twist)
            rclpy.spin_once(self.ros2_node, timeout_sec=0)
        if hasattr(self, 'flag_world_reset') and self.flag_world_reset:
            if hasattr(self, 'flag_action_navigation') and self.flag_action_navigation:
                self.move_along_path()
            if hasattr(self, 'velocity') and not self.is_drone:
                self.apply_action(action=self.velocity)

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
        if getattr(self, 'ros2_initialized', False):
            try:
                self.ros2_node.destroy_node()
                rclpy.shutdown()
            except Exception as e:
                print(f"ROS2关闭失败: {e}")

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
    def move_to(self, target_postion):
        """移动到目标位置"""
        import numpy as np
        positions, orientations = self.robot_entity.get_world_poses()
        car_yaw_angle = self.quaternion_to_yaw(orientations[0])

        car_to_target_angle = np.arctan2(target_postion[1] - positions[0][1], target_postion[0] - positions[0][0])
        delta_angle = car_to_target_angle - car_yaw_angle

        if abs(delta_angle) < 0.017:
            delta_angle = 0
        elif delta_angle < -np.pi:
            delta_angle += 2 * np.pi
        elif delta_angle > np.pi:
            delta_angle -= 2 * np.pi

        if np.linalg.norm(target_postion[0:2] - positions[0][0:2]) < 0.1:
            self.velocity = [0, 0]
            return True

        v_rotation = self.pid_angle.compute(delta_angle, dt=1 / 60)
        v_forward = 15

        v_left = v_forward + v_rotation
        v_right = v_forward - v_rotation
        v_max = 20

        v_left = np.clip(v_left, -v_max, v_max)
        v_right = np.clip(v_right, -v_max, v_max)

        self.velocity = [v_left, v_right]
        return False

    def move_along_path(self, path: list = None, flag_reset: bool = False):
        """让机器人沿着路径点运动"""
        if flag_reset == True:
            self.path_index = 0
            self.path = path

        if hasattr(self, 'path') and hasattr(self, 'path_index') and self.path_index < len(self.path):
            reach_flat = self.move_to(self.path[self.path_index])
            if reach_flat == True:
                self.path_index += 1
            return False
        else:
            if hasattr(self, 'flag_action_navigation'):
                self.flag_action_navigation = False
            if hasattr(self, 'state_skill_complete'):
                self.state_skill_complete = True
            return True

    def navigate_to(self, pos_target: np.array = None, reset_flag: bool = False):
        """导航到目标位置"""
        if pos_target is None:
            raise ValueError("no target position")
        elif pos_target[2] != 0:
            raise ValueError("小车的z轴高度得在平面上")

        positions, _ = self.robot_entity.get_world_poses()
        pos_robot = positions[0]
        pos_index_target = self.map_grid.compute_index(pos_target)
        pos_index_robot = self.map_grid.compute_index(pos_robot)

        grid_map = self.map_grid.value_map
        grid_map[pos_index_robot] = self.map_grid.empty_cell

        planner = AStar(self.map_grid.value_map, obs_value=1.0, free_value=0.0, directions="eight")
        path = planner.find_path(tuple(pos_index_robot), tuple(pos_index_target))

        real_path = np.zeros_like(path, dtype=np.float32)
        for i in range(path.shape[0]):
            real_path[i] = self.map_grid.pos_map[tuple(path[i])]
            real_path[i][-1] = 0

        self.move_along_path(real_path)
        if hasattr(self, 'flag_action_navigation'):
            self.flag_action_navigation = True
        if hasattr(self, 'state_skill'):
            self.state_skill = 'navigate_to'
        if hasattr(self, 'state_skill_complete'):
            self.state_skill_complete = False
        return

    def pick_up(self):
        """拾取物品"""
        pass

    def explore_zone(self, zone_corners: list = None, scane_direction: str = "horizontal", reset_flag: bool = False):
        """探索指定区域"""
        min_x = min(corner[0] for corner in zone_corners)
        max_x = max(corner[0] for corner in zone_corners)
        min_y = min(corner[1] for corner in zone_corners)
        max_y = max(corner[1] for corner in zone_corners)

        scan_direction = scane_direction

        import math
        effective_width = 2 * self.view_radius * math.sin(self.view_angle / 2)
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
