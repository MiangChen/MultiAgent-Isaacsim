from typing import List, Optional, Tuple

from map.map_grid_map import GridMap
from robot.robot_cfg import RobotCfg
from robot.robot_trajectory import Trajectory

import numpy as np
from isaacsim.core.api.scenes import Scene
from isaacsim.core.prims import RigidPrim
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.types import ArticulationActions


class RobotBase:
    """Base class of robot."""

    def __init__(self, config: RobotCfg, scene: Scene, map_grid: GridMap = None):
        self.config = config
        self.scene = scene
        # 代表机器人的实体
        self.robot_entity: Articulation = None
        self.prim_path: str = ''
        # 机器人的历史轨迹
        self.trajectory: Trajectory = None
        # 机器人的控制器
        self.controllers: dict = {}  # 用于存储多个控制器, 'controller name': function
        self.control_mode: str = ''  # 'joint_efforts', 'joint_velocities', 'joint_positions', 'joint_indices', 'joint_names'
        self.action: np.ndarray = None
        # 机器人的技能
        self.skills: dict = {}  # 用于记录其技能: 'skill name': function
        # 用于地图
        self.map_grid: GridMap = map_grid  # 用于存储一个实例化的gridmap
        # 用于回调函数中
        self.flag_world_reset: bool = False  # 用来记录下世界是不是被初始化了
        self.flag_action_navigation: bool = False  # 用来记录是不是启动导航了
        # 用于PDDL, 记录当前用的 skill, 之所以用skill, 是为了和action区分, action一般是底层的关节动作
        self.state_skill: str = ''
        self.state_skill_complete: bool = True  # 默认状态, 没有skill要做, 所以是True
        # 传感器相关 机器人的感知范围
        self.sensors: dict = {}
        self.view_angle: float = 2 * np.pi / 3  # 感知视野 弧度
        self.view_radius: float = 2  # 感知半径 米

    # def apply_action(self, action: ArticulationActions) -> None:
    #     """Apply actions of controllers to robot.
    #
    #     Args:
    #         action (ArticulationActions):
    #     """
    #     self.robot_entity.apply_action(action)
    #     raise NotImplementedError()

    def cleanup(self):
        for controller in self.controllers.values():
            controller.cleanup()
        for sensor in self.sensors.values():
            sensor.cleanup()
        for rigid_body in self.get_rigid_bodies():
            self._scene.remove_object(rigid_body.name_prefix)
            log.debug(f'rigid body {rigid_body} removed')
        log.debug(f'robot {self.name} clean up')

    def forward(self, velocity=None):
        raise NotImplementedError()

    def get_controllers(self):
        return self.controllers

    def get_obs(self) -> dict:
        """Get observation of robot, including controllers, sensors, and world pose.

        Raises:
            NotImplementedError: _description_
        """
        raise NotImplementedError()

    def get_rigid_bodies(self) -> List[RigidPrim]:
        raise NotImplementedError()

    def get_robot_base(self) -> RigidPrim:
        """
        Get base link of robot.

        Returns:
            RigidPrim: rigid prim of robot base link.
        """
        raise NotImplementedError()

    def get_robot_scale(self) -> np.ndarray:
        """Get robot scale.

        Returns:
            np.ndarray: robot scale in (x, y, z).
        """
        return self.robot_entity.get_local_scale()

    def get_robot_articulation(self) -> Articulation:
        """Get isaac robots instance (articulation).

        Returns:
            Robot: robot articulation.
        """
        return self.robot_entity

    def get_world_poses(self) -> Tuple[np.ndarray, np.ndarray]:
        pos_IB, q_IB = self.robot_entity.get_world_poses()
        pos_IB, q_IB = pos_IB[0], q_IB[0]
        return pos_IB, q_IB

    def initialize(self) -> None:
        return

    def move_to(self, target_position: np.ndarray, target_orientation: np.ndarray) -> bool:
        """

        Args:
            target_position: 目标位置 , shape (3,)
            target_orientation: 目标朝向, shape (4,)
        Returns:
            bool: 是否到达了目标点附近
        """
        raise NotImplementedError()

    def move_along_path(self, path: list = None, flag_reset: bool = False) -> bool:
        """
        让机器人沿着一个list的路径点运动
        需求: 在while外面 能够记录已经到达的点, 每次到达某个目标点的 10cm附近,就认为到了, 然后准备下一个点

        """
        if flag_reset == True:
            self.path_index = 0
            self.path = path

        if self.path_index < len(self.path):  # 当index==len的时候, 就已经到达目标了
            flag_reach = self.move_to(self.path[self.path_index])
            if flag_reach == True:
                self.path_index += 1
            return False
        else:
            self.flag_action_navigation = False
            self.state_skill_complete = True
            return True

    def navigate_to(self, target_pos, reset_flag: bool = False):
        """
        让机器人导航到某一个位置,
        不需要输入机器人的起始位置, 因为机器人默认都是从当前位置出发的;
        本质是使用A*等算法, 规划一系列的路径
        """
        raise NotImplementedError()

    def on_physics_step(self, step_size) -> None:
        """
        Args:
            step_size:  dt 时间间隔
        """
        raise NotImplementedError()

    def post_reset(self):
        """Set up things that happen after the world resets."""
        for sensor in self.sensors.values():
            sensor.post_reset()
        return

    def put_down(self):
        """
        让机器人把一个东西放下, 可以指定
        Returns:
        """
        raise NotImplementedError()

    def pick_up(self):
        """
        让机器人拿起来某一个东西, 需要指定物品的名称, 并且在操作范围内
        Returns:
        """
        raise NotImplementedError()

    def quaternion_to_yaw(self, orientation: Tuple[float, float, float, float]) -> float:
        """

        Args:
            orientation: 四元数存储的朝向


        Returns:
            yaw: 绕着z轴的旋转角度
        """
        import math
        alpha = 0.1
        norm = math.sqrt(sum(comp ** 2 for comp in orientation))
        if abs(norm - 1) > 0.1:
            print("没有归一")
        qw, qx, qy, qz = orientation
        # 计算方位角（绕 z 轴的旋转角度）
        sinr_cosp = 2.0 * (qw * qz + qx * qy)
        cosr_cosp = 1.0 - 2.0 * (qy ** 2 + qz ** 2)
        # 转换为 [-π, π] 范围，逆时针为正
        yaw = math.atan2(sinr_cosp, cosr_cosp)
        # 本来要做连续性, 避免pi -pi这个奇异点的, 但是实际测试完后发现并不需要了
        # if self.last_yaw is not None:
        #     delta_yaw = yaw - self.last_yaw
        #     if delta_yaw > np.pi:
        #         delta_yaw -= 2 * np.pi
        #     elif delta_yaw < -np.pi:
        #         delta_yaw += 2 * np.pi

        # yaw = self.last_yaw + delta_yaw

        # yaw = alpha * yaw + (1 - alpha) * self.last_yaw
        # self.last_yaw = yaw
        # 转换为 [0, 2π] 范围
        # if yaw < 0:
        #     yaw += 2 * math.pi
        return yaw

    @classmethod
    def register(cls, name: str):
        """Register a robot class with its name_prefix(decorator).

        Args:
            name(str): name_prefix of the robot class.
        """

        def decorator(robot_class):
            cls.robots[name] = robot_class

            @wraps(robot_class)
            def wrapped_function(*args, **kwargs):
                return robot_class(*args, **kwargs)

            return wrapped_function

        return decorator

    def set_up_to_scene(self, scene: Scene):
        """Set up robot in the scene.

        Args:
            scene (Scene): scene to set up.
        """
        # self._scene = scene
        robot_cfg = self.config
        if self.robot_entity:
            scene.add(self.robot_entity)
            # log.debug('self.robot_entity: ' + str(self.robot_entity))
        for rigid_body in self.get_rigid_bodies():
            scene.add(rigid_body)
        return

    def step(self, action: np.ndarray) -> Tuple[np.ndarray]:
        """

        Args:
            action: 使用np.ndarray存储的机器人速度

        Returns:
            obs: 机器人的观测量  但是似乎没有必要这样设计?
            info:
        """
        raise NotImplementedError

    def explore_zone(self, zone_corners: list = None, scane_direction: str = "horizontal", reset_flag: bool = False):
        """
        用户输入一个方形区域的四个角落点, 需要根据感知范围来探索这个区域, 感知范围是一个扇形的区域, 假设视野为120, 半径为2m,
        输入一个[[1,1], [1,10], [10,10], [10,1]]的方形区域, 该如何规划路径?

        """
        # 1. 计算区域的边界
        min_x = min(corner[0] for corner in zone_corners)
        max_x = max(corner[0] for corner in zone_corners)
        min_y = min(corner[1] for corner in zone_corners)
        max_y = max(corner[1] for corner in zone_corners)

        # 2. 确定扫描线的方向 (这里选择水平扫描)
        scan_direction = scane_direction  # 可以选择 "horizontal" 或 "vertical"

        # 3. 计算扫描线之间的距离，保证覆盖整个区域
        #    使用视野半径和视野角度来计算有效覆盖宽度
        import math
        effective_width = 2 * self.view_radius * math.sin(self.view_angle / 2)
        scan_line_spacing = effective_width * 0.8  # 稍微重叠，确保覆盖

        # 4. 生成扫描线
        scan_lines = []
        if scan_direction == "horizontal":
            y = min_y
        while y <= max_y:
            scan_lines.append(y)
            y += scan_line_spacing
        else:  # vertical
            x = min_x
            while x <= max_x:
                scan_lines.append(x)
                x += scan_line_spacing

        # 5. 生成路径点
        path_points = []
        if scan_direction == "horizontal":
            for i, y in enumerate(scan_lines):
                if i % 2 == 0:  # 偶数行，从左到右
                    path_points.append([min_x, y])
                    path_points.append([max_x, y])
                else:  # 奇数行，从右到左
                    path_points.append([max_x, y])
                    path_points.append([min_x, y])
        else:  # vertical
            for i, x in enumerate(scan_lines):
                if i % 2 == 0:  # 偶数列，从下到上
                    path_points.append([x, min_y])
                    path_points.append([x, max_y])
                else:  # 奇数列，从上到下
                    path_points.append([x, max_y])
                    path_points.append([x, min_y])

        return path_points


if __name__ == "__main__":
    config = {
        'name_prefix': 'jetbot3',
        'prim_path': '/World/Fancy_Robot3',

        # 'wheel_dof_names': ["left_wheel_joint", "right_wheel_joint"],
    }
    # create_robot=True,
    # usd_path=jet_robot_asset_path,
    # position=[-2, 0, 0],
    # }
    # type: str
    # prim_path: str
    # create_robot: bool = True
    # usd_path: Optional[str] = None  # If Optional, use default usd_path

    # common config
    position: Optional[Tuple[float, float, float]] = (0.0, 0.0, 0.0)
    orientation: Optional[Tuple[float, float, float, float]] = None
    scale: Optional[Tuple[float, float, float]] = None

    jetbot_config = RobotCfg()
