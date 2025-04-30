from typing import Optional
import numpy as np
from isaacsim.core.api.scenes import Scene
from isaacsim.robot.wheeled_robots.robots import WheeledRobot

from map.map_grid_map import GridMap
from controller.controller_pid import ControllerPID
from robot.robot_base import RobotBase
from robot.robot_trajectory import Trajectory
from robot.robot_cfg_h1 import RobotCfgH1
from controller.controller_policy_h1 import H1FlatTerrainPolicy

import carb
import numpy as np

from isaacsim.core.utils.prims import define_prim, get_prim_at_path
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.types import ArticulationActions


class RobotH1(RobotBase):
    def __init__(self, config: RobotCfgH1, scene: Scene = None, map_grid: GridMap = None) -> None:
        super().__init__(config, scene, map_grid)
        self.prim_path = config.prim_path + f'/{config.name_prefix}_{config.id}'

        prim = get_prim_at_path(self.prim_path)
        if not prim.IsValid():
            prim = define_prim(self.prim_path, "Xform")

            if config.usd_path:
                prim.GetReferences().AddReference(config.usd_path)  # 加载机器人USD模型
            else:
                carb.log_error("unable to add robot usd, usd_path not provided")
        # 初始化机器人关节树
        self.robot_entity = Articulation(
            prim_paths_expr=self.prim_path,
            name=config.name_prefix + f'_{config.id}',
            positions=np.array([config.position]),
            orientations=np.array([config.orientation]),
        )

        # self.config = config
        self.flag_active = False
        self.robot_prim = config.prim_path + f'/{config.name_prefix}_{config.id}'
        self.control_mode = 'joint_positions'
        # self.scale = config.scale  # 已经在config中有的, 就不要再拿别的量来存储了, 只存储一次config就可以, 所以注释掉了
        # from controller.controller_pid_jetbot import ControllerJetbot
        # self.controller = ControllerJetbot()
        # self.scene.add(self.robot)  # 需要再考虑下, scene加入robot要放在哪一个class中, 可能放在scene好一些
        self.pid_distance = ControllerPID(1, 0.1, 0.01, target=0)
        self.pid_angle = ControllerPID(10, 0, 0.1, target=0)

        # self.traj = Trajectory(
        #     robot_prim_path=self.robot_prim,
        #     # name='traj' + f'_{config.id}',
        #     id=config.id,
        #     max_points=100,
        #     color=(0.3, 1.0, 0.3),
        #     scene=self.scene,
        #     radius=0.05,
        # )

        # self.last_yaw = 0

        self.view_angle = 2 * np.pi / 3  # 感知视野 弧度
        self.view_radius = 2  # 感知半径 米

        # 神经网络控制器
        # prim_path = "/World/h1"
        self.controller_policy = H1FlatTerrainPolicy(prim_path=self.prim_path)
        self.base_command = np.zeros(3)
        return

    def initialize(self):
        self.controller_policy.initialize(self.robot_entity)  # 初始化配置

    def step(self, action):
        if self.control_mode == 'joint_position':
            action = ArticulationActions(joint_positions=action)
        elif self.control_mode == 'joint_velocities':
            action = ArticulationActions(joint_velocities=action)
        elif self.control_mode == 'joint_efforts':
            action = ArticulationActions(joint_efforts=action)
        else:
            raise NotImplementedError
        self.robot_entity.apply_action(action)

        # obs暂时未实现
        obs = None
        return obs

    def on_physics_step(self, step_size):
        if self.flag_world_reset == True:
            if self.flag_action_navigation == True:
                self.move_along_path()  # 每一次都计算下速度
                self.action = self.controller_policy.forward(step_size, self.base_command, self.robot_entity)
                self.step(self.action)
        return

    def move_to(self, target_postion):
        import numpy as np
        """
        让2轮差速小车在一个2D平面上运动到目标点
        缺点：不适合3D，无法避障，地面要是平的
        速度有两个分两，自转的分量 + 前进的分量
        """
        car_position, car_orientation = self.robot_entity.get_world_pose()  # self.get_world_pose()  ## type np.array
        # print(car_orientation)
        # print(type(car_orientation))
        # car_position, car_orientation = self.robot.get_world_pose()  ## type np.array
        # 获取2D方向的小车朝向，逆时针是正
        car_yaw_angle = self.quaternion_to_yaw(car_orientation)

        # 获取机器人和目标连线的XY平面上的偏移角度
        car_to_target_angle = np.arctan2(target_postion[1] - car_position[1], target_postion[0] - car_position[0])
        # 差速, 和偏移角度成正比，通过pi归一化
        delta_angle = car_to_target_angle - car_yaw_angle
        # if abs(car_to_target_angle - car_yaw_angle) > np.pi * 11/10:  # 超过pi，那么用另一个旋转方向更好， 归一化到 -pi ～ pi区间, 以及一个滞回，防止振荡
        #     delta_angle = delta_angle - 2 * np.pi
        if abs(delta_angle) < 0.017:  # 角度控制死区
            delta_angle = 0
        elif delta_angle < -np.pi:  # 当差距abs超过pi后, 就代表从这个方向转弯不好, 要从另一个方向转弯
            delta_angle += 2 * np.pi
        elif delta_angle > np.pi:
            delta_angle -= 2 * np.pi
        if np.linalg.norm(target_postion[0:2] - car_position[0:2]) < 0.1:
            v_forward = 0
            self.apply_action([0, 0])
            return True  # 已经到达目标点附近10cm, 停止运动

        # k1 = 1 / np.pi
        # v_rotation = k1 * delta_angle
        v_rotation = self.pid_angle.compute(delta_angle, dt=1 / 60)
        # 前进速度，和距离成正比
        k2 = 1
        v_forward = 15  # k2 * np.linalg.norm(target_postion[0:2] - car_position[0:2])

        # 合速度
        v_left = v_forward + v_rotation
        v_right = v_forward - v_rotation
        v_max = 20
        if v_left > v_max:
            v_left = v_max
        elif v_left < -v_max:
            v_left = -v_max
        if v_right > v_max:
            v_right = v_max
        elif v_right < -v_max:
            v_right = -v_max
        self.apply_action([v_left, v_right])
        # print(v_left, v_right)
        # print("v rotation", v_rotation, "v forward", v_forward)
        # print("yaw", car_yaw_angle, "target yaw", car_to_target_angle,"\tdelta angle", delta_angle, "\tdistance ", np.linalg.norm(target_postion[0:2] - car_position[0:2]))
        return False  # 还没有到达

    def move_along_path(self, path: list = None, flag_reset: bool = False):
        """
        让机器人沿着一个list的路径点运动
        需求: 在while外面 能够记录已经到达的点, 每次到达某个目标点的 10cm附近,就认为到了, 然后准备下一个点

        """
        if flag_reset == True:
            self.path_index = 0
            self.path = path

        # car_position, car_orientation = self.robot.get_world_pose()  ## type np.array
        # # 获取2D方向的小车朝向，逆时针是正
        # if np.linalg.norm(self.path[self.path_index][0:2] - car_position[0:2]) < 0.1:
        #     self.path_index = self.path_index + 1
        if self.path_index < len(self.path):  # 当index==len的时候, 就已经到达目标了
            reach_flat = self.move_to(self.path[self.path_index])
            print(self.path[self.path_index])
            if reach_flat == True:
                self.path_index += 1
            return False
        else:
            return True

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


if __name__ == '__main__':
    h1 = RobotH1()
