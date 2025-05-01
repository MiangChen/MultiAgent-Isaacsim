import numpy as np

from controller.controller_pid import ControllerPID
from controller.controller_pid_jetbot import ControllerJetbot
from map.map_grid_map import GridMap
from path_planning.path_planning_astar import AStar
from robot.robot_base import RobotBase
from robot.robot_trajectory import Trajectory
from robot.robot_cfg_jetbot import RobotCfgJetbot

import carb
from isaacsim.core.api.scenes import Scene
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.prims import define_prim, get_prim_at_path
from isaacsim.core.utils.types import ArticulationActions


class RobotJetbot(RobotBase):
    def __init__(self, config: RobotCfgJetbot, scene: Scene, map_grid: GridMap):
        super().__init__(config, scene, map_grid)
        self.prim_path = config.prim_path + f'/{config.name_prefix}_{config.id}'
        prim = get_prim_at_path(self.prim_path)
        if not prim.IsValid():
            prim = define_prim(self.prim_path, "Xform")
            if config.usd_path:
                prim.GetReferences().AddReference(config.usd_path)  # 加载机器人USD模型
            else:
                carb.log_error("unable to add robot usd, usd_path not provided")
        self.robot_entity = Articulation(
            prim_paths_expr=self.prim_path,
            name=config.name_prefix + f'_{config.id}',
            positions=np.array([config.position]),
            orientations=np.array([config.orientation]),
        )
        self.flag_active = False

        self.controller = ControllerJetbot()
        self.control_mode = 'joint_velocities'
        # self.scene.add(self.robot)  # 需要再考虑下, scene加入robot要放在哪一个class中, 可能放在scene好一些
        self.pid_distance = ControllerPID(1, 0.1, 0.01, target=0)
        self.pid_angle = ControllerPID(10, 0, 0.1, target=0)

        self.traj = Trajectory(
            robot_prim_path=config.prim_path + f'/{config.name_prefix}_{config.id}',
            # name='traj' + f'_{config.id}',
            id=config.id,
            max_points=100,
            color=(0.3, 1.0, 0.3),
            scene=self.scene,
            radius=0.05,
        )
        return

    def initialize(self) -> None:
        return

    def move_to(self, target_pos):
        import numpy as np
        """
        让2轮差速小车在一个2D平面上运动到目标点
        缺点：不适合3D，无法避障，地面要是平的
        速度有两个分两，自转的分量 + 前进的分量
        """
        pos, quat = self.get_world_poses()  # self.get_world_pose()  ## type np.array
        # 获取2D方向的朝向，逆时针是正
        yaw = self.quaternion_to_yaw(quat)
        # 获取机器人和目标连线的XY平面上的偏移角度
        robot_to_target_angle = np.arctan2(target_pos[1] - pos[1], target_pos[0] - pos[0])
        # 差速, 和偏移角度成正比，通过pi归一化
        delta_angle = robot_to_target_angle - yaw
        if abs(delta_angle) < 0.017:  # 角度控制死区
            delta_angle = 0
        elif delta_angle < -np.pi:  # 当差距abs超过pi后, 就代表从这个方向转弯不好, 要从另一个方向转弯
            delta_angle += 2 * np.pi
        elif delta_angle > np.pi:
            delta_angle -= 2 * np.pi
        if np.linalg.norm(target_pos[0:2] - pos[0:2]) < 0.1:
            self.action = [0, 0]
            return True  # 已经到达目标点附近10cm, 停止运动

        k_rotate = 1 / np.pi
        v_rotation = self.pid_angle.compute(delta_angle, dt=1 / 60)
        # 前进速度，和距离成正比
        k_forward = 1
        v_forward = 15

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
        self.action = [v_left, v_right]
        return False  # 还没有到达

    def on_physics_step(self, step_size):
        if self.flag_world_reset == True:
            if self.flag_action_navigation == True:
                self.move_along_path()  # 每一次都计算下速度
                self.step(self.action)
        return

    def pick_up(self):
        pass
        return

    def put_down(self):
        pass
        return

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


if __name__ == '__main__':
    explorer = Explorer()
    zone_corners = [[1, 1], [1, 10], [10, 10], [10, 1]]
    path = explorer.explore_zone(zone_corners)

    print("生成的路径点:")
    for point in path:
        print(point)

    # 可视化路径 (需要 matplotlib)
    import matplotlib.pyplot as plt

    x_coords = [point[0] for point in path]
    y_coords = [point[1] for point in path]

    plt.plot(x_coords, y_coords, marker='o', linestyle='-', color='blue')

    # 绘制区域边界
    zone_x = [corner[0] for corner in zone_corners] + [zone_corners[0][0]]
    zone_y = [corner[1] for corner in zone_corners] + [zone_corners[0][1]]
    plt.plot(zone_x, zone_y, color='red', linestyle='--')

    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title("探索路径")
    plt.grid(True)
    plt.show()
