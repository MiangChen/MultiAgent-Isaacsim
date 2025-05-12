from isaacsim.core.api.scenes import Scene

from map.map_grid_map import GridMap
from controller.controller_pid import ControllerPID
from robot.robot_base import RobotBase
from robot.robot_trajectory import Trajectory
from robot.robot_cfg_h1 import RobotCfgH1
from controller.controller_policy_h1 import H1FlatTerrainPolicy

import numpy as np

import carb
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.prims import define_prim, get_prim_at_path
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

        self.config = config
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
        self.view_angle = 2 * np.pi / 3  # 感知视野 弧度
        self.view_radius = 2  # 感知半径 米

        # 神经网络控制器
        # prim_path = "/World/h1"
        self.controller_policy = H1FlatTerrainPolicy(prim_path=self.prim_path)
        self.base_command = np.zeros(3)
        return

    def initialize(self):
        self.controller_policy.initialize(self.robot_entity)  # 初始化配置

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
        if np.linalg.norm(target_pos[0:2] - pos[0:2]) < 1:
            self.action = [0, 0]
            return True  # 已经到达目标点附近10cm, 停止运动
        # print(delta_angle, np.linalg.norm(target_pos[0:2] - pos[0:2]))
        k_rotate = 1 / np.pi
        v_rotation = self.pid_angle.compute(delta_angle, dt=1 / 200)
        if v_rotation > 1:
            v_rotation = 1
        elif v_rotation < -1:
            v_rotation = -1
        # 前进速度，和距离成正比
        k_forward = 1
        v_forward = 1
        self.base_command = [v_forward, 0, -1 * v_rotation]
        return False  # 还没有到达

    def step(self, action):
        if self.control_mode == 'joint_positions':
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
            else:
                self.action = self.controller_policy.forward(step_size, [0, 0, 0], self.robot_entity)
            self.step(self.action)
        return


if __name__ == '__main__':
    h1 = RobotH1()
