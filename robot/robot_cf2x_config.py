from typing import Optional

from robot.robot_base import RobotBase
from robot.robot_cfg import RobotCfg
from controller.controller_pid import PIDController


import numpy as np
from isaacsim.core.api.controllers import BaseController
from isaacsim.core.api.scenes import Scene
from isaacsim.core.utils.types import ArticulationAction

class Cf2xController(BaseController):
    def __init__(self):
        super().__init__(name="my_cool_controller")
        # An open loop controller that uses a unicycle model
        # self._wheel_radius = 0.03
        # self._wheel_base = 0.1125
        return

    def forward(self, command):
        # command will have two elements, first element is the forward velocity
        # second element is the angular velocity (yaw only).
        joint_velocities = [0.0, 0.0]
        joint_velocities[0] = ((2 * command[0]) - (command[1] * self._wheel_base)) / (2 * self._wheel_radius)
        joint_velocities[1] = ((2 * command[0]) + (command[1] * self._wheel_base)) / (2 * self._wheel_radius)
        # A controller has to return an ArticulationAction
        return ArticulationAction(joint_velocities=joint_velocities)

    def velocity(self, command):
        return ArticulationAction(joint_velocities=command)


from isaacsim.core.utils.nucleus import get_assets_root_path

assets_root_path = get_assets_root_path()
if assets_root_path is None:
    print("Could not find nucleus server with /Isaac folder")


class JetbotRobotCfg(RobotCfg):
    # meta info
    name: Optional[str] = 'jetbot'
    type: Optional[str] = 'JetbotRobot'
    prim_path: Optional[str] = '/World/jetbot'
    create_robot: Optional[bool] = True

    usd_path: Optional[str] = assets_root_path + "/Isaac/Robots/Jetbot/jetbot.usd"


class Jetbot(RobotBase):
    def __init__(self, config: JetbotRobotCfg, scene: Scene):
        super().__init__(config, scene)
        self.robot = WheeledRobot(
            prim_path=config.prim_path,
            name=config.name,
            wheel_dof_names=['left_wheel_joint', 'right_wheel_joint'],
            create_robot=True,
            position=np.array(config.position),
            orientation=np.array(config.orientation),
            usd_path=config.usd_path,
        )
        self.scale = config.scale
        self.controller = JetbotController()
        self.scene.add(self.robot)
        self.pid_distance = PIDController(1, 0.1, 0.01, target=0)
        self.pid_angle = PIDController(10, 0, 0.1, target=0)
        self.last_yaw = 0
        return

    def apply_action(self, action):
        self.robot.apply_action(self.controller.velocity(action))
        return

    def quaternion_to_yaw(self, orientation):
        import math
        alpha = 0.1
        norm = math.sqrt(sum(comp ** 2 for comp in orientation))
        if abs(norm-1) > 0.1 :
            print("没有归一")
        qw, qx, qy, qz = orientation
        # 计算方位角（绕 z 轴的旋转角度）
        sinr_cosp = 2.0 * (qw * qz + qx * qy)
        cosr_cosp = 1.0 - 2.0 * (qy ** 2 + qz ** 2)
        # 转换为 [-π, π] 范围，逆时针为正
        yaw = math.atan2(sinr_cosp, cosr_cosp)

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

    def move_to(self, target_postion):
        import numpy as np
        """
        让2轮差速小车在一个2D平面上运动到目标点
        缺点：不适合3D，无法避障，地面要是平的
        速度有两个分两，自转的分量 + 前进的分量
        """
        car_position, car_orientation = self.robot.get_world_pose()  ## type np.array
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
        # k1 = 1 / np.pi
        # v_rotation = k1 * delta_angle
        v_rotation = self.pid_angle.compute(delta_angle, dt=1/60)
        # 前进速度，和距离成正比
        k2 = 1
        v_forward = 4# k2 * np.linalg.norm(target_postion[0:2] - car_position[0:2])
        # v_forward = 0
        if np.linalg.norm(target_postion[0:2] - car_position[0:2]) < 0.01:
            v_forward = 0
        # 合速度
        v_left = v_forward + v_rotation
        v_right = v_forward - v_rotation
        v_max = 5
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
        return
