from typing import List


class ControllerHeadTo:
    def __init__(self,
                 position_robot: List[int] = None,
                 orientation: List[int] = None,
                 position_target: List[int] = None,
                 )


    def on_physics_step(self, step_size):
        self.apply_action(action=[5, 5])



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
