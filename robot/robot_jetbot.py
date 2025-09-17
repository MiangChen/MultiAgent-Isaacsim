import numpy as np

from controller.controller_pid import ControllerPID
from controller.controller_pid_jetbot import ControllerJetbot
from camera.camera_cfg import CameraCfg
from camera.camera_third_person_cfg import CameraThirdPersonCfg
from map.map_grid_map import GridMap
from path_planning.path_planning_astar import AStar
from robot.robot_base import RobotBase
from robot.robot_trajectory import Trajectory
from ros.ros_node import SwarmNode
from robot.robot_cfg_jetbot import RobotCfgJetbot

import carb
from isaacsim.core.api.scenes import Scene
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.prims import define_prim, get_prim_at_path
from isaacsim.core.utils.types import ArticulationActions

from gsi2isaacsim.gsi_msgs_helper import Plan, RobotFeedback, SkillInfo, Parameter, VelTwistPose


class RobotJetbot(RobotBase):
    def __init__(self, cfg_body: RobotCfgJetbot, cfg_camera: CameraCfg = None,
                 cfg_camera_third_person: CameraThirdPersonCfg = None, scene: Scene = None,
                 map_grid: GridMap = None, node: SwarmNode = None, scene_manager=None) -> None:
        super().__init__(cfg_body, cfg_camera, cfg_camera_third_person, scene, map_grid, node=node,
                         scene_manager=scene_manager)

        self.controller = ControllerJetbot()
        self.control_mode = 'joint_velocities'
        # # self.scene.add(self.robot)  # 需要再考虑下, scene加入robot要放在哪一个class中, 可能放在scene好一些
        self.pid_distance = ControllerPID(1, 0.1, 0.01, target=0)
        self.pid_angle = ControllerPID(10, 0, 0.1, target=0)

        self.counter = 0
        self.pub_period = 50
        self.previous_pos = None
        self.movement_threshold = 0.1  # 移动时，如果两次检测之间的移动距离小于这个阈值，那么就会判定其为异常

        self.node = node

        self.node.register_feedback_publisher(
            robot_class=self.cfg_body.name_prefix,
            robot_id=self.cfg_body.id,
            qos=50
        )

        self.init_ros2()

    def initialize(self) -> None:
        super().initialize()
        return

    def init_ros2(self):
        import omni.graph.core as og

        og.Controller.edit(
            {"graph_path": f"/ActionGraph/{self.cfg_body.name_prefix}_{self.cfg_body.id}",
             "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                    ("SubscribeJointState", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                    ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
                    ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),

                    ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),

                    ("SubscribeJointState.outputs:jointNames", "ArticulationController.inputs:jointNames"),
                    ("SubscribeJointState.outputs:positionCommand", "ArticulationController.inputs:positionCommand"),
                    ("SubscribeJointState.outputs:velocityCommand", "ArticulationController.inputs:velocityCommand"),
                    ("SubscribeJointState.outputs:effortCommand", "ArticulationController.inputs:effortCommand"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    # Providing path to /panda robot to Articulation Controller node
                    # Providing the robot path is equivalent to setting the targetPrim in Articulation Controller node
                    # ("ArticulationController.inputs:usePath", True),      # if you are using an older version of Isaac Sim, you may need to uncomment this line
                    ("PublishJointState.inputs:topicName",
                     f"joint_states_{self.cfg_body.name_prefix}_{self.cfg_body.id}"),
                    ("ArticulationController.inputs:robotPath", f"{self.cfg_body.prim_path}"),
                    ("PublishJointState.inputs:targetPrim", f"{self.cfg_body.prim_path}")
                ],
            },
        )

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
        # 前进速度：更大常数
        v_forward = 60.0  # 原来 30

        # 合速度
        v_left = v_forward + v_rotation
        v_right = v_forward - v_rotation

        v_max = 80.0  # 原来 40

        # 限幅
        v_left = np.clip(v_left, -v_max, v_max)
        v_right = np.clip(v_right, -v_max, v_max)
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

    def _params_from_pose(self, pos: np.ndarray, quat: np.ndarray) -> list[Parameter]:
        # 保证是 float -> str
        px, py, pz = float(pos[0]), float(pos[1]), float(pos[2])
        qx, qy, qz, qw = float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])

        base_return = [
            Parameter(key="pos_x", value=str(px)),
            Parameter(key="pos_y", value=str(py)),
            Parameter(key="pos_z", value=str(pz)),
            Parameter(key="quat_x", value=str(qx)),
            Parameter(key="quat_y", value=str(qy)),
            Parameter(key="quat_z", value=str(qz)),
            Parameter(key="quat_w", value=str(qw)),
        ]

        normal_return = [Parameter(key="status", value="normal"), *base_return]
        abnormal_return = [Parameter(key="status", value="abnormal"), *base_return]

        if self.previous_pos:
            if np.sqrt((self.previous_pos[0] - px) ** 2 + (self.previous_pos[1] - py) ** 2 + (
                    self.previous_pos[2] - pz) ** 2) < self.movement_threshold:
                self.previous_pos = [px, py, pz]
                return abnormal_return
            else:
                self.previous_pos = [px, py, pz]
                return normal_return

        else:
            self.previous_pos = [px, py, pz]
            return normal_return

    def _publish_feedback_pose(self):
        pos, quat = self.get_world_poses()

        skill = SkillInfo(
            skill=self.current_task_name,
            params=self._params_from_pose(pos, quat),
            object_id="",
            task_id=self.current_task_id,
        )

        msg = RobotFeedback(
            robot_id=f"{self.cfg_body.name_prefix}_{self.cfg_body.id}",
            skill_feedback=skill,
        )

        self.node.publish_navigation_feedback(self.cfg_body.name_prefix, self.cfg_body.id, msg)

    def _publish_status_pose(self):

        if not getattr(self, 'ros2_initialized', False):
            return

        import numpy as np

        positions, orientations = self.robot_entity.get_world_poses()
        pos = positions[0]
        orn = orientations[0]

        #    注：这些 API 返回 (N, 3) 的数组/张量；这里只取第 0 个
        lin_v = self.robot_entity.get_linear_velocities(indices=[0], clone=True)
        ang_v = self.robot_entity.get_angular_velocities(indices=[0], clone=True)

        # 统一成 numpy，做个健壮性兜底
        lin_v0 = np.asarray(lin_v)[0] if np.size(lin_v) else np.zeros(3, dtype=float)
        ang_v0 = np.asarray(ang_v)[0] if np.size(ang_v) else np.zeros(3, dtype=float)

        msg = VelTwistPose()

        # vel 字段：保持与你原逻辑一致，全部置 0
        msg.vel.x = 0.0
        msg.vel.y = 0.0
        msg.vel.z = 0.0

        # twist：使用仿真里的实时速度
        msg.twist.linear.x = float(lin_v0[0])
        msg.twist.linear.y = float(lin_v0[1])
        msg.twist.linear.z = float(lin_v0[2])

        msg.twist.angular.x = float(ang_v0[0])
        msg.twist.angular.y = float(ang_v0[1])
        msg.twist.angular.z = float(ang_v0[2])

        # pose：使用仿真里的实时位置与朝向
        msg.pose.position.x = float(pos[0])
        msg.pose.position.y = float(pos[1])
        msg.pose.position.z = float(pos[2])

        msg.pose.orientation.x = float(orn.x)
        msg.pose.orientation.y = float(orn.y)
        msg.pose.orientation.z = float(orn.z)
        msg.pose.orientation.w = float(orn.w)

        self.node.publish_motion(
            robot_class=self.cfg_body.name_prefix,
            robot_id=self.cfg_body.id,
            msg=msg
        )

    def on_physics_step(self, step_size):
        super().on_physics_step(step_size)

        self._publish_status_pose()
        self.counter += 1

        if self.flag_world_reset:
            if self.flag_action_navigation:
                self.move_along_path()  # 每一次都计算下速度
                self.step(self.action)
                # if self.counter % self.pub_period == 0:
                #     self._publish_feedback_pose()

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
