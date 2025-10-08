import torch

from controller.controller_pid import ControllerPID
from controller.controller_pid_jetbot import ControllerJetbot
from robot.sensor.camera import CfgCamera, CfgCameraThird
from map.map_grid_map import GridMap
from path_planning.path_planning_astar import AStar
from robot.robot import Robot
from robot.robot_trajectory import Trajectory
from robot.cfg import CfgJetbot
from robot.body.body_jetbot import BodyJetbot
from utils import to_torch


from isaacsim.core.api.scenes import Scene
from isaacsim.core.utils.types import ArticulationActions

from gsi2isaacsim.gsi_msgs_helper import (
    Plan,
    RobotFeedback,
    SkillInfo,
    Parameter,
    VelTwistPose,
)


class RobotJetbot(Robot):
    def __init__(
        self,
        cfg_robot: CfgJetbot,
        cfg_camera: CfgCamera = None,
        cfg_camera_third_person: CfgCameraThird = None,
        scene: Scene = None,
        map_grid: GridMap = None,
        scene_manager=None,
    ) -> None:
        super().__init__(
            cfg_robot,
            cfg_camera,
            cfg_camera_third_person,
            scene=scene,
            map_grid=map_grid,
            scene_manager=scene_manager,
        )
        self.body = BodyJetbot(cfg_robot=self.cfg_robot, scene=scene)
        self.controller = ControllerJetbot()
        self.control_mode = "joint_velocities"
        # # self.scene.add(self.robot)  # 需要再考虑下, scene加入robot要放在哪一个class中, 可能放在scene好一些
        self.pid_distance = ControllerPID(1, 0.1, 0.01, target=0)
        self.pid_angle = ControllerPID(10, 0, 0.1, target=0)

        self.counter = 0
        self.pub_period = 50
        self.previous_pos = None
        self.movement_threshold = (
            0.1  # 移动时，如果两次检测之间的移动距离小于这个阈值，那么就会判定其为异常
        )

        # self.node = node
        #
        # self.node.register_feedback_publisher(
        #     robot_class=self.cfg_robot.type,
        #     robot_id=self.cfg_robot.id,
        #     qos=50
        # )
        # self.node.register_motion_publisher(
        #     robot_class=self.cfg_robot.type,
        #     robot_id=self.cfg_robot.id,
        #     qos=50
        # )

        # self.init_ros2()

    def initialize(self) -> None:
        super().initialize()
        return

    def init_ros2(self):
        import omni.graph.core as og

        og.Controller.edit(
            {
                "graph_path": f"/ActionGraph/{self.cfg_robot.type}_{self.cfg_robot.id}",
                "evaluator_name": "execution",
            },
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                    (
                        "SubscribeJointState",
                        "isaacsim.ros2.bridge.ROS2SubscribeJointState",
                    ),
                    (
                        "ArticulationController",
                        "isaacsim.core.nodes.IsaacArticulationController",
                    ),
                    ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
                    (
                        "OnPlaybackTick.outputs:tick",
                        "SubscribeJointState.inputs:execIn",
                    ),
                    (
                        "OnPlaybackTick.outputs:tick",
                        "ArticulationController.inputs:execIn",
                    ),
                    (
                        "ReadSimTime.outputs:simulationTime",
                        "PublishJointState.inputs:timeStamp",
                    ),
                    (
                        "SubscribeJointState.outputs:jointNames",
                        "ArticulationController.inputs:jointNames",
                    ),
                    (
                        "SubscribeJointState.outputs:positionCommand",
                        "ArticulationController.inputs:positionCommand",
                    ),
                    (
                        "SubscribeJointState.outputs:velocityCommand",
                        "ArticulationController.inputs:velocityCommand",
                    ),
                    (
                        "SubscribeJointState.outputs:effortCommand",
                        "ArticulationController.inputs:effortCommand",
                    ),
                ],
                og.Controller.Keys.SET_VALUES: [
                    # Providing path to /panda robot to Articulation Controller node
                    # Providing the robot path is equivalent to setting the targetPrim in Articulation Controller node
                    # ("ArticulationController.inputs:usePath", True),      # if you are using an older version of Isaac Sim, you may need to uncomment this line
                    (
                        "PublishJointState.inputs:topicName",
                        f"joint_states_{self.cfg_robot.type}_{self.cfg_robot.id}",
                    ),
                    (
                        "ArticulationController.inputs:robotPath",
                        f"{self.cfg_robot.path_prim_swarm}",
                    ),
                    (
                        "PublishJointState.inputs:targetPrim",
                        f"{self.cfg_robot.path_prim_swarm}",
                    ),
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
        pos, quat = self.get_world_poses()  # type np.array
        target_pos = to_torch(target_pos, device=pos.device)

        if self.counter % self.pub_period == 0:
            self._publish_feedback(
                params=self._params_from_pose(pos, quat),
                progress=self._calc_dist(pos, self.nav_end) * 100 / self.nav_dist,
            )

        # 获取2D方向的朝向，逆时针是正
        yaw = self.quaternion_to_yaw(quat)
        # 获取机器人和目标连线的XY平面上的偏移角度
        robot_to_target_angle = np.arctan2(
            target_pos[1] - pos[1], target_pos[0] - pos[0]
        )
        # 差速, 和偏移角度成正比，通过pi归一化
        delta_angle = robot_to_target_angle - yaw
        if abs(delta_angle) < 0.017:  # 角度控制死区
            delta_angle = 0
        elif (
            delta_angle < -np.pi
        ):  # 当差距abs超过pi后, 就代表从这个方向转弯不好, 要从另一个方向转弯
            delta_angle += 2 * np.pi
        elif delta_angle > np.pi:
            delta_angle -= 2 * np.pi
        if torch.linalg.norm(target_pos[0:2] - pos[0:2]) < 0.1:
            self.action = [0, 0]
            self._publish_feedback(
                params=self._params_from_pose(pos, quat), progress=100
            )
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

    def on_physics_step(self, step_size):
        super().on_physics_step(step_size)

        # self._publish_status_pose()
        self.counter += 1

        if self.flag_world_reset:
            if self.flag_action_navigation:
                self.move_along_path()  # 每一次都计算下速度
                self.step(self.action)
                # if self.counter % self.pub_period == 0:
                #     self._publish_feedback_pose()

        return

    def step(self, action):
        action = to_torch(action)
        if self.control_mode == "joint_position":
            action = ArticulationActions(joint_positions=action)
        elif self.control_mode == "joint_velocities":
            action = ArticulationActions(joint_velocities=action)
        elif self.control_mode == "joint_efforts":
            action = ArticulationActions(joint_efforts=action)
        else:
            raise NotImplementedError
        self.robot_entity.apply_action(action)

        # obs暂时未实现
        obs = None
        return obs


if __name__ == "__main__":
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

    plt.plot(x_coords, y_coords, marker="o", linestyle="-", color="blue")

    # 绘制区域边界
    zone_x = [corner[0] for corner in zone_corners] + [zone_corners[0][0]]
    zone_y = [corner[1] for corner in zone_corners] + [zone_corners[0][1]]
    plt.plot(zone_x, zone_y, color="red", linestyle="--")

    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title("探索路径")
    plt.grid(True)
    plt.show()
