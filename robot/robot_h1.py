from typing import Dict

import numpy as np

from controller.controller_pid import ControllerPID
from log.log_manager import LogManager
from map.map_grid_map import GridMap
from robot.body.body_h1 import BodyH1
from robot.body import BodyRobot
from robot.cfg import CfgH1
from robot.robot import Robot
from robot.robot_trajectory import Trajectory
from robot.sensor.camera import CfgCamera, CfgCameraThird
from controller.controller_policy_h1 import H1FlatTerrainPolicy
from utils import to_torch, quaternion_to_yaw

from isaacsim.core.api.scenes import Scene
from isaacsim.core.utils.types import ArticulationActions

from gsi2isaacsim.gsi_msgs_helper import (
    Plan,
    RobotFeedback,
    SkillInfo,
    Parameter,
    VelTwistPose,
)

logger = LogManager.get_logger(__name__)


class RobotH1(Robot):
    """
    H1 Robot Class.
    This class uses an asynchronous factory pattern for initialization
    because its policy controller requires asynchronous loading.
    Please use `await RobotH1.create(...)` to instantiate.
    """

    def __init__(
        self,
        cfg_robot: Dict,
        map_grid: GridMap = None,
        scene: Scene = None,
        scene_manager=None,
    ) -> None:
        self.cfg_robot = CfgH1(**cfg_robot)
        super().__init__(
            scene,
            scene_manager,
            map_grid,
        )
        # self.create_robot_entity()
        self.control_mode = "joint_positions"
        self.scene_manager = scene_manager
        # 初始化PID控制器等同步组件
        self.pid_distance = ControllerPID(1, 0.1, 0.01, target=0)
        self.pid_angle = ControllerPID(10, 0, 0.1, target=0)

        # 将控制器先初始化为 None，它将在异步工厂中被正确创建
        self.controller_policy: H1FlatTerrainPolicy | None = None
        self.base_command = np.zeros(3)

        self.counter = 0
        self.pub_period = 50
        self.previous_pos = None
        self.movement_threshold = (
            0.1  # 移动时，如果两次检测之间的移动距离小于这个阈值，那么就会判定其为异常
        )
        self.body = BodyH1(cfg_robot=self.cfg_robot, scene=self.scene)

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

    @classmethod
    async def create(
        cls,
        cfg_robot: CfgH1,
        # cfg_camera: CfgCamera = None,
        # cfg_camera_third_person: CfgCameraThird = None,
        map_grid: GridMap = None,
        scene: Scene = None,
        scene_manager=None,
    ) -> "RobotH1":
        """
        Asynchronously creates and fully initializes a RobotH1 instance,
        including its asynchronous policy controller.
        """
        instance = cls(
            cfg_robot=cfg_robot,
            # cfg_camera=cfg_camera,
            # cfg_camera_third_person=cfg_camera_third_person,
            map_grid=map_grid,
            scene=scene,
            scene_manager=scene_manager,
        )

        # 异步地创建并加载 H1FlatTerrainPolicy 控制器
        instance.controller_policy = await H1FlatTerrainPolicy.create(
            prim_path=instance.cfg_robot.path_prim_swarm
        )
        return instance

    def initialize(self):
        """
        Initializes the robot's connection to the simulation world entities.
        This should be called after the world is ready (e.g., after a reset).
        """
        super().initialize()
        if self.controller_policy:
            self.controller_policy.initialize(self.body.robot_articulation)
        else:
            logger.error(f"[Warning] RobotH1 '{self.name}' has no controller_policy")

    def move_to(self, target_pos):
        """
        让2轮差速小车在一个2D平面上运动到目标点
        缺点：不适合3D，无法避障，地面要是平的
        速度有两个分两，自转的分量 + 前进的分量
        """
        pos, quat = self.body.get_world_poses()
        # if self.counter % self.pub_period == 0:
        #     self._publish_feedback(
        #         params=self._params_from_pose(pos, quat),
        #         progress=self._calc_dist(pos, self.nav_end) * 100 / self.nav_dist,
        #     )
        # 获取2D方向的朝向，逆时针是正
        yaw = quaternion_to_yaw(quaternion=quat)

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
        if np.linalg.norm(target_pos[0:2] - pos[0:2]) < 1:
            self.action = [0, 0]
            self._publish_feedback(
                params=self._params_from_pose(pos, quat), progress=100
            )
            return True  # 已经到达目标点附近10cm, 停止运动
        # logger.info(f"delta_angle, np.linalg.norm(target_pos[0:2] - pos[0:2])")
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
        action = to_torch(action)
        if self.control_mode == "joint_positions":
            action = ArticulationActions(joint_positions=action)
        elif self.control_mode == "joint_velocities":
            action = ArticulationActions(joint_velocities=action)
        elif self.control_mode == "joint_efforts":
            action = ArticulationActions(joint_efforts=action)
        else:
            raise NotImplementedError
        self.body.robot_articulation.apply_action(action)

        # obs暂时未实现
        obs = None
        return obs

    def on_physics_step(self, step_size):
        super().on_physics_step(step_size)

        self.counter += 1
        self._publish_status_pose()

        if self.flag_world_reset == True:
            if self.flag_action_navigation == True:
                self.move_along_path()  # 每一次都计算下速度
                self.action = self.controller_policy.forward(
                    step_size, self.base_command, self.body.robot_articulation
                )
            else:
                self.action = self.controller_policy.forward(
                    step_size, [0, 0, 0], self.body.robot_articulation
                )
            self.step(self.action)
        return
