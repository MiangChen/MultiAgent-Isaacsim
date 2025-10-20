# =============================================================================
# Robot Jetbot Module - Jetbot Robot Implementation
# =============================================================================
#
# This module provides the Jetbot robot implementation with
# camera sensors, and navigation capabilities within the Isaac Sim environment.
#
# =============================================================================

# Standard library imports
from typing import Dict

# Third-party library imports
import torch

# Local project imports
from map.map_grid_map import GridMap
from physics_engine.isaacsim_utils import Scene, ArticulationActions
from robot.robot import Robot
from robot.robot_trajectory import Trajectory
from robot.cfg import CfgJetbot
from robot.body.body_jetbot import BodyJetbot
from robot.sensor.camera import CfgCamera, CfgCameraThird
from utils import to_torch, quat_to_yaw

# Custom ROS message imports
from gsi_msgs.gsi_msgs_helper import (
    Plan,
    RobotFeedback,
    SkillInfo,
    Parameter,
    VelTwistPose,
)


class RobotJetbot(Robot):
    def __init__(
        self,
        cfg_robot: Dict = {},
        # cfg_camera: CfgCamera = None,
        # cfg_camera_third_person: CfgCameraThird = None,
        scene: Scene = None,
        map_grid: GridMap = None,
        scene_manager=None,
    ) -> None:
        self.cfg_robot = CfgJetbot(**cfg_robot)
        super().__init__(
            # cfg_camera,
            # cfg_camera_third_person,
            scene=scene,
            map_grid=map_grid,
            scene_manager=scene_manager,
        )
        self.body = BodyJetbot(cfg_robot=self.cfg_robot, scene=scene)
        self.control_mode = "joint_velocities"
        # # self.scene.add(self.robot)  # 需要再考虑下, scene加入robot要放在哪一个class中, 可能放在scene好一些

        self.counter = 0
        self.pub_period = 50
        self.previous_pos = None
        self.movement_threshold = (
            0.1  # 移动时，如果两次检测之间的移动距离小于这个阈值，那么就会判定其为异常
        )

    def initialize(self) -> None:
        super().initialize()
        return

    def on_physics_step(self, step_size):
        super().on_physics_step(step_size)

        self.counter += 1

        if self.flag_world_reset:
            if self.flag_action_navigation:
                self.step(self.action)

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
