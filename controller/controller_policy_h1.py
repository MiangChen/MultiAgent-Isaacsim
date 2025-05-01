# Copyright (c) 2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from typing import Optional

from controller.controller_policy import PolicyController

import numpy as np
import omni
import omni.kit.commands
from isaacsim.core.utils.rotations import quat_to_rot_matrix
from isaacsim.core.utils.types import ArticulationActions
# from isaacsim.storage.native import get_assets_root_path
from isaacsim.core.utils.nucleus import get_assets_root_path


class H1FlatTerrainPolicy(PolicyController):
    """The H1 Humanoid running Flat Terrain Policy Locomotion Policy"""

    def __init__(
            self,
            prim_path: str,
            root_path: Optional[str] = None,
            name: str = "h1",
            usd_path: Optional[str] = None,
            position: Optional[np.ndarray] = None,
            orientation: Optional[np.ndarray] = None,
    ) -> None:
        """
        Initialize H1 robot and import flat terrain policy.

        Args:
            prim_path (str) -- prim path of the robot on the stage
            root_path (Optional[str]): The path to the articulation root of the robot
            name (str) -- name of the quadruped
            usd_path (str) -- robot usd filepath in the directory
            position (np.ndarray) -- position of the robot
            orientation (np.ndarray) -- orientation of the robot

        """
        assets_root_path = get_assets_root_path()
        if usd_path == None:
            usd_path = assets_root_path + "/Isaac/Robots/Unitree/H1/h1.usd"
        super().__init__(name, prim_path, root_path, usd_path, position, orientation)

        self.load_policy(
            assets_root_path + "/Isaac/Samples/Policies/H1_Policies/h1_policy.pt",
            assets_root_path + "/Isaac/Samples/Policies/H1_Policies/h1_env.yaml",
            # 本地的
            # "/home/ubuntu/PycharmProjects/multiagent-isaacsim/IsaacLab/logs/rsl_rl/h1_flat/2025-04-14_19-39-24/exported/policy.pt",
            # "/home/ubuntu/PycharmProjects/multiagent-isaacsim/IsaacLab/logs/rsl_rl/h1_flat/2025-04-14_19-39-24/params/env.yaml",
        )
        self._action_scale = 0.5
        self._previous_action = np.zeros(19)
        self._policy_counter = 0
        self.base_command = np.zeros(3)

    def _compute_observation(self, command, robot):
        """
        Compute the observation vector for the policy.

        Argument:
        command (np.ndarray) -- the robot command (v_x, v_y, w_z)

        Returns:
        np.ndarray -- The observation vector.
        """
        lin_vel_I = robot.get_linear_velocities()[0]  # shape(1,3) -> shape(3)
        ang_vel_I = robot.get_angular_velocities()[0]
        pos_IB, q_IB = robot.get_world_poses()
        pos_IB, q_IB = pos_IB[0], q_IB[0]

        R_IB = quat_to_rot_matrix(q_IB)
        R_BI = R_IB.transpose()
        lin_vel_b = np.matmul(R_BI, lin_vel_I)
        ang_vel_b = np.matmul(R_BI, ang_vel_I)
        gravity_b = np.matmul(R_BI, np.array([0.0, 0.0, -1.0]))

        obs = np.zeros(69)
        # Base lin vel
        obs[:3] = lin_vel_b
        # Base ang vel
        obs[3:6] = ang_vel_b
        # Gravity
        obs[6:9] = gravity_b
        # Command
        obs[9:12] = command
        # Joint states
        current_joint_pos = robot.get_joint_positions()[0]
        current_joint_vel = robot.get_joint_velocities()[0]
        obs[12:31] = current_joint_pos - self.default_pos
        obs[31:50] = current_joint_vel
        # Previous Action
        obs[50:69] = self._previous_action
        return obs

    def forward(self, dt, command, robot) -> np.ndarray:
        """
        Compute the desired articulation action and apply them to the robot articulation.

        Argument:
        dt (float) -- Timestep update in the world.
        command (np.ndarray) -- the robot command (v_x, v_y, w_z)

        """
        if self._policy_counter % self._decimation == 0:
            obs = self._compute_observation(command, robot)
            self.action = self._compute_action(obs)
            self._previous_action = self.action.copy()
        position = np.tile(np.array(self.default_pos + (self.action * self._action_scale)), (1,1))
        self._policy_counter += 1
        return position

    def initialize(self, robot):
        """
        Overloads the default initialize function to use default articulation root properties in the USD
        """
        return super().initialize(set_articulation_props=False, robot=robot)

    def on_physics_step(self, step_size) -> None:
        # global first_step
        # global reset_needed
        # if first_step:
        #     for robot in robots:
        #         robot.initialize()
        #     first_step = False
        # elif reset_needed:
        #     my_world.reset(True)
        #     reset_needed = False
        #     first_step = True
        # else:
        #     for robot in robots:
        #         robot.forward(step_size, base_command)
        #
        self.forward(step_size, self.base_command)
