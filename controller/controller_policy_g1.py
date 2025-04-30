# Copyright (c) 2024, NVIDIA CORPORATION. All rights reserved.

#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from typing import Optional


import numpy as np
import omni
import omni.kit.commands
from isaacsim.core.utils.nucleus import get_assets_root_path
from isaacsim.core.utils.rotations import quat_to_rot_matrix
from isaacsim.core.utils.types import ArticulationAction
# from isaacsim.robot.policy.examples.controllers import PolicyController
# from isaacsim.storage.native import get_assets_root_path  #  暂不清除和nuclues的区别


class G1FlatTerrainPolicy(PolicyController):
    """The G1 Humanoid running Flat Terrain Policy Locomotion Policy"""

    def __init__(
            self,
            prim_path: str,
            root_path: Optional[str] = None,
            name: str = "g1",
            usd_path: Optional[str] = None,
            position: Optional[np.ndarray] = None,
            orientation: Optional[np.ndarray] = None,
    ) -> None:
        """
        Initialize G1 robot and import flat terrain policy.

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
            usd_path = assets_root_path + f"/Isaac/Robots/Unitree/G1/{name}.usd"
        super().__init__(name, prim_path, root_path, usd_path, position, orientation)
        self.load_policy(
            "/home/ubuntu/Downloads/robots/g1/policy/move_by_speed/g1_15000.onnx",
            assets_root_path + "/Isaac/Samples/Policies/H1_Policies/h1_env.yaml",
        )
        self._action_scale = 0.5
        self._previous_action = np.zeros(19)
        self._policy_counter = 0
        # from controller.controller_policy import PolicyController
        # self.policy_runner = PolicyController()


    def _compute_observation(self, command):
        """
        Compute the observation vector for the policy.

        Argument:
        command (np.ndarray) -- the robot command (v_x, v_y, w_z)

        Returns:
        np.ndarray -- The observation vector.

        """
        lin_vel_I = self.robot.get_linear_velocity()
        ang_vel_I = self.robot.get_angular_velocity()
        pos_IB, q_IB = self.robot.get_world_pose()

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
        current_joint_pos = self.robot.get_joint_positions()
        current_joint_vel = self.robot.get_joint_velocities()
        obs[12:31] = current_joint_pos - self.default_pos
        obs[31:50] = current_joint_vel
        # Previous Action
        obs[50:69] = self._previous_action
        return obs

    def forward(self, dt, command):
        """
        Compute the desired articulation action and apply them to the robot articulation.

        Argument:
        dt (float) -- Timestep update in the world.
        command (np.ndarray) -- the robot command (v_x, v_y, w_z)

        """
        if self._policy_counter % self._decimation == 0:
            obs = self._compute_observation(command)
            self.action = self._compute_action(obs)
            self._previous_action = self.action.copy()

        action = ArticulationAction(joint_positions=self.default_pos + (self.action * self._action_scale))
        self.robot.apply_action(action)

        self._policy_counter += 1

    def initialize(self):
        """
        Overloads the default initialize function to use default articulation root properties in the USD
        """
        return super().initialize(set_articulation_props=False)

if __name__ == "__main__":
    pass