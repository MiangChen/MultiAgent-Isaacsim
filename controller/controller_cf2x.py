import numpy as np

from isaacsim.core.api.controllers import BaseController
from isaacsim.core.utils.types import ArticulationActions


class ControllerCf2x(BaseController):
    def __init__(self):
        super().__init__(name="cf2x_controller")
        # An open loop controller that uses a unicycle model
        return

    def forward(self, command):
        # command will have two elements, first element is the forward velocity
        # second element is the angular velocity (yaw only).
        joint_efforts = np.tile(np.array([0.11, 0.11, 0.11, 0.11]), (1,1))
        # A controller has to return an ArticulationAction
        return ArticulationActions(joint_efforts=joint_efforts)

    def velocity(self, command):
        command = np.tile(np.array(command), (1, 1))
        return ArticulationActions(joint_velocities=command)

    def set_position(self, position):
        """
        直接设置无人机的位置（瞬移）。
        Args:
            position: 目标位置，形如[x, y, z]
        Returns:
            ArticulationActions 或自定义动作对象
        """
        # 这里假设调用者直接操作实体位置，返回None即可
        return None

    def set_velocity(self, velocity):
        """
        设置无人机速度。
        Args:
            velocity: 速度向量 [vx, vy, vz]
        Returns:
            ArticulationActions 或自定义动作对象
        """
        # 这里假设调用者直接操作实体速度，返回None即可
        return None
