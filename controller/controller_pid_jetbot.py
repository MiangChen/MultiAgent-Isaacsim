from isaacsim.core.api.controllers import BaseController
from isaacsim.core.utils.types import ArticulationAction


class ControllerJetbot(BaseController):
    def __init__(self):
        super().__init__(name="my_cool_controller")
        # An open loop controller that uses a unicycle model
        self._wheel_radius = 0.03
        self._wheel_base = 0.1125
        return

    def forward(self, command):
        # command will have two elements, first element is the forward velocity
        # second element is the angular velocity (yaw only).
        joint_velocities = [0.0, 0.0]
        joint_velocities[0] = ((2 * command[0]) - (command[1] * self._wheel_base)) / (2 * self._wheel_radius)
        joint_velocities[1] = ((2 * command[0]) + (command[1] * self._wheel_base)) / (2 * self._wheel_radius)
        # A controller has to return an ArticulationAction
        # return ArticulationAction(joint_velocities=joint_velocities)
        return joint_velocities

    def velocity(self, command):
        return ArticulationAction(joint_velocities=command)
