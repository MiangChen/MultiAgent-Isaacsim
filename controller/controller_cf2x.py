from isaacsim.core.api.controllers import BaseController
from isaacsim.core.utils.types import ArticulationAction


class ControllerCf2x(BaseController):
    def __init__(self):
        super().__init__(name="cf2x_controller")
        # An open loop controller that uses a unicycle model
        return

    def forward(self, command):
        # command will have two elements, first element is the forward velocity
        # second element is the angular velocity (yaw only).
        joint_velocities = [10000, 10000, 10000, 10000]
        # A controller has to return an ArticulationAction
        return ArticulationAction(joint_velocities=joint_velocities)

    def velocity(self, command):
        return ArticulationAction(joint_velocities=command)
