from isaacsim.core.api.controllers import BaseController
from isaacsim.core.utils.types import ArticulationAction


from typing import List, Optional, Tuple

from pydantic import BaseModel


class BaseCfg(BaseModel):
    def update(self, **kwargs):
        return self.model_copy(update=kwargs, deep=True)



class ControllerCfg(BaseCfg, extra='allow'):
    """
    A specialized model representing controllers within a system, inheriting from BaseModel with an extended configuration to allow additional keys.

    This class serves as a data structure to encapsulate controller-related information crucial for configuring and managing control systems, particularly in robotic applications. It provides a structured way to define controllers, including their name_prefix, type, sub-controllers (if any), and a reference frame.

    Attributes:
        name (str): The unique identifier for the controller.
        type (str): Specifies the controller's type or category, determining its behavior and functionality.
        sub_controllers (Optional[List[ControllerModel]], optional): A list of nested 'ControllerModel' instances, enabling hierarchical controller structures. Defaults to None.

    Usage:
        Instantiate a `ControllerModel` to define a controller configuration. Optionally, nest other `ControllerModel` instances within the `sub_controllers` attribute to model complex controller hierarchies.

    Example Type Hints Usage:
        - `name_prefix`: Always a string.
        - `type`: String defining controller type.
        - `sub_controllers`: A list of `ControllerModel` instances or None.
    """

    name: str
    type: str
    sub_controllers: Optional[List['ControllerCfg']] = None


class DifferentialDriveControllerCfg(ControllerCfg):

    type: Optional[str] = 'DifferentialDriveController'
    wheel_radius: float
    wheel_base: float

class MoveAlongPathPointsControllerCfg(ControllerCfg):
    name: Optional[str] = 'move_along_path'
    type: Optional[str] = 'MoveAlongPathPointsController'
    forward_speed: Optional[float] = None
    rotation_speed: Optional[float] = None
    threshold: Optional[float] = None

class MoveToPointBySpeedControllerCfg(ControllerCfg):

    type: Optional[str] = 'MoveToPointBySpeedController'
    forward_speed: Optional[float] = None
    rotation_speed: Optional[float] = None
    threshold: Optional[float] = None

#
# move_by_speed_cfg = DifferentialDriveControllerCfg(name='move_by_speed', wheel_base=0.1125, wheel_radius=0.03)
#
# move_to_point_cfg = MoveToPointBySpeedControllerCfg(
#     name='move_to_point',
#     forward_speed=1.0,
#     rotation_speed=1.0,
#     threshold=0.1,
#     sub_controllers=[move_by_speed_cfg],
# )
#
# move_along_path_cfg = MoveAlongPathPointsControllerCfg(
#     name='move_along_path',
#     forward_speed=1.0,
#     rotation_speed=1.0,
#     threshold=0.1,
#     sub_controllers=[move_to_point_cfg],
# )
#

#
# class CoolController(BaseController):
#     def __init__(self):
#         super().__init__(name_prefix="my_cool_controller")
#         # An open loop controller that uses a unicycle model
#         self._wheel_radius = 0.03
#         self._wheel_base = 0.1125
#         return
#
#     def forward(self, command):
#         # command will have two elements, first element is the forward velocity
#         # second element is the angular velocity (yaw only).
#         joint_velocities = [0.0, 0.0]
#         joint_velocities[0] = ((2 * command[0]) - (command[1] * self._wheel_base)) / (2 * self._wheel_radius)
#         joint_velocities[1] = ((2 * command[0]) + (command[1] * self._wheel_base)) / (2 * self._wheel_radius)
#         # A controller has to return an ArticulationAction
#         return ArticulationAction(joint_velocities=joint_velocities)
#
