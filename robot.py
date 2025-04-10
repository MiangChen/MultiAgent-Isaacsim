import numpy as np
# from omni.isaac.core.prims import RigidPrim
# from omni.isaac.core.robots.robot import Robot as IsaacRobot
# from omni.isaac.core.scenes import Scene
from typing import List, Optional, Tuple

from isaacsim.core.prims import RigidPrim
# from isaacsim.core.robots.robot import Robot as IsaacRobot
from isaacsim.core.api.robots import Robot as IsaacRobot
from isaacsim.core.api.scenes import Scene
from pydantic import BaseModel


class BaseCfg(BaseModel):
    def update(self, **kwargs):
        return self.model_copy(update=kwargs, deep=True)

class RobotCfg(BaseCfg):
    """
    Represents a robot configuration with customizable attributes and optional components like controllers and sensors.

    This RobotCfg class is designed to store metadata and common configurations for robotic models. It inherits from BaseCfg,
    providing a structured way to define a robot's properties within a simulation or robotic application context. The model includes
    details about the robot's USD (Universal Scene Description) path, initial position, orientation, and other settings crucial for
    simulation initialization and control.

    Attributes:
        name (str): The name_prefix identifier for the robot.
        type (str): The type or category of the robot.
        prim_path (str): The USD prim path where the robot is located or should be instantiated within a scene.
        create_robot (bool, optional): Flag indicating whether to create the robot instance during simulation setup. Defaults to True.
        usd_path (Optional[str], optional): The file path to the USD containing the robot definition. If None, a default path is used.

        position (Optional[List[float]], optional): Initial position of the robot in world frame. Defaults to (0.0, 0.0, 0.0).
        orientation (Optional[List[float]], optional): Initial orientation of the robot in quaternion. Defaults to None.
        scale (Optional[List[float]], optional): Scaling factor for the robot. Defaults to None.

        controllers (Optional[List[ControllerCfg]], optional): List of controller configurations attached to the robot. Defaults to None.
        sensors (Optional[List[SensorCfg]], optional): List of sensor configurations attached to the robot. Defaults to None.
    """
    # meta info
    name_prefix: str
    type: str
    prim_path: str
    usd_path: Optional[str] = None  # If Optional, use default usd_path

    # common config
    position: Optional[Tuple[float, float, float]] = (0.0, 0.0, 0.0)
    orientation: Optional[Tuple[float, float, float, float]] = (0.0, 0.0, 0.0, 1.0)
    scale: Optional[Tuple[float, float, float]] = (1.0, 1.0, 1.0)
    # controllers: Optional[List[ControllerCfg]] = None
    # sensors: Optional[List[SensorCfg]] = None


class RobotBase:
    """Base class of robot."""

    def __init__(self, config: RobotCfg, scene: Scene):

        self.config = config
        self.robot_entity: IsaacRobot | None = None  # 代表机器人的实体
        self.controllers = {}
        self.sensors = {}
        self.scene = scene

    def set_up_to_scene(self, scene: Scene):
        """Set up robot in the scene.

        Args:
            scene (Scene): scene to set up.
        """
        # self._scene = scene
        robot_cfg = self.config
        if self.robot_entity:
            scene.add(self.robot_entity)
            # log.debug('self.robot_entity: ' + str(self.robot_entity))
        for rigid_body in self.get_rigid_bodies():
            scene.add(rigid_body)
        # from grutopia.core.robot.controller import BaseController, create_controllers
        # from grutopia.core.robot.sensor import BaseSensor, create_sensors
        #
        # self.controllers: Dict[str, BaseController] = create_controllers(robot_cfg, self, scene)
        # self.sensors: Dict[str, BaseSensor] = create_sensors(robot_cfg, self, scene)

    def post_reset(self):
        """Set up things that happen after the world resets."""
        for sensor in self.sensors.values():
            sensor.post_reset()

    def cleanup(self):
        for controller in self.controllers.values():
            controller.cleanup()
        for sensor in self.sensors.values():
            sensor.cleanup()
        for rigid_body in self.get_rigid_bodies():
            self._scene.remove_object(rigid_body.name_prefix)
            log.debug(f'rigid body {rigid_body} removed')
        log.debug(f'robot {self.name} clean up')

    def apply_action(self, action: dict):
        """Apply actions of controllers to robot.

        Args:
            action (dict): action dict.
              key: controller name_prefix.
              value: corresponding action array.
        """
        raise NotImplementedError()

    def get_obs(self) -> dict:
        """Get observation of robot, including controllers, sensors, and world pose.

        Raises:
            NotImplementedError: _description_
        """
        raise NotImplementedError()

    def get_robot_base(self) -> RigidPrim:
        """
        Get base link of robot.

        Returns:
            RigidPrim: rigid prim of robot base link.
        """
        raise NotImplementedError()

    def get_robot_scale(self) -> np.ndarray:
        """Get robot scale.

        Returns:
            np.ndarray: robot scale in (x, y, z).
        """
        return self.robot_entity.get_local_scale()

    def get_robot_articulation(self) -> IsaacRobot:
        """Get isaac robots instance (articulation).

        Returns:
            Robot: robot articulation.
        """
        return self.robot_entity

    def get_controllers(self):
        return self.controllers

    def get_rigid_bodies(self) -> List[RigidPrim]:
        return []

    @classmethod
    def register(cls, name: str):
        """Register a robot class with its name_prefix(decorator).

        Args:
            name(str): name_prefix of the robot class.
        """

        def decorator(robot_class):
            cls.robots[name] = robot_class

            @wraps(robot_class)
            def wrapped_function(*args, **kwargs):
                return robot_class(*args, **kwargs)

            return wrapped_function

        return decorator

#
# def create_robots(runtime: TaskRuntime, scene: Scene) -> Dict[str, RobotBase]:
#     """Create robot instances in runtime.
#
#     Args:
#         runtime (TaskRuntime): task runtime.
#         scene (Scene): isaac scene.
#
#     Returns:
#         Dict[str, RobotBase]: robot instances dictionary.
#     """
#     robot_map = {}
#     for robot in runtime.robots:
#         if robot.type not in RobotBase.robots:
#             raise KeyError(f'unknown robot type "{robot.type}"')
#         robot_cls = RobotBase.robots[robot.type]
#         robot_ins: RobotBase = robot_cls(robot, scene)
#         robot_map[robot.name_prefix] = robot_ins
#         robot_ins.set_up_to_scene(scene)
#         log.debug(f'===== {robot.name_prefix} loaded =====')
#     return robot_map

if __name__ == "__main__":
    config = {
        'name_prefix': 'jetbot3',
        'prim_path':'/World/Fancy_Robot3',

    # 'wheel_dof_names': ["left_wheel_joint", "right_wheel_joint"],
    }
    # create_robot=True,
    # usd_path=jet_robot_asset_path,
    # position=[-2, 0, 0],
    # }
    # type: str
    # prim_path: str
    # create_robot: bool = True
    # usd_path: Optional[str] = None  # If Optional, use default usd_path

    # common config
    position: Optional[Tuple[float, float, float]] = (0.0, 0.0, 0.0)
    orientation: Optional[Tuple[float, float, float, float]] = None
    scale: Optional[Tuple[float, float, float]] = None

    jetbot_config = RobotCfg()