from typing import List, Optional, Tuple

from map.map_grid_map import GridMap
from robot.robot_cfg import RobotCfg


import numpy as np
from isaacsim.core.prims import RigidPrim
# from isaacsim.core.robots.robot import Robot as IsaacRobot  # 已经在4.5中取消了
# from isaacsim.core.api.robots import Robot as IsaacRobot
from isaacsim.core.api.scenes import Scene


class RobotBase:
    """Base class of robot."""

    def __init__(self, config: RobotCfg, scene: Scene, map_grid: GridMap=None):

        self.config = config
        self.scene = scene
        self.map_grid = GridMap
        # self.robot_entity: IsaacRobot | None = None  # 代表机器人的实体
        self.controllers = {}
        self.sensors = {}
        self.flag_world_reset = False  # 用来记录下世界是不是被初始化了
        self.flag_action_navigation = False  # 用来记录是不是启动导航了

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

    def navigate_to(self, target_pos, reset_flag: bool = False):
        """
        让机器人导航到某一个位置,
        不需要输入机器人的起始位置, 因为机器人默认都是从当前位置出发的
        """

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

    def get_robot_articulation(self):
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


if __name__ == "__main__":
    config = {
        'name_prefix': 'jetbot3',
        'prim_path': '/World/Fancy_Robot3',

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
