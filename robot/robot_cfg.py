from typing import List, Optional, Tuple
from pydantic import BaseModel
# from isaacsim.storage.native import get_assets_root_path
from isaacsim.core.utils.nucleus import get_assets_root_path

assets_root_path = get_assets_root_path()  # 不可以放在
if assets_root_path is None:
    print("Could not find nucleus server with /Isaac folder")

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
