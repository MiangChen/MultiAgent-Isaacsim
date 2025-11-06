# Standard library imports
from dataclasses import dataclass, field
from typing import Dict, Optional, Tuple

# Local project imports
from config.config_manager import config_manager
from robot.sensor.camera import CfgCamera, CfgCameraThird
from robot.sensor.lidar import CfgLidar

ASSET_PATH = config_manager.get("path_asset")
ROBOT_TOPICS = config_manager.get("robot_topics")


@dataclass
class CfgRobot:
    type: str = "robot"
    id: int = 0
    namespace: str = "robot"
    path_prim_swarm: str = "/World/robot"
    path_prim_robot: Optional[str] = None
    path_usd: Optional[str] = None

    cfg_dict_camera: Dict[str, CfgCamera] = field(default_factory=dict)
    cfg_dict_lidar: Dict[str, CfgLidar] = field(default_factory=dict)
    cfg_dict_camera_third: Dict[str, CfgCameraThird] = field(default_factory=dict)

    position: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    quat: Tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
    scale: Tuple[float, float, float] = (1.0, 1.0, 1.0)
    base: Tuple[float, float, float] = (5.0, 3.0, 0.0)

    disable_gravity: bool = False  # 这个参数通常是在不使用物理引擎动力学的条件下用
    use_simplified_controller: bool = False

    # ROS Topics配置 (从全局配置中获取)
    topics: Dict[str, str] = field(default_factory=dict)

    def __post_init__(self):
        # 处理从字典加载时的类型转换
        if self.cfg_dict_camera:
            self.cfg_dict_camera = {
                name: CfgCamera(**camera_data)
                for name, camera_data in self.cfg_dict_camera.items()
            }

        if self.cfg_dict_lidar:
            self.cfg_dict_lidar = {
                name: CfgLidar(**lidar_data)
                for name, lidar_data in self.cfg_dict_lidar.items()
            }

        if self.cfg_dict_camera_third:
            self.cfg_dict_camera_third = {
                name: CfgCameraThird(**camera_data)
                for name, camera_data in self.cfg_dict_camera_third.items()
            }

        # 根据机器人类型设置ROS topics
        if not self.topics and self.type in ROBOT_TOPICS:
            self.topics = ROBOT_TOPICS[self.type].copy()
