# Standard library imports
from dataclasses import dataclass, field
from typing import Dict, Optional, Tuple

# Local project imports
from config.config_manager import config_manager

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

    # Note: Sensor configuration removed
    # Sensors are now created using CARLA-style Blueprint system
    # See main_example.py for examples of adding sensors to robots

    position: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    quat: Tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
    scale: Tuple[float, float, float] = (1.0, 1.0, 1.0)
    base: Tuple[float, float, float] = (5.0, 3.0, 0.0)

    disable_gravity: bool = False  # 这个参数通常是在不使用物理引擎动力学的条件下用
    use_simplified_controller: bool = False

    # ROS Topics配置 (从全局配置中获取)
    topics: Dict[str, str] = field(default_factory=dict)

    def __post_init__(self):
        # Note: Sensor configuration processing removed
        # Sensors are now created using CARLA-style Blueprint system
        pass

        # 根据机器人类型设置ROS topics
        if not self.topics and self.type in ROBOT_TOPICS:
            self.topics = ROBOT_TOPICS[self.type].copy()
