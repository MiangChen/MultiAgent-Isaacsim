from dataclasses import dataclass, field
from typing import List, Tuple

# Local project imports
from .cfg_robot import CfgRobot, ASSET_PATH


@dataclass
class CfgTarget(CfgRobot):
    type: str = "target"
    path_prim_robot: str = "/World/robot/car1"
    path_usd: str = (
        ASSET_PATH + "/Isaac/Robots/Jetbot/jetbot.usd"
    )  # 先用jetbot的模型来当目标
    robot_radius: float = 0.2
    move_path: List[Tuple[float, float, float]] = field(
        default_factory=lambda: [(0, 0, 0), (11, 26, 0), (-11, 26, 0)]
    )
