# Base skills - 基础技能 (所有机器人通用)
from .base import (
    navigate_to,
    explore,
    detect,
    track,
    take_photo,
    object_detection,
)

# Drone skills - 无人机技能
from .drone import take_off

# Manipulation skills - 机械臂操作技能
from .manipulation import (
    pick_up,
    put_down,
)

# Target skills - 目标机器人技能
from .target import move

__all__ = [
    # Base
    'navigate_to',
    'explore',
    'detect',
    'track',
    'take_photo',
    'object_detection',
    # Drone
    'take_off',
    # Manipulation
    'pick_up',
    'put_down',
    # Target
    'move',
]
