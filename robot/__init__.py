"""
Robot module - Main interface for robot implementations and components.
"""

# Main robot classes
from .robot import Robot
from .robot_drone_autel import RobotDroneAutel
from .robot_drone_cf2x import RobotCf2x
from .robot_g1 import RobotG1
from .robot_h1 import RobotH1
from .robot_jetbot import RobotJetbot

from .target import Target

# Robot components
from .body import *
from .cfg import *
from .controller import *

__all__ = [
    # Main robot classes
    "Robot",
    "RobotDroneAutel",
    "RobotCf2x",
    "RobotG1",
    "RobotH1",
    "RobotJetbot",
    "Target",
]
