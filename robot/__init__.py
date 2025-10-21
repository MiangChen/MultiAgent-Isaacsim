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
# from .robot_trajectory import RobotTrajectory
from .target import Target
from .swarm_manager import SwarmManager

# Robot components
from .body import *
from .cfg import *
from .controller import *
from .sensor import *
from .skill import *

__all__ = [
    # Main robot classes
    "Robot",
    "RobotDroneAutel",
    "RobotCf2x",
    "RobotG1",
    "RobotH1",
    "RobotJetbot",
    # "RobotTrajectory",
    "Target",
    "SwarmManager",
    
    # All exports from submodules are included via *
]