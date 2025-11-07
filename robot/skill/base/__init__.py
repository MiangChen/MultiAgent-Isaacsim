"""
Robot skill base module - Unified interface for all robot skills.
"""

# Detection skills
from .detection import detect

# Exploration skills
from .exploration import explore, plan_exploration_waypoints

# Manipulation skills
from .manipulation import pick_up, put_down

# Navigation skills
from .navigation import (
    navigate_to,
    return_home,
    NodePlannerOmpl,
    NodeTrajectoryGenerator,
)

# Object detection skills
from .object_detection import object_detection

# Photo skills
from .take_photo import take_photo

# Communication skills
from .broadcast import broadcast

__all__ = [
    # Detection
    "detect",
    # Exploration
    "explore",
    "plan_exploration_waypoints",
    # Manipulation
    "pick_up",
    "put_down",
    # Navigation
    "navigate_to",
    "return_home",
    "NodePlannerOmpl",
    "NodeTrajectoryGenerator",
    # Object Detection
    "object_detection",
    # Photo
    "take_photo",
    # Communication
    "broadcast",
]
