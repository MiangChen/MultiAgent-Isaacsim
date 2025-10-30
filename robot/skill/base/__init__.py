"""
Robot skill base module - Unified interface for all robot skills.
"""

# Detection skills
from .detection import detect_skill

# Exploration skills
from .exploration import explore_skill, plan_exploration_waypoints_skill

# Manipulation skills
from .manipulation import pick_up_skill, put_down_skill

# Navigation skills
from .navigation import (
    navigate_to_skill,
    return_home_skill,
    TrajectoryManager,
    NodePlannerOmpl,
    NodeTrajectoryGenerator,
)

# Object detection skills
from .object_detection import object_detection_skill

# Photo skills
from .take_photo import take_photo

# Communication skills
from .broadcast import broadcast

__all__ = [
    # Detection
    "detect_skill",
    # Exploration
    "explore_skill",
    "plan_exploration_waypoints_skill",
    # Manipulation
    "pick_up_skill",
    "put_down_skill",
    # Navigation
    "navigate_to_skill",
    "return_home_skill",
    "TrajectoryManager",
    "NodePlannerOmpl",
    "NodeTrajectoryGenerator",
    # Object Detection
    "object_detection_skill",
    # Photo
    "take_photo",
    # Communication
    "broadcast",
]
