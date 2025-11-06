"""
Robot skill module - Main interface for robot skills and skill management.
"""

# Import skill registry
from .skill_registry import SkillRegistry

# Import all base skills
from .base import *

__all__ = ["SkillRegistry"]
