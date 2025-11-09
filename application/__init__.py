"""
Application Layer - Skills and high-level logic

This layer is independent from simulation layer.
Uses ROS for communication and Control objects for execution.
"""

from application.skill_manager import SkillManager
from application.skill_registry import SkillRegistry

__all__ = [
    'SkillManager',
    'SkillRegistry',
]
