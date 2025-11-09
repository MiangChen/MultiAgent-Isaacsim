"""Skill Registry - Application Layer

Centralized skill registration and management system.
"""

import os
import yaml
from typing import Dict, List, Callable, Optional


class SkillRegistry:
    """
    Application layer skill registry
    
    Manages skill registration and robot type compatibility.
    """
    
    _skills: Dict[str, Callable] = {}
    _skill_config: Optional[Dict] = None
    _robot_type_skills: Dict[str, List[str]] = {}
    
    @classmethod
    def register(cls, skill_name: Optional[str] = None):
        """
        Decorator to register a skill function
        
        Args:
            skill_name: Name of the skill (optional, defaults to function name)
            
        Usage:
            @SkillRegistry.register()
            def navigate_to(**kwargs):
                pass
            
            @SkillRegistry.register('custom_name')
            def my_skill(**kwargs):
                pass
        """
        def decorator(skill_func: Callable) -> Callable:
            name = skill_name or skill_func.__name__
            cls._skills[name] = skill_func
            return skill_func
        
        # Support both @register and @register()
        if callable(skill_name):
            skill_func = skill_name
            name = skill_func.__name__
            cls._skills[name] = skill_func
            return skill_func
        
        return decorator
    
    @classmethod
    def get_skill(cls, skill_name: str) -> Optional[Callable]:
        """
        Get skill function by name
        
        Args:
            skill_name: Name of the skill
            
        Returns:
            Skill function or None if not found
        """
        return cls._skills.get(skill_name)
    
    @classmethod
    def get_all_skills(cls) -> Dict[str, Callable]:
        """
        Get all registered skills
        
        Returns:
            Dictionary of skill name to skill function
        """
        return cls._skills.copy()
    
    @classmethod
    def get_skill_names(cls) -> List[str]:
        """
        Get all registered skill names
        
        Returns:
            List of skill names
        """
        return list(cls._skills.keys())
    
    @classmethod
    def load_config(cls, config_path: Optional[str] = None) -> None:
        """
        Load skill configuration from YAML file
        
        Args:
            config_path: Path to config file, defaults to application/skill_config.yaml
        """
        if config_path is None:
            current_dir = os.path.dirname(__file__)
            config_path = os.path.join(current_dir, "skill_config.yaml")
        
        try:
            with open(config_path, "r", encoding="utf-8") as f:
                cls._skill_config = yaml.safe_load(f)
            
            # Build robot type to skills mapping
            cls._robot_type_skills = {}
            for category, info in cls._skill_config.items():
                robot_types = info.get("robot_types", [])
                skills = info.get("skills", [])
                
                for robot_type in robot_types:
                    if robot_type not in cls._robot_type_skills:
                        cls._robot_type_skills[robot_type] = []
                    cls._robot_type_skills[robot_type].extend(skills)
            
            # Remove duplicates
            for robot_type in cls._robot_type_skills:
                cls._robot_type_skills[robot_type] = list(set(cls._robot_type_skills[robot_type]))
        
        except Exception as e:
            raise RuntimeError(f"Failed to load skill config from {config_path}: {e}")
    
    @classmethod
    def get_skills_for_robot_type(cls, robot_type: str) -> List[str]:
        """
        Get supported skills for a robot type
        
        Args:
            robot_type: Type of robot (e.g., 'jetbot', 'cf2x', 'h1')
            
        Returns:
            List of supported skill names
        """
        if cls._skill_config is None:
            cls.load_config()
        
        return cls._robot_type_skills.get(robot_type, [])
    
    @classmethod
    def is_skill_supported(cls, robot_type: str, skill_name: str) -> bool:
        """
        Check if a skill is supported by a robot type
        
        Args:
            robot_type: Type of robot
            skill_name: Name of the skill
            
        Returns:
            True if supported, False otherwise
        """
        supported_skills = cls.get_skills_for_robot_type(robot_type)
        return skill_name in supported_skills
    
    @classmethod
    def import_all_skills(cls) -> None:
        """
        Import all skills to trigger decorator registration
        
        This ensures all skills are registered when called.
        """
        # Import all skill modules to trigger @register decorators
        from application.skills import (
            navigate_to,
            explore,
            take_off,
            pick_up,
            put_down,
            take_photo,
            detect,
            object_detection
        )
    
    @classmethod
    def clear(cls) -> None:
        """Clear all registered skills"""
        cls._skills.clear()
        cls._robot_type_skills.clear()
        cls._skill_config = None
