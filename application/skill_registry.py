from typing import Dict, Callable


class SkillRegistry:
    _skills: Dict[str, Callable] = {}
    _imported = False
    
    @classmethod
    def register(cls, skill_name=None):
        """Decorator: @SkillRegistry.register()"""
        def decorator(skill_func: Callable) -> Callable:
            name = skill_name or skill_func.__name__
            cls._skills[name] = skill_func
            return skill_func
        
        # Support @register and @register()
        if callable(skill_name):
            skill_func = skill_name
            cls._skills[skill_func.__name__] = skill_func
            return skill_func
        
        return decorator
    
    @classmethod
    def get_all_skills(cls) -> Dict[str, Callable]:
        """Get all registered skills (auto-import if needed)"""
        if not cls._imported:
            cls._import_all_skills()
        return cls._skills.copy()
    
    @classmethod
    def get_skill(cls, skill_name: str) -> Callable:
        """Get skill by name (auto-import if needed)"""
        if not cls._imported:
            cls._import_all_skills()
        return cls._skills.get(skill_name)
    
    @classmethod
    def _import_all_skills(cls):
        """Import all skills to trigger @register decorators"""
        if cls._imported:
            return
        
        # Import all skill modules
        from application.skills import (
            navigate_to, explore, detect, track, take_photo, object_detection,
            take_off,
            pick_up, put_down,
            move,
        )
        
        cls._imported = True
