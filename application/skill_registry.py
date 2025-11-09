"""
Skill Registry - Application layer

Registry for robot skills, decoupled from simulation layer.
"""


class SkillRegistry:
    """Global skill registry"""
    _skills = {}
    
    @classmethod
    def register(cls, name: str = None):
        """Decorator to register a skill function"""
        def decorator(func):
            skill_name = name or func.__name__
            cls._skills[skill_name] = func
            return func
        return decorator
    
    @classmethod
    def get_skill(cls, name: str):
        """Get a registered skill function"""
        return cls._skills.get(name)
    
    @classmethod
    def list_skills(cls):
        """List all registered skills"""
        return list(cls._skills.keys())
    
    @classmethod
    def clear(cls):
        """Clear all registered skills"""
        cls._skills.clear()
