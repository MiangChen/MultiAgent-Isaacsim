"""Skill Manager - Application layer skill management"""

from typing import Dict, Any, Optional, List
from application.skill_registry import SkillRegistry


class SkillManager:
    """Manage skills for a single robot"""
    
    def __init__(self, robot, auto_register: bool = False):
        self.robot = robot
        self.namespace = robot.namespace
        self.robot_type = getattr(robot.body.cfg_robot, 'robot_type', 'unknown') if hasattr(robot, 'body') else 'unknown'
        
        self.skill_states = {}
        self.skill_functions = {}
        self.skill_params = {}
        self.skill_data = {}
        self.skill_errors = {}
        self.skill_feedbacks = {}
        
        # Auto-register skills if requested
        if auto_register:
            self.auto_register_skills()
    
    def get_skill_state(self, skill_name: str) -> Optional[str]:
        return self.skill_states.get(skill_name)
    
    def set_skill_state(self, skill_name: str, state: str):
        self.skill_states[skill_name] = state
    
    def get_skill_data(self, skill_name: str, key: str, default=None):
        if skill_name not in self.skill_data:
            self.skill_data[skill_name] = {}
        return self.skill_data[skill_name].get(key, default)
    
    def set_skill_data(self, skill_name: str, key: str, value):
        if skill_name not in self.skill_data:
            self.skill_data[skill_name] = {}
        self.skill_data[skill_name][key] = value
    
    def register_skill(self, skill_name: str, skill_function):
        """Register a skill function"""
        self.skill_functions[skill_name] = skill_function
    
    def register_skill_from_registry(self, skill_name: str):
        """Register a skill from the global registry"""
        skill_func = SkillRegistry.get_skill(skill_name)
        if skill_func:
            self.register_skill(skill_name, skill_func)
        else:
            raise ValueError(f"Skill '{skill_name}' not found in registry")
    
    def auto_register_skills(self):
        """Auto-register all supported skills for this robot type"""
        supported_skills = SkillRegistry.get_skills_for_robot_type(self.robot_type)
        
        for skill_name in supported_skills:
            try:
                self.register_skill_from_registry(skill_name)
            except ValueError:
                # Skill not in registry, skip
                pass
    
    def get_supported_skills(self) -> List[str]:
        """Get list of supported skills for this robot type"""
        return SkillRegistry.get_skills_for_robot_type(self.robot_type)
    
    def is_skill_supported(self, skill_name: str) -> bool:
        """Check if a skill is supported by this robot type"""
        return SkillRegistry.is_skill_supported(self.robot_type, skill_name)
    
    def execute_skill(self, skill_name: str, **kwargs) -> Dict[str, Any]:
        if skill_name not in self.skill_functions:
            return {"status": "failed", "message": f"Skill {skill_name} not found"}
        
        skill_func = self.skill_functions[skill_name]
        kwargs['robot'] = self.robot
        kwargs['skill_manager'] = self
        
        result = skill_func(**kwargs)
        
        # Store feedback
        if result:
            self.skill_feedbacks[skill_name] = result
        
        return result
    
    def reset_skill(self, skill_name: str):
        self.skill_states.pop(skill_name, None)
        self.skill_data.pop(skill_name, None)
        self.skill_errors.pop(skill_name, None)
        self.skill_feedbacks.pop(skill_name, None)
    
    def form_feedback(self, status: str = "processing", message: str = "none", progress: int = 100):
        return {"status": status, "message": message, "progress": progress}
