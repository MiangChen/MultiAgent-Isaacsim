"""Skill Manager - Application layer skill management"""

from typing import Dict, Any, Optional


class SkillManager:
    """Manage skills for a single robot"""
    
    def __init__(self, robot):
        self.robot = robot
        self.namespace = robot.namespace
        self.skill_states = {}
        self.skill_functions = {}
        self.skill_params = {}
        self.skill_data = {}
        self.skill_errors = {}
    
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
        self.skill_functions[skill_name] = skill_function
    
    def execute_skill(self, skill_name: str, **kwargs) -> Dict[str, Any]:
        if skill_name not in self.skill_functions:
            return {"status": "failed", "message": f"Skill {skill_name} not found"}
        
        skill_func = self.skill_functions[skill_name]
        kwargs['robot'] = self.robot
        kwargs['skill_manager'] = self
        return skill_func(**kwargs)
    
    def reset_skill(self, skill_name: str):
        self.skill_states.pop(skill_name, None)
        self.skill_data.pop(skill_name, None)
        self.skill_errors.pop(skill_name, None)
    
    def form_feedback(self, status: str = "processing", message: str = "none", progress: int = 100):
        return {"status": status, "message": message, "progress": progress}
