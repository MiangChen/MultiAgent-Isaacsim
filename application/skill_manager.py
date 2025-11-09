from typing import Dict, Any
from application.skill_registry import SkillRegistry


class SkillManager:
    def __init__(self, robot, auto_register: bool = True):
        self.robot = robot
        self.skills = {}
        self.skill_states = {}
        self.skill_data = {}
        self.skill_errors = {}
        
        # Auto-register all skills from registry
        if auto_register:
            self.register_all_from_registry()
    
    def get_skill_state(self, skill_name: str):
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
        self.skills[skill_name] = skill_function
    
    def register_all_from_registry(self):
        """Auto-register all skills from SkillRegistry"""
        all_skills = SkillRegistry.get_all_skills()
        for skill_name, skill_func in all_skills.items():
            self.register_skill(skill_name, skill_func)
    
    def execute_skill(self, skill_name: str, **kwargs) -> Dict[str, Any]:
        if skill_name not in self.skills:
            return {"status": "failed", "message": f"Skill {skill_name} not found"}
        
        kwargs['robot'] = self.robot
        kwargs['skill_manager'] = self
        return self.skills[skill_name](**kwargs)
    
    def form_feedback(self, status: str = "processing", message: str = "", progress: int = 100):
        return {"status": status, "message": message, "progress": progress}
