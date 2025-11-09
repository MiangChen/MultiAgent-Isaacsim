"""
Skill Manager - Application layer skill management

Manages robot skills independently from simulation layer.
Skills use ROS for communication and Control objects for execution.
"""

from typing import Dict, Any, Optional


class SkillManager:
    """Manage skills for a single robot (Application layer)"""
    
    def __init__(self, robot):
        self.robot = robot
        self.namespace = robot.namespace
        
        # Skill state management (copied from robot.py)
        self.skill_states = {}  # {"navigate_to": "EXECUTING", ...}
        self.skill_functions = {}  # {"navigate_to": navigate_to_func, ...}
        self.skill_params = {}  # {"navigate_to": {...}, ...}
        self.skill_data = {}  # Private data for each skill
        self.skill_errors = {}  # Error messages
    
    def get_skill_state(self, skill_name: str) -> Optional[str]:
        """Get skill state"""
        return self.skill_states.get(skill_name)
    
    def set_skill_state(self, skill_name: str, state: str):
        """Set skill state"""
        self.skill_states[skill_name] = state
    
    def get_skill_data(self, skill_name: str, key: str, default=None):
        """Get skill private data"""
        if skill_name not in self.skill_data:
            self.skill_data[skill_name] = {}
        return self.skill_data[skill_name].get(key, default)
    
    def set_skill_data(self, skill_name: str, key: str, value):
        """Set skill private data"""
        if skill_name not in self.skill_data:
            self.skill_data[skill_name] = {}
        self.skill_data[skill_name][key] = value
    
    def register_skill(self, skill_name: str, skill_function):
        """Register a skill function"""
        self.skill_functions[skill_name] = skill_function
    
    def execute_skill(self, skill_name: str, **kwargs) -> Dict[str, Any]:
        """Execute a skill"""
        if skill_name not in self.skill_functions:
            return {"status": "failed", "message": f"Skill {skill_name} not found"}
        
        skill_func = self.skill_functions[skill_name]
        kwargs['robot'] = self.robot
        kwargs['skill_manager'] = self
        
        return skill_func(**kwargs)
    
    def reset_skill(self, skill_name: str):
        """Reset skill state"""
        self.skill_states.pop(skill_name, None)
        self.skill_data.pop(skill_name, None)
        self.skill_errors.pop(skill_name, None)
    
    def form_feedback(self, status: str = "processing", message: str = "none", progress: int = 100):
        """Form feedback dict (compatible with old interface)"""
        return {
            "status": status,
            "message": message,
            "progress": progress,
        }
