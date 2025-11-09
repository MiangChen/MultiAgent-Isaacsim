"""Application Layer - Skills and high-level logic"""

from application.skill_manager import SkillManager

# Import skills to trigger @register decorators
# 导入技能模块以触发装饰器注册
import application.skills

__all__ = ['SkillManager']
