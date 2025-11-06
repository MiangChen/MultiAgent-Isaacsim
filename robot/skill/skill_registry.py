# =============================================================================
# Skill Registry Module - Skill Registration and Management System
# =============================================================================
#
# This module provides a centralized skill registration system that allows
# different robot types to register their supported skills using decorators.
#
# =============================================================================

# Standard library imports
from typing import Dict, List, Callable, Optional

# Local project imports
from log.log_manager import LogManager

logger = LogManager.get_logger(__name__)


class SkillRegistry:
    """
    技能注册系统
    
    用于管理不同机器人类型支持的技能，通过装饰器方式注册技能函数
    支持按需导入技能模块
    """
    
    _skills: Dict[str, Dict[str, Callable]] = {}
    
    # 技能模块路径映射
    _skill_module_mapping = {
        "navigate_to_skill": "robot.skill.base.navigation.navigate_to",
        "detect_skill": "robot.skill.base.detection.detect", 
        "take_off": "robot.skill.drone.take_off.take_off",
        "pick_up_skill": "robot.skill.base.manipulation.pick_up",
        "put_down_skill": "robot.skill.base.manipulation.put_down",
        "take_photo": "robot.skill.base.take_photo",
        "object_detection_skill": "robot.skill.base.object_detection",
        "explore_skill": "robot.skill.base.exploration.explore",
        "track_skill": "robot.skill.base.track",
    }
    
    @classmethod
    def register(cls, robot_types: List[str] = None):
        """
        装饰器：为指定的机器人类型注册技能
        
        Args:
            robot_types: 支持该技能的机器人类型列表，如果为None则自动推断
            
        Returns:
            装饰器函数
        """
        def decorator(skill_func: Callable):
            skill_name = skill_func.__name__
            
            # 如果没有指定机器人类型，自动推断
            if robot_types is None:
                inferred_types = cls._infer_robot_types_from_skill(skill_func)
            else:
                inferred_types = robot_types
            
            for robot_type in inferred_types:
                if robot_type not in cls._skills:
                    cls._skills[robot_type] = {}
                
                cls._skills[robot_type][skill_name] = skill_func
                logger.debug(f"Registered skill '{skill_name}' for robot type '{robot_type}'")
            
            return skill_func
        return decorator
    
    @classmethod
    def _infer_robot_types_from_skill(cls, skill_func: Callable) -> List[str]:
        """
        根据技能函数自动推断支持的机器人类型
        
        Args:
            skill_func: 技能函数
            
        Returns:
            推断出的机器人类型列表
        """
        skill_name = skill_func.__name__
        
        # 优先从配置文件推断
        config_types = cls._infer_robot_types_from_config(skill_name)
        if config_types:
            logger.debug(f"Inferred robot types from config for '{skill_name}': {config_types}")
            return config_types
        
        # 如果配置文件中没有，根据文件路径推断
        import inspect
        import os
        
        try:
            # 获取技能函数的文件路径
            skill_file = inspect.getfile(skill_func)
            skill_path = os.path.normpath(skill_file)
            
            # 根据路径推断机器人类型
            if '/drone/' in skill_path:
                # 无人机专用技能
                inferred_types = ['cf2x']
            elif '/base/' in skill_path:
                # 基础技能，大多数机器人都支持
                inferred_types = ['jetbot', 'g1', 'h1', 'cf2x']
            elif '/manipulation/' in skill_path:
                # 操作技能，只有有手臂的机器人支持
                inferred_types = ['g1', 'h1']
            else:
                # 默认支持所有机器人类型
                inferred_types = ['jetbot', 'g1', 'h1', 'cf2x']
            
            logger.debug(f"Inferred robot types from path for '{skill_name}': {inferred_types}")
            return inferred_types
                
        except Exception as e:
            logger.warning(f"Failed to infer robot types for {skill_func.__name__}: {e}")
            # 默认支持所有机器人类型
            return ['jetbot', 'g1', 'h1', 'cf2x']
    
    @classmethod
    def get_skill(cls, robot_type: str, skill_name: str) -> Optional[Callable]:
        """
        获取指定机器人类型的技能函数
        
        Args:
            robot_type: 机器人类型
            skill_name: 技能名称
            
        Returns:
            技能函数，如果不存在则返回None
        """
        return cls._skills.get(robot_type, {}).get(skill_name)
    

    
    @classmethod
    def get_skills_for_robot(cls, robot_type: str) -> Dict[str, Callable]:
        """
        获取指定机器人类型的所有技能
        
        Args:
            robot_type: 机器人类型
            
        Returns:
            技能字典 {skill_name: skill_function}
        """
        return cls._skills.get(robot_type, {}).copy()
    
    @classmethod
    def get_all_robot_types(cls) -> List[str]:
        """
        获取所有已注册技能的机器人类型
        
        Returns:
            机器人类型列表
        """
        return list(cls._skills.keys())
    
    @classmethod
    def get_skill_names_for_robot(cls, robot_type: str) -> List[str]:
        """
        获取指定机器人类型支持的所有技能名称
        
        Args:
            robot_type: 机器人类型
            
        Returns:
            技能名称列表
        """
        return list(cls._skills.get(robot_type, {}).keys())
    
    @classmethod
    def is_skill_supported(cls, robot_type: str, skill_name: str) -> bool:
        """
        检查指定机器人类型是否支持某个技能
        
        Args:
            robot_type: 机器人类型
            skill_name: 技能名称
            
        Returns:
            是否支持该技能
        """
        return skill_name in cls._skills.get(robot_type, {})
    
    @classmethod
    def get_registry_info(cls) -> Dict[str, List[str]]:
        """
        获取注册表信息概览
        
        Returns:
            {robot_type: [skill_names]} 格式的字典
        """
        return {
            robot_type: list(skills.keys()) 
            for robot_type, skills in cls._skills.items()
        }
    
    @classmethod
    def register_skill_module(cls, skill_name: str, module_path: str):
        """
        注册技能模块路径
        
        Args:
            skill_name: 技能名称
            module_path: 模块路径
        """
        cls._skill_module_mapping[skill_name] = module_path
        logger.debug(f"Registered skill module mapping: {skill_name} -> {module_path}")
    
    @classmethod
    def get_skill_module_mapping(cls) -> Dict[str, str]:
        """获取技能模块路径映射"""
        return cls._skill_module_mapping.copy()
    
    @classmethod
    def auto_register(cls):
        """
        自动注册装饰器：根据技能位置自动推断支持的机器人类型
        
        Returns:
            装饰器函数
        """
        return cls.register(robot_types=None)
    
    @classmethod
    def load_skill_config(cls, config_path: str = None):
        """
        从配置文件加载技能映射关系
        
        Args:
            config_path: 配置文件路径，默认使用内置配置
        """
        import yaml
        import os
        
        if config_path is None:
            # 使用默认配置文件路径
            current_dir = os.path.dirname(__file__)
            config_path = os.path.join(current_dir, 'skill_config.yaml')
        
        try:
            with open(config_path, 'r', encoding='utf-8') as f:
                config = yaml.safe_load(f)
            
            cls._skill_config = config
            logger.info(f"Loaded skill configuration from {config_path}")
            
        except Exception as e:
            logger.error(f"Failed to load skill config from {config_path}: {e}")
            cls._skill_config = {}
    
    @classmethod
    def _infer_robot_types_from_config(cls, skill_name: str) -> List[str]:
        """
        根据配置文件推断技能支持的机器人类型
        
        Args:
            skill_name: 技能名称
            
        Returns:
            支持的机器人类型列表
        """
        if not hasattr(cls, '_skill_config'):
            cls.load_skill_config()
        
        # 在配置中查找技能
        for category, info in cls._skill_config.items():
            if 'skills' in info and skill_name in info['skills']:
                return info['robot_types']
        
        # 如果配置中没找到，使用路径推断
        return None
    
    @classmethod
    def clear_registry(cls):
        """清空注册表（主要用于测试）"""
        cls._skills.clear()
        logger.info("Skill registry cleared")
