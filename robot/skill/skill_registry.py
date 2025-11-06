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
import os
import yaml

# Local project imports
from log.log_manager import LogManager

logger = LogManager.get_logger(__name__)


class SkillRegistry:
    """
    技能注册系统

    用于管理不同机器人类型支持的技能，通过装饰器方式注册技能函数
    """

    _skills: Dict[str, Dict[str, Callable]] = {}

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
            robot_types = cls._infer_robot_types_from_config(skill_name)
            for robot_type in robot_types:
                if robot_type not in cls._skills:
                    cls._skills[robot_type] = {}

                cls._skills[robot_type][skill_name] = skill_func
                logger.debug(
                    f"Registered skill '{skill_name}' for robot type '{robot_type}'"
                )

            return skill_func

        return decorator

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
    def _infer_robot_types_from_config(cls, skill_name: str) -> List[str]:
        """
        根据配置文件推断技能支持的机器人类型

        Args:
            skill_name: 技能名称

        Returns:
            支持的机器人类型列表
        """
        if not hasattr(cls, "_skill_config"):
            current_dir = os.path.dirname(__file__)
            config_path = os.path.join(current_dir, "skill_config.yaml")

            try:
                with open(config_path, "r", encoding="utf-8") as f:
                    config = yaml.safe_load(f)

                cls._skill_config = config
                logger.info(f"Loaded skill configuration from {config_path}")

            except Exception as e:
                raise f"Failed to load skill config from {config_path}: {repr(e)}"

        for skill_category, info in cls._skill_config.items():
            if skill_name in info.get("skills"):
                return info["robot_types"]

        raise ValueError(f"Skill {skill_name} not found in skill config")
