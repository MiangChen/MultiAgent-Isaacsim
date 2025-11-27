from typing import Dict, Any, Callable, Optional, List
from pathlib import Path
import yaml


class SkillManager:
    """
    Skill Manager - 技能管理器

    管理机器人的技能注册和执行。使用类变量存储全局技能注册表，
    每个机器人实例可以选择性地使用全局技能。

    使用装饰器注册技能：
        @SkillManager.register()
        def my_skill(robot, **kwargs):
            pass

    创建管理器：
        skill_manager = SkillManager(robot, auto_register=True)
        result = skill_manager.execute_skill('navigate_to', goal_pos=[10, 20, 0])
    """

    # 类变量：全局技能注册表
    _global_skills: Dict[str, Callable] = {}
    _skill_config: Dict[str, Any] = None

    def __init__(self, robot, auto_register: bool = True, robot_type: str = None):
        """
        初始化技能管理器

        Args:
            robot: 机器人实例
            auto_register: 是否自动注册所有全局技能
            robot_type: 机器人类型 (e.g., 'jetbot', 'h1', 'cf2x', 'drone_autel')
                       用于过滤可用技能
        """
        self.robot = robot
        self.robot_type = robot_type
        self.skills = {}
        self.skill_states = {}
        self.skill_data = {}
        self.skill_errors = {}

        # Simulation time (updated by SkillROSInterface)
        self.sim_time: float = 0.0

        # Load skill config if not loaded
        if SkillManager._skill_config is None:
            SkillManager._load_skill_config()

        # Auto-register skills based on robot type
        if auto_register:
            self.register_skills_for_robot_type()

    # ============================================================================
    # 装饰器：注册技能到全局注册表
    # ============================================================================

    @classmethod
    def register(cls, skill_name: Optional[str] = None):
        """
        装饰器：注册技能函数到全局注册表

        Args:
            skill_name: 技能名称（可选，默认使用函数名）

        Examples:
            @SkillManager.register()
            def navigate_to(robot, **kwargs):
                pass

            @SkillManager.register("custom_name")
            def my_skill(robot, **kwargs):
                pass
        """

        def decorator(skill_func: Callable) -> Callable:
            name = skill_name or skill_func.__name__
            cls._global_skills[name] = skill_func
            return skill_func

        # Support both @register and @register()
        if callable(skill_name):
            skill_func = skill_name
            cls._global_skills[skill_func.__name__] = skill_func
            return skill_func

        return decorator

    @classmethod
    def get_global_skills(cls) -> Dict[str, Callable]:
        """获取所有全局注册的技能"""
        return cls._global_skills.copy()

    @classmethod
    def list_global_skills(cls) -> list:
        """列出所有全局注册的技能名称"""
        return list(cls._global_skills.keys())

    @classmethod
    def _load_skill_config(cls) -> None:
        """加载技能配置文件"""
        config_path = Path(__file__).parent.parent / "config" / "skill_config.yaml"
        if config_path.exists():
            with open(config_path, "r") as f:
                cls._skill_config = yaml.safe_load(f)
        else:
            cls._skill_config = {}

    @classmethod
    def get_skills_for_robot_type(cls, robot_type: str) -> List[str]:
        """
        获取指定机器人类型可用的技能列表

        Args:
            robot_type: 机器人类型 (e.g., 'jetbot', 'h1', 'cf2x', 'drone_autel')

        Returns:
            该机器人类型可用的技能名称列表
        """
        if cls._skill_config is None:
            cls._load_skill_config()

        available_skills = set()

        for group_name, group_config in cls._skill_config.items():
            if not isinstance(group_config, dict):
                continue
            robot_types = group_config.get("robot_types", [])
            skills = group_config.get("skills", [])

            if robot_type in robot_types:
                available_skills.update(skills)

        return list(available_skills)

    # ============================================================================
    # 实例方法：管理单个机器人的技能
    # ============================================================================

    def register_skill(self, skill_name: str, skill_function: Callable):
        """注册技能到当前机器人"""
        self.skills[skill_name] = skill_function

    def register_all_global_skills(self):
        """从全局注册表注册所有技能（不考虑机器人类型）"""
        for skill_name, skill_func in self._global_skills.items():
            self.register_skill(skill_name, skill_func)

    def register_skills_for_robot_type(self):
        """
        根据机器人类型注册可用技能

        如果未指定 robot_type，则注册所有全局技能
        """
        if self.robot_type is None:
            # 未指定类型，注册所有技能
            self.register_all_global_skills()
            return

        # 获取该机器人类型可用的技能
        available_skills = self.get_skills_for_robot_type(self.robot_type)

        # 只注册配置中允许的技能
        for skill_name, skill_func in self._global_skills.items():
            if skill_name in available_skills:
                self.register_skill(skill_name, skill_func)

    def execute_skill(self, skill_name: str, **kwargs) -> Dict[str, Any]:
        """
        执行技能

        Args:
            skill_name: 技能名称
            **kwargs: 技能参数

        Returns:
            技能执行结果

        Note:
            - 技能使用状态机模式，每次调用会根据当前状态执行相应逻辑
            - 如果需要重新执行已完成的技能，请先调用 reset_skill()
        """
        if skill_name not in self.skills:
            return {"status": "failed", "message": f"Skill {skill_name} not found"}

        kwargs["robot"] = self.robot
        kwargs["skill_manager"] = self
        return self.skills[skill_name](**kwargs)

    def list_skills(self) -> list:
        """列出当前机器人可用的技能"""
        return list(self.skills.keys())

    # ============================================================================
    # 技能状态管理
    # ============================================================================

    def get_skill_state(self, skill_name: str):
        """获取技能状态"""
        return self.skill_states.get(skill_name)

    def set_skill_state(self, skill_name: str, state: str):
        """设置技能状态"""
        self.skill_states[skill_name] = state

    def get_skill_data(self, skill_name: str, key: str, default=None):
        """获取技能数据"""
        if skill_name not in self.skill_data:
            self.skill_data[skill_name] = {}
        return self.skill_data[skill_name].get(key, default)

    def set_skill_data(self, skill_name: str, key: str, value):
        """设置技能数据"""
        if skill_name not in self.skill_data:
            self.skill_data[skill_name] = {}
        self.skill_data[skill_name][key] = value

    def form_feedback(
        self, status: str = "processing", message: str = "", progress: int = 100
    ):
        """构造反馈消息"""
        return {"status": status, "message": message, "progress": progress}

    def reset_skill(self, skill_name: str):
        """
        重置技能状态和数据

        Args:
            skill_name: 技能名称
        """
        if skill_name in self.skill_states:
            del self.skill_states[skill_name]
        if skill_name in self.skill_data:
            del self.skill_data[skill_name]
        if skill_name in self.skill_errors:
            del self.skill_errors[skill_name]

    def reset_all_skills(self):
        """重置所有技能状态"""
        self.skill_states.clear()
        self.skill_data.clear()
        self.skill_errors.clear()
