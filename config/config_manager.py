# =============================================================================
# Config Manager Module - Unified Configuration Management
# =============================================================================
#
# This module provides a centralized configuration management system that
# integrates YAML file configurations with command-line arguments for
# flexible application configuration.
#
# =============================================================================

# Standard library imports
import os
import pprint
from pathlib import Path
from typing import Any, List, Dict

# Third-party library imports
import yaml


class ConfigManager:

    def __init__(self):
        self.config: Dict[str, Any] = {}
        current_dir = os.path.dirname(os.path.abspath(__file__))
        config_path = os.path.join(current_dir, "../config/config_parameter.yaml")
        self.config_path = os.path.normpath(config_path)
        self.load()

    def load(self):
        with open(self.config_path, "r") as f:
            self.config = yaml.safe_load(f)

        self._derive_paths()
        return self

    def get(self, key: str) -> Any:
        """
        安全地从最终配置中获取一个值。支持点状访问 (e.g., 'world.name')。
        """
        keys = key.split(".")
        value = self.config
        for k in keys:
            value = value[k]
        return value

    def _derive_paths(self):
        """
        计算所有派生路径
        """
        # 关键: 自动计算项目根目录，消除硬编码
        # config/config_manager.py -> config -> project_root
        project_root = Path(__file__).parent.parent.absolute()
        self.config["project_root"] = str(project_root)

        # 获取场景名称
        world_name = self.get("world.name")
        if not world_name:
            raise ValueError("配置中必须提供 'world.name'")

        # 获取场景 USD 文件的绝对路径
        user_usd_files_json_path = project_root / "asset" / "User_assets.json"
        if not user_usd_files_json_path.exists():
            raise FileNotFoundError(f"找不到场景定义文件: {user_usd_files_json_path}")

        with open(user_usd_files_json_path, "r") as f:
            world_name_dic = yaml.safe_load(f)

        if world_name not in world_name_dic:
            raise ValueError(
                f"在 {user_usd_files_json_path} 中找不到名为 '{world_name}' 的场景"
            )
        self.config["world_usd_path"] = world_name_dic[world_name]

    def get_summary(self) -> str:
        summary = "=== Configuration Summary ===\n"
        summary += f"Source config file: {self.config_path}\n"
        summary += "-----------------------------------\n"
        summary += pprint.pformat(self.config)
        summary += "\n==================================="
        return summary


config_manager = ConfigManager()
