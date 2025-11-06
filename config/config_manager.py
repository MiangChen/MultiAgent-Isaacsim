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
        self.config_path = Path(__file__).parent / "config_parameter.yaml"
        self.load()

    def load(self) -> None:
        with open(self.config_path, "r") as f:
            self.config = yaml.safe_load(f)

        self._derive_paths()
        return

    def get(self, key: str) -> Any:
        """
        安全地从最终配置中获取一个值。支持点状访问 (e.g., 'world.name')。
        """
        keys = key.split(".")
        value = self.config
        for k in keys:
            value = value[k]
        return value

    def _derive_paths(self) -> None:
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

        # 动态搜索 asset 目录下所有 JSON 文件
        asset_dir = project_root / "asset"
        world_usd_path = None

        for json_file in asset_dir.glob("*.json"):
            try:
                with open(json_file, "r") as f:
                    world_name_dic = yaml.safe_load(f)
                if isinstance(world_name_dic, dict) and world_name in world_name_dic:
                    world_usd_path = world_name_dic[world_name]
                    break
            except Exception:
                continue

        if world_usd_path is None:
            raise ValueError(
                f"在 asset 目录的所有 JSON 文件中都找不到名为 '{world_name}' 的场景"
            )
        self.config["world_usd_path"] = world_usd_path

        return

    def get_summary(self) -> str:
        summary = "=== Configuration Summary ===\n"
        summary += f"Source config file: {self.config_path}\n"
        summary += "-----------------------------------\n"
        summary += pprint.pformat(self.config)
        summary += "\n==================================="
        return summary


config_manager = ConfigManager()
