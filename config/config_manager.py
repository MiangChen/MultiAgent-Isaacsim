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
import argparse
import os
import pprint
from pathlib import Path
from typing import Any, List, Dict

# Third-party library imports
import yaml


class ConfigManager:
    """
    一个统一的配置管理器。
    负责整合来自 YAML 文件和命令行参数的配置，并计算派生路径。
    加载顺序: YAML 文件 -> 命令行参数。
    """

    def __init__(self):
        # 通过args传递的参数
        self._parser = self._create_argument_parser()
        self.args, self.args_unknown = self._parser.parse_known_args()
        # 通过文件读取的参数
        self.config: Dict[str, Any] = {}
        current_dir = os.path.dirname(os.path.abspath(__file__))
        config_path = os.path.join(current_dir, "../config/config_parameter.yaml")
        self.config_path = os.path.normpath(config_path)
        self.load()

    def _create_argument_parser(self) -> argparse.ArgumentParser:
        """
        创建并配置参数解析器，取代 argument_parser.py 的功能。
        """
        parser = argparse.ArgumentParser(description="Isaac Sim Multi-Agent Simulation")

        group = parser.add_mutually_exclusive_group()
        group.add_argument(
            "--namespace",
            type=str,
            default=None,
            help="Comma-separated list of namespaces for multi-UAV simulation. Overrides config.",
        )

        return parser

    def load(self):
        """
        加载、合并和处理所有配置。
        """
        with open(self.config_path, "r") as f:
            self.config = yaml.safe_load(f)

        # 将命令行参数覆盖到配置上
        cli_args_dict = vars(self.args)
        if cli_args_dict.get("namespace") is not None:
            namespaces_list = [
                ns.strip() for ns in cli_args_dict["namespace"].split(",") if ns.strip()
            ]
            self.config["namespace"] = namespaces_list

        # 计算派生路径和值
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
        """
        生成最终配置的摘要，用于日志记录。
        """
        summary = "=== Unified Configuration Summary ===\n"
        summary += f"Source config file: {self.args.config}\n"
        summary += "-----------------------------------\n"
        summary += pprint.pformat(self.config)
        summary += "\n==================================="
        return summary


config_manager = ConfigManager()
