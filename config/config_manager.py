import argparse
from pathlib import Path
import pprint
from typing import Any, List, Dict
import yaml


class ConfigManager:
    """
    一个统一的配置管理器。
    负责整合来自 YAML 文件和命令行参数的配置，并计算派生路径。
    加载顺序: YAML 文件 -> 命令行参数。
    """

    def __init__(self):
        self.config: Dict[str, Any] = {}
        self._parser = self._create_argument_parser()
        self.args = None

        self.load()

    def load(self, args: List[str] = None):
        """
        加载、合并和处理所有配置。
        """
        # 1. 解析命令行参数
        self.args = self._parser.parse_args(args)

        # 2. 从 YAML 文件加载基础配置
        config_file = self.args.config
        if config_file and Path(config_file).exists():
            with open(config_file, "r") as f:
                self.config = yaml.safe_load(f)
        else:
            raise FileNotFoundError(
                f"指定的配置文件 '{config_file}' 未找到。请检查路径。"
            )

        # 3. 将命令行参数覆盖到配置上
        cli_args_dict = vars(self.args)
        if cli_args_dict.get("ros") is not None:
            self.config.setdefault("ros", {})["enable"] = cli_args_dict["ros"]

        # 4. 计算派生路径和值 (取代 variables.py 的功能)
        self._derive_paths()

        return self

    def get(self, key: str, default: Any = None) -> Any:
        """
        安全地从最终配置中获取一个值。支持点状访问 (e.g., 'world.name')。
        """
        keys = key.split(".")
        value = self.config
        try:
            for k in keys:
                value = value[k]
            return value
        except (KeyError, TypeError):
            return default

    def _derive_paths(self):
        """
        计算所有派生路径，取代 variables.py 的功能。
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
        user_usd_files_json_path = project_root / "asset" / "user_usd_files.json"
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

    def _create_argument_parser(self) -> argparse.ArgumentParser:
        """
        创建并配置参数解析器，取代 argument_parser.py 的功能。
        """
        parser = argparse.ArgumentParser(description="Isaac Sim Multi-Agent Simulation")

        parser.add_argument(
            "--config",
            type=str,
            default="./config/sim_cfg.yaml",
            help="Path to the main YAML configuration file.",
        )

        parser.add_argument(
            "--ros",
            type=lambda x: str(x).lower() in ["true", "1", "yes"],
            metavar="{true,false}",
            help="Override ROS2 integration setting from the config file.",
        )

        parser.add_argument(
            "--enable",
            type=str,
            action="append",
            help="Enable a feature. Can be used multiple times.",
        )
        return parser
