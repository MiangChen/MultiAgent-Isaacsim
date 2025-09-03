import yaml
from pathlib import Path
from typing import Any, Dict
import pprint


class ConfigManager:
    """
    一个以 YAML 文件为单一可信来源的统一配置管理器。
    负责加载、处理、验证并提供所有应用程序配置。
    """

    def __init__(self, config_path: str):
        """
        初始化管理器并加载所有配置。

        Args:
            config_path (str): 主配置文件的路径。
        """
        self.config_path = Path(config_path)
        self.config: Dict[str, Any] = self._load_from_yaml()

        # 加载后立即执行后处理
        self._derive_paths()
        self._validate_config()

    def _load_from_yaml(self) -> Dict[str, Any]:
        """从 YAML 文件加载基础配置。"""
        if not self.config_path.exists():
            raise FileNotFoundError(
                f"FATAL: 主配置文件未找到: '{self.config_path}'"
            )

        print(f"--- 正在从 '{self.config_path}' 加载配置 ---")
        with open(self.config_path, 'r') as f:
            return yaml.safe_load(f)

    def _derive_paths(self):
        """
        计算所有派生路径值，完全取代 variables.py 的功能。
        """
        # 项目根目录，基于此文件位置计算
        project_root = Path(__file__).parent.parent.absolute()
        self.config['project_root'] = str(project_root)

        # 处理 world_usd_path
        world_name = self.get('simulation.world.name')
        if not world_name:
            raise ValueError("配置中必须提供 'simulation.world.name'")

        # user_usd_files.json 的路径是相对于项目根目录的
        user_usd_files_path = project_root / 'asset' / 'user_usd_files.json'
        if not user_usd_files_path.exists():
            raise FileNotFoundError(f"找不到场景定义文件: {user_usd_files_path}")

        with open(user_usd_files_path, 'r') as f:
            world_name_dic = yaml.safe_load(f)

        if world_name not in world_name_dic:
            raise ValueError(f"在 {user_usd_files_path} 中找不到名为 '{world_name}' 的场景")

        # 将计算出的路径存回配置中
        self.config['simulation']['world']['usd_path'] = world_name_dic[world_name]

    def _validate_config(self):
        """
        统一对加载完成的配置进行校验。
        """
        web_port = self.get('web.port', 8080)
        if not (1 <= web_port <= 65535):
            raise ValueError(f"Web 端口必须在 1 到 65535 之间, 得到 {web_port}")

        rate = self.get('web.data_collection_rate', 5.0)
        if rate <= 0:
            raise ValueError(f"数据收集频率必须为正数, 得到 {rate}")

        # ... 在这里添加所有其他必要的校验 ...
        print("--- 配置校验通过 ---")

    def get(self, key: str, default: Any = None) -> Any:
        """
        从最终配置中获取一个值。支持点状访问。
        例如: config_manager.get('web.port')
        """
        keys = key.split('.')
        value = self.config
        try:
            for k in keys:
                value = value[k]
            return value
        except (KeyError, TypeError):
            return default

    def get_summary(self) -> str:
        """生成最终配置的摘要，用于日志记录。"""
        summary = "=== Final Resolved Configuration Summary ===\n"
        summary += f"Source config file: {self.config_path}\n"
        summary += "----------------------------------------\n"
        summary += pprint.pformat(self.config)
        summary += "\n========================================"
        return summary