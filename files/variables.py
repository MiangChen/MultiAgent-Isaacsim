import yaml
import platform
import os

def load_config(file_path:str = None) -> dict:

    _current_dir = os.path.dirname(os.path.abspath(__file__))
    # absolute path
    file_path = os.path.join(_current_dir, file_path)
    """从YAML文件加载配置。"""
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            return yaml.safe_load(f)
    except FileNotFoundError:
        print(f"错误: 配置文件 '{file_path}' 未找到。")
        return None
    except yaml.YAMLError as e:
        print(f"错误: 解析YAML文件时出错: {e}")
        return None


ASSET_PATH = load_config('./usd_path.yaml')['asset_path']
