import json
import yaml
import platform
import os

_current_dir = os.path.dirname(os.path.abspath(__file__))

def load_config(file_path:str = None) -> dict:
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


file_path = os.path.join(_current_dir, './env_cfg.yaml')
ASSET_PATH = load_config(file_path)['asset_path']
PATH_ISAACSIM_ASSETS = ASSET_PATH
# 项目的位置
PATH_PROJECT = f'/home/ubuntu/multiagent-isaacsimROS/src/multiagent_isaacsim/multiagent_isaacsim'

WORLD_NAME = load_config(file_path)['world']['name']  # 获取需要的场景名字


# 获取场景的绝对路径
file_path = os.path.join(_current_dir, '../asset/user_usd_files.json')
with open(file_path, 'r', encoding='utf-8') as file:
    world_name_dic = json.load(file)

WORLD_USD_PATH = world_name_dic[WORLD_NAME]
print(WORLD_USD_PATH)
