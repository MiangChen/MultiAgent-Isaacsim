# import yaml
#
# def read_yaml_file(file_path):
#     """
#     读取 YAML 文件并返回一个字典。
#
#     Args:
#         file_path (str): YAML 文件的路径。
#
#     Returns:
#         dict: 从 YAML 文件读取的数据，以字典形式表示。
#     """
#     try:
#         with open(file_path, 'r') as file:
#             data = yaml.safe_load(file)  # 使用 safe_load 避免潜在的安全问题
#         return data
#     except FileNotFoundError:
#         print(f"错误：文件 '{file_path}' 未找到。")
#         return None
#     except yaml.YAMLError as e:
#         print(f"错误：解析 YAML 文件时出错：{e}")
#         return None
#
#
# file_path = 'variables.yaml'  # 替换为你的 YAML 文件名
# data = read_yaml_file(file_path)
#
# # 访问特定的值：
# name_usr = data.get('NAME_USR')
# path_project = data.get('PATH_PROJECT')
# path_isaacsim_assets = data.get('PATH_ISAACSIM_ASSETS')

# ubuntu的用户名字
NAME_USR = 'ubuntu'
# 项目的地址
PATH_PROJECT = f'/home/{NAME_USR}/PycharmProjects/multiagent-isaacsim'
# ISAACSIM资产包的位置
PATH_ISAACSIM_ASSETS = f'/home/{NAME_USR}/isaacsim_assets/'
# conda环境的名字
NAME_CONDA_ENV = 'env_isaaclab'
# isaacsim的位置, 这里以通过pip安装在conda中的isaacsim为例子
PATH_ISAACSIM = f'/home/{NAME_USR}/anaconda3/envs/{NAME_CONDA_ENV}/lib/python3.10/site-packages/isaacsim'
# isaaclab的位置, 这里以本地项目中git clone下来的isaaclab为例子
PATH_ISAACLAB = f'/home/{NAME_USR}/PycharmProjects/multiagent-isaacim/IsaacLab'
