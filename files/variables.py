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

NAME_USR = 'ubuntu'
PATH_PROJECT = '/home/ubuntu/PycharmProjects/multiagent-isaacsim'
PATH_ISAACSIM_ASSETS = '/home/ubuntu/isaacsim_assets/'