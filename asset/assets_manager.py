"""
import os
import json
from pathlib import Path

# 指定要扫描的目录
folder_to_scan = "C:/isaacsim_assets/Assets/Isaac/4.5/Isaac"
output_json = "usd_files.json"

def collect_usd_files(base_folder):
    usd_files = {}
    base_path = Path(base_folder).resolve()

    for dirpath, dirnames, filenames in os.walk(base_path):
        # 如果当前目录包含 .thumbs 子文件夹
        if ".thumbs" in dirnames:
            current_path = Path(dirpath)
            for file in current_path.glob("*.usd"):
                # 获取相对路径
                relative_path = file.relative_to(base_path)
                usd_files[file.name] = str(relative_path).replace("\\", "/")  # 统一用斜杠

    return usd_files

# 执行收集
result = collect_usd_files(folder_to_scan)

# 写入 JSON 文件
with open(output_json, "w", encoding="utf-8") as f:
    json.dump(result, f, ensure_ascii=False, indent=2)

print(f"已找到 {len(result)} 个 .usd 文件，结果已写入：{output_json}")
"""
# 以上代码是提取官方资产库资产，储存到usd_files.json，已进行筛选删除，无需再次执行。
# 自有资产请存储至 C:/isaacsim_assets/Assets/Isaac/4.5/Isaac/Assets/User 中，并在可以直接用的usd文件名里加上available后缀。
# 以下代码提取自有资产库的usd文件，储存到user_usd_files.json。

import os
import json
from pathlib import Path

# 目录
folder_to_scan = "/home/ubuntu/isaacsim_assets/Assets/Isaac/4.5/Isaac/User"
output_json = "user_usd_files.json"

def collect_available_usd_files(base_folder):
    usd_files = {}
    base_path = Path(base_folder).resolve()

    for dirpath, dirnames, filenames in os.walk(base_path):
        current_path = Path(dirpath)
        for file in current_path.glob("*.usd"):
            if "available" in file.name:
                relative_path = file.relative_to(base_path)
                usd_files[file.name] = str(relative_path).replace("\\", "/")  # 用斜杠

    return usd_files

# 执行收集
result = collect_available_usd_files(folder_to_scan)

# 写入 JSON 文件
with open(output_json, "w", encoding="utf-8") as f:
    json.dump(result, f, ensure_ascii=False, indent=2)

print(f"已找到 {len(result)} 个包含 'available' 的 .usd 文件，结果已写入：{output_json}")
