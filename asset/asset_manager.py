# =============================================================================
# Asset Manager Module - Asset Discovery and Management
# =============================================================================
#
# This module provides functionality for discovering, cataloging, and managing
# USD assets within the Isaac Sim environment, including automated asset
# scanning and metadata generation.
#
# =============================================================================

# Standard library imports
import json
import os
from pathlib import Path

# Third-party library imports
import yaml


def process_asset_subdirectory(
    scan_path: Path, base_output_dir: Path, is_user_folder: bool = False
):
    output_filename = base_output_dir / f"{scan_path.name}_assets.json"
    print(
        f"--- 正在处理子目录: '{scan_path.name}' -> 输出到 '{output_filename.name}' ---"
    )

    usd_files = {}

    for file_path in scan_path.rglob("*.usd"):
        if is_user_folder and "available" not in file_path.name:
            continue

        path_to_store = str(file_path).replace("\\", "/")
        usd_files[file_path.name] = path_to_store

    if usd_files:
        with open(output_filename, "w", encoding="utf-8") as f:
            json.dump(usd_files, f, ensure_ascii=False, indent=2)
        print(f"处理完成！已找到 {len(usd_files)} 个文件。\n")
    else:
        print("未找到匹配的 .usd 文件。\n")


def main():
    project_root = Path(__file__).parent.parent.resolve()
    config_file_path = project_root / "config" / "config_parameter.yaml"
    with open(config_file_path, "r", encoding="utf-8") as f:
        config_data = yaml.safe_load(f)
    path_asset = config_data.get("path_asset")

    if not path_asset or not os.path.isdir(path_asset):
        print(f"path asset:{path_asset} is not valid")
    output_dir = Path(".")
    output_dir.mkdir(exist_ok=True)
    for entry in Path(path_asset).iterdir():
        if entry.is_dir():
            is_user_dir = entry.name == "User"
            process_asset_subdirectory(entry, output_dir, is_user_folder=is_user_dir)


if __name__ == "__main__":
    main()
