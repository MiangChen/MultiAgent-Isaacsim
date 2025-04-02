# -*- coding: utf-8 -*-

import os
import argparse
import sys

# 尝试导入 USD 模块。如果失败，说明 USD 环境未设置或未安装。
try:
    from pxr import Usd
except ImportError:
    print("Error: Failed to import 'pxr'.")
    print("Please ensure that the Pixar USD Python bindings are installed")
    print("and that the necessary environment variables (e.g., PYTHONPATH)")
    print("are set correctly.")
    sys.exit(1)

def convert_usdc_to_usda(input_usdc_path, output_usd_path):
    """
    Converts a binary USDC file (.usdc) to an ASCII USDA file (.usd or .usda).

    Args:
        input_usdc_path (str): Path to the input .usdc file.
        output_usd_path (str): Path for the output ASCII .usd or .usda file.

    Returns:
        bool: True if conversion was successful, False otherwise.
    """
    print(f"Attempting to convert '{input_usdc_path}' to '{output_usd_path}'...")

    # 1. 检查输入文件是否存在
    if not os.path.exists(input_usdc_path):
        print(f"Error: Input file not found: {input_usdc_path}")
        return False

    # 2. 检查输入文件扩展名 (可选，但推荐)
    if not input_usdc_path.lower().endswith('.usdc'):
        print(f"Warning: Input file '{input_usdc_path}' does not end with .usdc.")
        # 你可以选择在这里返回 False 或继续执行

    # 3. 检查输出文件扩展名 (可选，但推荐)
    if not output_usd_path.lower().endswith(('.usd', '.usda')):
        print(f"Warning: Output file '{output_usd_path}' should ideally end with .usd or .usda for clarity.")
        print("It will still be saved in ASCII format regardless.")

    # 4. 确保输出目录存在 (可选)
    output_dir = os.path.dirname(output_usd_path)
    if output_dir and not os.path.exists(output_dir):
        try:
            os.makedirs(output_dir)
            print(f"Created output directory: {output_dir}")
        except OSError as e:
            print(f"Error: Could not create output directory '{output_dir}': {e}")
            return False

    try:
        # 5. 打开 USDC Stage
        print(f"Opening stage: {input_usdc_path}...")
        stage = Usd.Stage.Open(input_usdc_path)

        if not stage:
            print(f"Error: Failed to open stage '{input_usdc_path}'. Is it a valid USD file?")
            return False

        # 6. 导出为 USDA (ASCII) 格式
        # Usd.Stage.Export() 会根据输出文件的扩展名自动推断格式。
        # .usd 或 .usda 会被导出为 ASCII 格式。
        print(f"Exporting stage to ASCII format: {output_usd_path}...")
        success = stage.Export(output_usd_path) # 关键步骤

        if not success:
            print(f"Error: Failed to export stage to '{output_usd_path}'")
            return False

        print("Conversion successful!")
        return True

    except Exception as e:
        # 捕获可能由 USD C++ 底层抛出的异常
        print(f"An unexpected error occurred during conversion: {e}")
        # 如果需要更具体的 USD 错误处理，可以导入并捕获 pxr.Tf.ErrorException
        # from pxr import Tf
        # except Tf.ErrorException as e:
        #     print(f"USD Error: {e}")
        return False

def main():
    """主函数，处理命令行参数并调用转换函数。"""
    parser = argparse.ArgumentParser(
        description="Convert a binary USDC file (.usdc) to an ASCII USDA file (.usd or .usda)."
    )
    parser.add_argument(
        "input_usdc",
        help="Path to the input .usdc file.",
        default="/home/ubuntu/Downloads/CrazyCock_Character_low_poly_animated/scene.usdc",
        nargs='?',  # 允许不提供该参数
    )
    parser.add_argument(
        "output_usd",
        help="Path for the output ASCII .usd or .usda file.",
        default="/home/ubuntu/Downloads/CrazyCock_Character_low_poly_animated/scene.usd",
        nargs='?',  # 允许不提供该参数
    )

    args = parser.parse_args()

    if not convert_usdc_to_usda(args.input_usdc, args.output_usd):
        sys.exit(1) # 以错误状态退出

if __name__ == "__main__":
    main()
