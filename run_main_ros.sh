#!/bin/bash

# --- 1. 项目根目录 ---
# 这个脚本会自动检测其所在的目录为项目根目录，通常无需修改。
PROJECT_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# --- 2. Python 可执行文件路径 ---
PY_EXECUTABLE="/home/ubuntu/anaconda3/envs/env_isaaclab/bin/python"

# --- 3. 要运行的主 Python 脚本 ---
MAIN_PY_SCRIPT="main.py"

# --- 4. ROS 2 环境设置文件 ---
ROS2_SETUP_PATH="/opt/ros/humble/setup.bash"

# --- 5. (可选) 自定义 ROS 2 工作空间 ---
CUSTOM_WORKSPACE_SETUP_PATH="$PROJECT_ROOT/src/gsi_msgs/install/setup.bash"

# --- 6. 默认参数 ---
DEFAULT_ARGS=(
  "--config" "$PROJECT_ROOT/config/config_parameter.yaml"
)

# ==============================================================================
#                                  LOGIC
# ==============================================================================

# --- 1. 环境设置 ---
echo "[INFO] Sourcing ROS 2 environments..."

if [ -f "$ROS2_SETUP_PATH" ]; then
  source "$ROS2_SETUP_PATH"
else
  echo "[ERROR] ROS 2 setup file not found at: $ROS2_SETUP_PATH"
  exit 1
fi

if [ -n "$CUSTOM_WORKSPACE_SETUP_PATH" ] && [ -f "$CUSTOM_WORKSPACE_SETUP_PATH" ]; then
  source "$CUSTOM_WORKSPACE_SETUP_PATH"
  echo "[INFO] Sourced custom workspace: $CUSTOM_WORKSPACE_SETUP_PATH"
fi

# --- 2. 处理命令行参数 ---
# 将所有传递给此脚本的参数存储到 EXTRA_ARGS 数组中
EXTRA_ARGS=("$@")

# --- 3. 拼接最终的 Python 脚本路径 ---
MAIN_PY_FULL_PATH="$PROJECT_ROOT/$MAIN_PY_SCRIPT"

# --- 4. 启动主程序 ---
exec "$PY_EXECUTABLE" "$MAIN_PY_FULL_PATH" \
  "${DEFAULT_ARGS[@]}" \
  "${EXTRA_ARGS[@]}"