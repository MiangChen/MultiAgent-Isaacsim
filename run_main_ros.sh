#!/bin/bash

# --- 1. 项目根目录 ---
# 这个脚本会自动检测其所在的目录为项目根目录，通常无需修改。
PROJECT_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# --- 2. Python 可执行文件路径 ---
PY_EXECUTABLE="/home/ubuntu/anaconda3/envs/env_isaaclab/bin/python"

# --- 3. 要运行的主 Python 脚本 ---
MAIN_PY_SCRIPT="main.py"

# --- 4. 自定义 ROS 2 工作空间 ---
CUSTOM_WORKSPACE_SETUP_PATH=(
  "/opt/ros/humble/setup.bash"
  "/home/ubuntu/planner/install/setup.bash"
  "$PROJECT_ROOT/src/gsi_msgs/install/setup.bash"
)

# --- 6. 默认参数 ---
DEFAULT_ARGS=(
  "--config-path" "$PROJECT_ROOT/config/config_parameter.yaml"
)

# ==============================================================================
#                                  LOGIC
# ==============================================================================

# --- 1. 环境设置 ---
echo "[INFO] Sourcing custom workspaces..."
for ws_path in "${CUSTOM_WORKSPACE_SETUP_PATHS[@]}"; do
  if [ -f "$ws_path" ]; then
    source "$ws_path"
    echo "  -> Sourced: $ws_path"
  else
    echo "  -> [WARNING] Custom workspace setup file not found: $ws_path"
  fi
done

# --- 2. 处理命令行参数 ---
# 将所有传递给此脚本的参数存储到 EXTRA_ARGS 数组中
EXTRA_ARGS=("$@")

# --- 3. 拼接最终的 Python 脚本路径 ---
MAIN_PY_FULL_PATH="$PROJECT_ROOT/$MAIN_PY_SCRIPT"

# --- 4. 启动主程序 ---
exec "$PY_EXECUTABLE" "$MAIN_PY_FULL_PATH" \
  "${DEFAULT_ARGS[@]}" \
  "${EXTRA_ARGS[@]}"