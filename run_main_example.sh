#!/bin/bash
# sudo apt install ros-humble-rmw-fastrtps-cpp
PATH_PYTHON="path to python, example: /home/cleanuser/anaconda3/envs/env_isaacsim"
PATH_GSI_MSG="path to gsi_msgs, example: /home/cleanuser/PycharmProjects/isaacsim-gsi/src/gsi_msgs"

# --- 1. 项目根目录 ---
PROJECT_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# --- 2. Python 可执行文件路径 ---
PY_EXECUTABLE="$PATH_PYTHON/bin/python"

# --- 路径检查 / Path Validation ---
if [ ! -f "$PY_EXECUTABLE" ]; then
    echo "========================================"
    echo "错误：Python可执行文件不存在 / ERROR: Python executable not found"
    echo "期望路径 / Expected path: $PY_EXECUTABLE"
    echo "请检查 PATH_PYTHON 配置是否正确"
    echo "Please check if PATH_PYTHON is configured correctly"
    echo "========================================"
    exit 1
fi

if [ ! -d "$PATH_GSI_MSG" ]; then
    echo "========================================"
    echo "错误：GSI消息路径不存在 / ERROR: GSI messages path does not exist"
    echo "当前配置 / Current config: PATH_GSI_MSG=\"$PATH_GSI_MSG\""
    echo "请修改脚本中的 PATH_GSI_MSG 变量为正确的路径"
    echo "Please modify the PATH_GSI_MSG variable in the script to the correct path"
    echo "========================================"
    exit 1
fi

echo "[INFO] 路径验证通过 / Path validation passed"

# --- 3. 要运行的主 Python 脚本 ---
MAIN_PY_SCRIPT="main_example.py"

# --- 4. 自定义 ROS 2 工作空间 ---
WORKSPACE_SETUP_PATH=(
  "/opt/ros/humble/setup.bash"
  "$PATH_GSI_MSG/install/setup.bash"
)

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DISTRO=humble
export PYTHONPATH="$PATH_GSI_MSG/..:/opt/ros/humble/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages:$PYTHONPATH"
export LD_LIBRARY_PATH="/opt/ros/humble/lib:$PATH_PYTHON/lib/python3.10/site-packages/isaacsim/exts/isaacsim.ros2.bridge/humble/lib:$PATH_GSI_MSG/install/scene_msgs/lib:$PATH_GSI_MSG/install/plan_msgs/lib:$LD_LIBRARY_PATH"

SYSTEM_LIBSTDC=$(find /usr/lib/x86_64-linux-gnu/ -name "libstdc++.so.6" | head -n 1)
SYSTEM_LIBSPDLOG=$(find /usr/lib/x86_64-linux-gnu/ -name "libspdlog.so.*" | head -n 1)

if [ -f "$SYSTEM_LIBSTDC" ] && [ -f "$SYSTEM_LIBSPDLOG" ]; then
    export LD_PRELOAD="$SYSTEM_LIBSTDC:$SYSTEM_LIBSPDLOG"
    echo "[INFO] 正在预加载系统库以解决冲突: $LD_PRELOAD"
else
    echo "[WARNING] 未能找到系统的 libstdc++.so.6 或 libspdlog.so.*，可能会出现库冲突。"
fi

#export FASTRTPS_DEFAULT_PROFILES_FILE=~/.ros/fastdds.xml

# ==============================================================================
#                                  LOGIC
# ==============================================================================

# --- 1. 环境设置 ---
echo "[INFO] Sourcing custom workspaces..."
for ws_path in "${WORKSPACE_SETUP_PATH[@]}"; do
  if [ -f "$ws_path" ]; then
    source "$ws_path"
    echo "  -> Sourced: $ws_path"
  else
    echo "  -> [WARNING] Custom workspace setup file not found: $ws_path"
  fi
done

# --- 2. 拼接最终的 Python 脚本路径 ---
MAIN_PY_FULL_PATH="$PROJECT_ROOT/$MAIN_PY_SCRIPT"

# --- 3. 启动主程序 ---
exec "$PY_EXECUTABLE" "$MAIN_PY_FULL_PATH"
