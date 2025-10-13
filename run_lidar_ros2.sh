#!/bin/bash
# sudo apt install ros-humble-rmw-fastrtps-cpp
USERNAME="cleanuser"
PATH_PYTHON="/home/cleanuser/anaconda3/envs/env_isaacsim"

# --- 1. 项目根目录 ---
PROJECT_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# --- 2. Python 可执行文件路径 ---
PY_EXECUTABLE="$PATH_PYTHON/bin/python"

# --- 3. 要运行的主 Python 脚本 ---
MAIN_PY_SCRIPT="isaacsim_lidar.py"

# --- 4. 自定义 ROS 2 工作空间 ---
WORKSPACE_SETUP_PATH=(
  "/opt/ros/humble/setup.bash"
)

# --- 5. Args 默认参数 ---
DEFAULT_ARGS=(
  "--namespace" "uav1,uav2,uav3"
)

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DISTRO=humble
export PYTHONPATH="/opt/ros/humble/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages:$PYTHONPATH"
export LD_LIBRARY_PATH="/opt/ros/humble/lib:$PATH_PYTHON/lib/python3.10/site-packages/isaacsim/exts/isaacsim.ros2.bridge/humble/lib:$LD_LIBRARY_PATH"
#SYSTEM_LIBSTDC=$(find /usr/lib/ -name "libstdc++.so.6" | head -n 1)
#if [ -f "$SYSTEM_LIBSTDC" ]; then
#    echo "[INFO] 找到系统 libstdc++.so.6: $SYSTEM_LIBSTDC"
#    export LD_PRELOAD="$SYSTEM_LIBSTDC"
#else
#    echo "[WARNING] 未找到系统级的 libstdc++.so.6，LD_PRELOAD 未设置。"
#fi
export FASTRTPS_DEFAULT_PROFILES_FILE=~/.ros/fastdds.xml

# ==============================================================================
#                                  LOGIC
# ==============================================================================

# --- 1. 环境设置 ---
echo "[INFO] Sourcing custom workspaces..."
for ws_path in "${WORKSPACE_SETUP_PATHS[@]}"; do
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
exec "$PY_EXECUTABLE" "$MAIN_PY_FULL_PATH" \
  "${DEFAULT_ARGS[@]}"