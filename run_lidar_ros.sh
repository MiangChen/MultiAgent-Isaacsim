#!/bin/bash
# sudo apt install ros-humble-rmw-fastrtps-cpp
PATH_PYTHON="/home/cleanuser/anaconda3/envs/env_isaacsim"
PATH_GSI_MSG="/home/cleanuser/PycharmProjects/isaacsim-gsi/src/gsi_msgs"

# --- 1. 项目根目录 ---
PROJECT_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# --- 2. Python 可执行文件路径 ---
PY_EXECUTABLE="$PATH_PYTHON/bin/python"

# --- 3. 要运行的主 Python 脚本 ---
MAIN_PY_SCRIPT="isaacsim_lidar.py"

# --- 4. 自定义 ROS 2 工作空间 ---
WORKSPACE_SETUP_PATH=(
  "/opt/ros/humble/setup.bash"
  "$PATH_GSI_MSG/install/setup.bash"
)

# --- 5. Args 默认参数 ---
DEFAULT_ARGS=(
  "--namespace" "uav1,uav2,uav3"
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
exec "$PY_EXECUTABLE" "$MAIN_PY_FULL_PATH" \
  "${DEFAULT_ARGS[@]}"