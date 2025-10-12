#!/bin/bash

# --- 1. 项目根目录 ---
PROJECT_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# --- 2. Python 可执行文件路径 ---
PY_EXECUTABLE="/home/ubuntu/anaconda3/envs/env_isaaclab/bin/python"

# --- 3. 要运行的主 Python 脚本 ---
MAIN_PY_SCRIPT="main.py"

# --- 4. 自定义 ROS 2 工作空间 ---
WORKSPACE_SETUP_PATH=(
  "/opt/ros/humble/setup.bash"
  "$PROJECT_ROOT/src/gsi_msgs/install/setup.bash"
)


export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DISTRO=humble
export PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages:$PYTHONPATH
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/ros/humble/lib:/home/ubuntu/anaconda3/envs/env_isaaclab/lib/python3.10/site-packages/isaacsim/exts/isaacsim.ros2.bridge/humble/lib:/home/ubuntu/PycharmProjects/isaacsim-gsi/src/gsi_msgs/install/scene_msgs/lib:/home/ubuntu/PycharmProjects/isaacsim-gsi/src/gsi_msgs/install/plan_msgs/lib
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
exec "$PY_EXECUTABLE" "$MAIN_PY_FULL_PATH"
