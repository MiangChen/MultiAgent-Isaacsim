#!/bin/bash
# 文件名：start_pycharm_with_ros2.sh

# 完整的 ROS2 环境设置
# source /opt/ros/humble/setup.bash

# 导出所有必要的环境变量
# export LD_LIBRARY_PATH=/opt/ros/humble/lib:/opt/ros/humble/local/lib:$LD_LIBRARY_PATH
# export PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages:$PYTHONPATH

# 启动 PyCharm
set -euo pipefail

# ===== 防呆：不要用 sudo =====
if [[ "${EUID:-$(id -u)}" -eq 0 ]]; then
  echo "[ERROR] 请勿用 sudo 运行本脚本；GUI 的 PyCharm 不应在 root 下启动。"
  exit 1
fi

# ===== 0) 目标：用系统 Python 3.10，避免 conda 污染 =====
MINICONDA_DIR="${MINICONDA_DIR:-$HOME/miniconda3}"

# 尝试优雅地 deactivate：只有在已经激活并且有 shell 函数时才调用
if [[ "${CONDA_SHLVL:-0}" -gt 0 ]]; then
  if [[ -f "$MINICONDA_DIR/etc/profile.d/conda.sh" ]]; then
    # shellcheck disable=SC1091
    source "$MINICONDA_DIR/etc/profile.d/conda.sh"
    conda deactivate || true
  fi
fi

# 无论是否成功，手工“净化”与 conda 相关的环境痕迹（PATH/LD_LIBRARY_PATH/PYTHONPATH）
# 只移除包含 miniconda/conda 关键字的段落，不影响其它路径
sanitize_path () {
  local val="${1:-}"
  [[ -z "$val" ]] && { echo ""; return; }
  echo "$val" | awk -v RS=: -v ORS=: '!/\/(mini)?conda/ {print}' | sed 's/:$//'
}

export PATH="$(sanitize_path "${PATH:-}")"
export LD_LIBRARY_PATH="$(sanitize_path "${LD_LIBRARY_PATH:-}")"
export PYTHONPATH="$(sanitize_path "${PYTHONPATH:-}")"

# 清掉常见 conda 变量
unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PROMPT_MODIFIER CONDA_SHLVL || true

# ===== 1) ROS 基础环境（系统 python 3.10）=====
if [[ -f /opt/ros/humble/setup.bash ]]; then
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  set -u
else
  echo "[ERROR] 未找到 /opt/ros/humble/setup.bash"
  exit 1
fi

# ===== 2) 叠加你的消息工作区（若存在）=====
if [[ -f "${HOME}/ws_msgs/install/setup.bash" ]]; then
  set +u
  # shellcheck disable=SC1091
  source "${HOME}/ws_msgs/install/setup.bash"
  set -u
else
  echo "[WARN] 未找到 ~/ws_msgs/install/setup.bash，消息包可能无法被导入。"
fi

# （可选）更多 overlay：
# if [[ -f "${HOME}/another_ws/install/setup.bash" ]]; then
#   set +u; source "${HOME}/another_ws/install/setup.bash"; set -u
# fi

# ===== 3) 常见 ROS 运行时变量（按需开启）=====
# export ROS_DOMAIN_ID=10
# export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# ===== 4) 一点点自检与提示 =====
if command -v python3 >/dev/null 2>&1; then
  PYVER="$(python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")' 2>/dev/null || echo "unknown")"
  echo "[INFO] python3 version: ${PYVER}"
  if [[ "$PYVER" != "3.10" ]]; then
    echo "[WARN] 当前系统 python3 不是 3.10，ROS2 Humble 官方建议 3.10。"
  fi
else
  echo "[WARN] 未找到系统 python3。"
fi

# ===== 5) 启动 PyCharm =====
if command -v pycharm-community >/dev/null 2>&1; then
  pycharm-community &
elif command -v pycharm-professional >/dev/null 2>&1; then
  pycharm-professional &
else
  echo "[ERROR] 未找到 pycharm-community 或 pycharm-professional 可执行程序。"
  exit 1
fi

echo "[INFO] PyCharm 已在『系统 Python + ROS Humble + ~/ws_msgs(如有)』环境下启动。"

