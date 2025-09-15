#!/usr/bin/env bash
# 文件名：run_main.sh
set -euo pipefail

# ====== 可按需修改 ======
CONDA_ROOT="${CONDA_ROOT:-$HOME/miniconda3}"
CONDA_ENV="${CONDA_ENV:-env_isaaclab}"
PY_CONDA="$CONDA_ROOT/envs/$CONDA_ENV/bin/python"
MAIN_PY="${MAIN_PY:-$HOME/multiagent-isaacsimROS/src/multiagent_isaacsim/multiagent_isaacsim/main.py}"

DEFAULT_ENABLE_ARGS=(
  --enable isaacsim.ros2.bridge
  --enable omni.kit.graph.editor.core
  --enable omni.graph.bundle.action
  --enable omni.kit.graph.delegate.modern
  --enable isaacsim.asset.gen.omap
  --enable omni.graph.window.action
)

# 解析参数：--main 切换入口，其余透传
EXTRA_ARGS=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --main) MAIN_PY="$2"; shift 2 ;;
    *) EXTRA_ARGS+=("$1"); shift ;;
  esac
done

# 基础检查
if [[ ! -x "$PY_CONDA" ]]; then
  echo "[ERROR] 找不到 conda 解释器: $PY_CONDA"
  exit 1
fi
if [[ ! -f "$MAIN_PY" ]]; then
  echo "[ERROR] main.py 未找到: $MAIN_PY"
  exit 1
fi

# 激活 conda
if [[ -f "$CONDA_ROOT/etc/profile.d/conda.sh" ]]; then
  # shellcheck disable=SC1090
  source "$CONDA_ROOT/etc/profile.d/conda.sh"
  conda activate "$CONDA_ENV"
else
  echo "[ERROR] 未找到 $CONDA_ROOT/etc/profile.d/conda.sh"
  exit 1
fi

# ===== 叠加 ROS + 工作区（用 set +u 防止 unbound variable）=====
# 预定义可能被 setup.bash 读取的变量，防止 nounset 报错
export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES-}"
export COLCON_CURRENT_PREFIX="${COLCON_CURRENT_PREFIX-}"
export COLCON_TRACE="${COLCON_TRACE-}"

if [[ -f /opt/ros/humble/setup.bash ]]; then
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  set -u
else
  echo "[ERROR] 未找到 /opt/ros/humble/setup.bash"
  exit 1
fi

if [[ -f "$HOME/gsi_msgs/install/setup.bash" ]]; then
  set +u
  # shellcheck disable=SC1091
  source "$HOME/gsi_msgs/install/setup.bash"
  set -u
else
  echo "[WARN] 未找到 ~/gsi_msgs/install/setup.bash，消息包可能无法被导入。"
fi

# ===== 环境变量（你要求的）=====
export PYTHONUNBUFFERED=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DISTRO=humble

# 自动推断 Python 次版本，拼接桥接库目录
PY_MINOR="$("$PY_CONDA" -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")' 2>/dev/null || echo "3.10")"
BRIDGE_LIB="$CONDA_ROOT/envs/$CONDA_ENV/lib/python${PY_MINOR}/site-packages/isaacsim/exts/isaacsim.ros2.bridge/humble/lib"
if [[ -d "$BRIDGE_LIB" ]]; then
  if [[ -n "${LD_LIBRARY_PATH:-}" ]]; then
    export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$BRIDGE_LIB"
  else
    export LD_LIBRARY_PATH="$BRIDGE_LIB"
  fi
else
  echo "[WARN] 未找到桥接库目录：$BRIDGE_LIB"
fi

# ===== 自检（无 heredoc 版本）=====
"$PY_CONDA" -c "from plan_msgs.msg import Plan, SkillInfo, TimestepSkills; from scene_msgs.msg import PrimTransform, SceneModifications; print('[CHECK] plan_msgs/scene_msgs OK')" || {
  echo '[ERROR] plan_msgs/scene_msgs 导入失败'; exit 1;
}

"$PY_CONDA" -c "import importlib, sys; m=importlib.util.find_spec('rclpy');
print('[CHECK] rclpy OK' if m else '[WARN] rclpy 未安装或不可用')" || true

# ===== 设置 Python 的工作目录 =====
PY_WORKDIR="${PY_WORKDIR:-/home/ubuntu/multiagent-isaacsimROS/src/multiagent_isaacsim/multiagent_isaacsim}"
if [[ -d "$PY_WORKDIR" ]]; then
  cd "$PY_WORKDIR"
else
  echo "[ERROR] 工作目录不存在: $PY_WORKDIR"
  exit 1
fi

# ===== 启动 =====
echo "[INFO] Python: $("$PY_CONDA" -V) @ $PY_CONDA"
echo "[INFO] Launch: $MAIN_PY"
echo "[INFO] default enables: ${DEFAULT_ENABLE_ARGS[*]}"
echo "[INFO] extra args      : ${EXTRA_ARGS[*]:-(none)}"
echo "[INFO] LD_LIBRARY_PATH : ${LD_LIBRARY_PATH:-<empty>}"

exec "$PY_CONDA" "$MAIN_PY" \
  "${DEFAULT_ENABLE_ARGS[@]}" \
  "${EXTRA_ARGS[@]}"