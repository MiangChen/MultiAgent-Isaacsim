#!/usr/bin/env bash
# 文件名：run_main.sh
set -euo pipefail

# ===== 可按需修改 =====
CONDA_ROOT="${CONDA_ROOT:-$HOME/miniconda3}"
CONDA_ENV="${CONDA_ENV:-env_isaaclab}"
PY_CONDA="$CONDA_ROOT/envs/$CONDA_ENV/bin/python"
MAIN_PY="${MAIN_PY:-$HOME/multiagent-isaacsimROS/src/multiagent_isaacsim/multiagent_isaacsim/main.py}"

# 需要默认附加给 main.py 的参数（可根据需要增删）
DEFAULT_ENABLE_ARGS=(
  --enable isaacsim.ros2.bridge
  --enable omni.kit.graph.editor.core
  --enable omni.graph.bundle.action
  --enable omni.kit.graph.delegate.modern
  --enable isaacsim.asset.gen.omap
  --enable omni.graph.window.action
)

# 解析简单参数：--main 换入口；其余参数透传给 main.py
EXTRA_ARGS=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --main) MAIN_PY="$2"; shift 2 ;;
    *) EXTRA_ARGS+=("$1"); shift ;;
  esac
done

# 0) 基础检查
if [[ ! -x "$PY_CONDA" ]]; then
  echo "[ERROR] 找不到 conda 解释器: $PY_CONDA"
  echo "       请检查 CONDA_ROOT/CONDA_ENV 或在脚本顶部覆盖变量。"
  exit 1
fi
if [[ ! -f "$MAIN_PY" ]]; then
  echo "[ERROR] main.py 未找到: $MAIN_PY"
  exit 1
fi

# 1) 激活 conda 环境
if [[ -f "$CONDA_ROOT/etc/profile.d/conda.sh" ]]; then
  # shellcheck disable=SC1090
  source "$CONDA_ROOT/etc/profile.d/conda.sh"
  conda activate "$CONDA_ENV"
else
  echo "[ERROR] 未找到 $CONDA_ROOT/etc/profile.d/conda.sh，无法激活 conda。"
  exit 1
fi

# 2) 叠加 ROS + 消息工作区
if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
else
  echo "[ERROR] 未找到 /opt/ros/humble/setup.bash"
  exit 1
fi
if [[ -f "$HOME/ws_msgs/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "$HOME/ws_msgs/install/setup.bash"
else
  echo "[WARN] 未找到 ~/ws_msgs/install/setup.bash，消息包可能无法被导入。"
fi

# 3) 环境变量设置（按你要求）
export PYTHONUNBUFFERED=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DISTRO=humble

# 计算 isaacsim.ros2.bridge 的库目录（随 Python 次版本变化）
PY_MINOR="$("$PY_CONDA" - <<'PY'
import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")
PY
)"
BRIDGE_LIB="$CONDA_ROOT/envs/$CONDA_ENV/lib/python${PY_MINOR}/site-packages/isaacsim/exts/isaacsim.ros2.bridge/humble/lib"

if [[ -d "$BRIDGE_LIB" ]]; then
  # 追加到 LD_LIBRARY_PATH（保持已有值）
  if [[ -n "${LD_LIBRARY_PATH:-}" ]]; then
    export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$BRIDGE_LIB"
  else
    export LD_LIBRARY_PATH="$BRIDGE_LIB"
  fi
else
  echo "[WARN] 未找到桥接库目录：$BRIDGE_LIB"
fi

# 4) 快速自检
"$PY_CONDA" - <<'PY' || {
  from plan_msgs.msg import Plan, SkillInfo, TimestepSkills
  from scene_msgs.msg import PrimTransform, SceneModifications
  print("[CHECK] plan_msgs/scene_msgs OK")
PY

"$PY_CONDA" - <<'PY' || true
try:
    import rclpy
    print("[CHECK] rclpy OK")
except Exception as e:
    print("[WARN] rclpy 导入失败：", e)
    print("       如果你的程序需要运行 ROS 节点，建议用系统 Python 或 Isaac 的 python.sh。")
PY

# 5) 运行 main.py（conda 解释器）
echo "[INFO] Python: $("$PY_CONDA" -V) @ $PY_CONDA"
echo "[INFO] Launch: $MAIN_PY"
echo "[INFO]   default enables: ${DEFAULT_ENABLE_ARGS[*]}"
echo "[INFO]   extra args      : ${EXTRA_ARGS[*]:-(none)}"
echo "[INFO]   LD_LIBRARY_PATH : ${LD_LIBRARY_PATH:-<empty>}"

exec "$PY_CONDA" "$MAIN_PY" \
  "${DEFAULT_ENABLE_ARGS[@]}" \
  "${EXTRA_ARGS[@]}"
