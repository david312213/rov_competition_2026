#!/usr/bin/env bash
set -euo pipefail

# 纯视频/识别一键入口。QGC 直接接收 BlueOS -> 5600；本脚本只处理
# BlueOS -> 5700 -> YOLO 5702。它不启动飞控网关，也没有任何控制接口。

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
ROS_SETUP="/opt/ros/humble/setup.bash"
VENV_SETUP="${PROJECT_DIR}/.venv/bin/activate"
WORKSPACE_SETUP="${PROJECT_DIR}/ros2_ws/install/setup.bash"
ROBOT_CONFIG="${PROJECT_DIR}/config/robot.yaml"
AUTONOMY_TEMPLATE="${PROJECT_DIR}/ros2_ws/src/rov_competition/config/autonomy.yaml"
AUTONOMY_LOCAL="${PROJECT_DIR}/config/autonomy.local.yaml"
AUTONOMY_CONFIG="${AUTONOMY_CONFIG:-${AUTONOMY_TEMPLATE}}"
TARGETS_CONFIG="${PROJECT_DIR}/ros2_ws/src/rov_competition/config/targets.yaml"

START_QGC=true
START_VIEWER=true
ROV_IP="${ROV_IP:-192.168.2.2}"
TOPSIDE_IP="${TOPSIDE_IP:-192.168.2.1}"
LAUNCH_PID=""
VIEWER_PID=""

usage() {
  cat <<'EOF'
用法：./scripts/start_video_test.sh [--no-qgc] [--no-viewer]

默认启动 5700 -> 5702 视频分流和 GPU YOLO，并尝试打开 QGC 与带框查看器。
本脚本只做识别显示，不连接飞控、不解锁、不发送运动或机械爪命令。

如 QGC 不在 PATH：
  QGC_EXECUTABLE=/完整路径/QGroundControl.AppImage ./scripts/start_video_test.sh
EOF
}

while (($# > 0)); do
  case "$1" in
    --no-qgc) START_QGC=false ;;
    --no-viewer) START_VIEWER=false ;;
    -h|--help) usage; exit 0 ;;
    *) echo "未知参数：$1" >&2; usage >&2; exit 2 ;;
  esac
  shift
done

for required in \
  "${ROS_SETUP}" "${VENV_SETUP}" "${WORKSPACE_SETUP}" "${ROBOT_CONFIG}" \
  "${AUTONOMY_TEMPLATE}" "${TARGETS_CONFIG}"; do
  [[ -r "${required}" ]] || { echo "缺少必要文件：${required}" >&2; exit 1; }
done

set +u
source "${ROS_SETUP}"
source "${VENV_SETUP}"
source "${WORKSPACE_SETUP}"
set -u
export PYTHONNOUSERSITE=1
cd "${PROJECT_DIR}"
if [[ -z "${AUTONOMY_CONFIG:-}" || "${AUTONOMY_CONFIG}" == "${AUTONOMY_TEMPLATE}" ]]; then
  if [[ -r "${AUTONOMY_LOCAL}" ]]; then
    AUTONOMY_CONFIG="${AUTONOMY_LOCAL}"
  else
    AUTONOMY_CONFIG="${AUTONOMY_TEMPLATE}"
  fi
fi
[[ -r "${AUTONOMY_CONFIG}" ]] || { echo "缺少活动自主配置：${AUTONOMY_CONFIG}" >&2; exit 1; }

for command in python ros2 ss ping ip; do
  command -v "${command}" >/dev/null 2>&1 || { echo "缺少命令 ${command}" >&2; exit 1; }
done

python - <<'PY'
import cv2
import numpy
import torch

if int(numpy.__version__.split(".", 1)[0]) >= 2:
    raise RuntimeError(f"现场环境要求 NumPy 1.x，当前 {numpy.__version__}")
if not torch.cuda.is_available():
    raise RuntimeError("视频测试必须使用 CUDA GPU，不允许静默退回 CPU")
print(f"视觉环境正常：NumPy {numpy.__version__}, OpenCV {cv2.__version__}")
print(f"GPU：{torch.cuda.get_device_name(0)}")
PY

ros2 run rov_competition rov_field_setup \
  --project-dir "${PROJECT_DIR}" weights verify

port_owner() { ss -H -lunp 2>/dev/null | grep -E ":$1([[:space:]]|$)" || true; }
ensure_free_port() {
  local owner
  owner="$(port_owner "$1")"
  [[ -z "${owner}" ]] || { echo "UDP $1 已被占用：" >&2; echo "${owner}" >&2; exit 1; }
}

ensure_free_port 5700
ensure_free_port 5702
if ! ip -br -4 addr | grep -Eq "(^|[[:space:]])${TOPSIDE_IP}/24([[:space:]]|$)"; then
  echo "有线网卡未设为 ${TOPSIDE_IP}/24。" >&2
  exit 1
fi
ping -c 3 -W 2 "${ROV_IP}" >/dev/null || { echo "无法连通 ${ROV_IP}" >&2; exit 1; }

QGC_OWNER="$(port_owner 5600)"
if [[ -n "${QGC_OWNER}" ]] && ! grep -Eiq 'qgroundcontrol' <<<"${QGC_OWNER}"; then
  echo "UDP 5600 已被非 QGC 程序占用：" >&2
  echo "${QGC_OWNER}" >&2
  exit 1
fi
if ros2 node list 2>/dev/null | grep -Eq '^/(rov_autonomy|rov_stream_bridge|rov_search_perception)$'; then
  echo "检测到旧的视频/识别节点，请先回原终端 Ctrl+C。" >&2
  exit 1
fi

cleanup() {
  local status=$?
  trap - EXIT INT TERM
  for pid in "${VIEWER_PID}" "${LAUNCH_PID}"; do
    [[ -n "${pid}" ]] || continue
    kill -0 "${pid}" 2>/dev/null || continue
    kill -INT "${pid}" 2>/dev/null || true
  done
  [[ -z "${VIEWER_PID}" ]] || wait "${VIEWER_PID}" 2>/dev/null || true
  [[ -z "${LAUNCH_PID}" ]] || wait "${LAUNCH_PID}" 2>/dev/null || true
  echo
  echo "视频分流和 YOLO 已停止；QGC 继续由操作员管理。"
  exit "${status}"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

if "${START_QGC}" && ! pgrep -f '[Qq][Gg]round[Cc]ontrol' >/dev/null 2>&1; then
  QGC_COMMAND="${QGC_EXECUTABLE:-}"
  [[ -n "${QGC_COMMAND}" ]] || QGC_COMMAND="$(command -v QGroundControl 2>/dev/null || true)"
  [[ -n "${QGC_COMMAND}" ]] || QGC_COMMAND="$(command -v qgroundcontrol 2>/dev/null || true)"
  if [[ -n "${QGC_COMMAND}" && -x "${QGC_COMMAND}" ]]; then
    "${QGC_COMMAND}" >"${XDG_RUNTIME_DIR:-/tmp}/rov_qgc_$$.log" 2>&1 &
  elif command -v flatpak >/dev/null 2>&1 \
    && flatpak info org.mavlink.qgroundcontrol >/dev/null 2>&1; then
    flatpak run org.mavlink.qgroundcontrol \
      >"${XDG_RUNTIME_DIR:-/tmp}/rov_qgc_$$.log" 2>&1 &
  else
    echo "未自动找到 QGC；请手动打开，默认视频端口为 5600。"
  fi
fi

if "${START_QGC}"; then
  QGC_READY=false
  for _ in {1..40}; do
    if grep -Eiq 'qgroundcontrol' <<<"$(port_owner 5600)"; then
      QGC_READY=true
      break
    fi
    sleep 0.5
  done
  [[ "${QGC_READY}" == true ]] || {
    echo "QGC 未监听默认 UDP H.264 端口 5600。" >&2
    exit 1
  }
fi

echo "============================================================"
echo "ROV 一键视频识别测试（纯感知，不连接飞控）"
echo "  QGC：BlueOS -> UDP 5600"
echo "  YOLO：BlueOS -> UDP 5700 -> UDP 5702"
echo "  停止：本终端 Ctrl+C"
echo "============================================================"

ros2 launch rov_competition video_test.launch.py \
  robot_config:="${ROBOT_CONFIG}" \
  autonomy_config:="${AUTONOMY_CONFIG}" \
  targets_config:="${TARGETS_CONFIG}" \
  source_port:=5700 \
  ai_port:=5702 &
LAUNCH_PID=$!
sleep 2
kill -0 "${LAUNCH_PID}" 2>/dev/null || { echo "ROS 视频启动失败。" >&2; exit 1; }

if "${START_VIEWER}"; then
  if ros2 pkg prefix rqt_image_view >/dev/null 2>&1; then
    (sleep 4; exec ros2 run rqt_image_view rqt_image_view) &
    VIEWER_PID=$!
    echo "带框查看器将打开；选择 /rov/annotated_image，传输方式选 compressed。"
  else
    echo "未安装 rqt_image_view；可用 ros2 topic hz /rov/detections 检查。"
  fi
fi

set +e
wait "${LAUNCH_PID}"
STATUS=$?
set -e
exit "${STATUS}"
