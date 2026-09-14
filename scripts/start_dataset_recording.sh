#!/usr/bin/env bash
set -euo pipefail

# QGC 手柄录像：QGC 直接接收 5600，软件链从 5700 复制到录像器 5704。
# 本脚本绝不连接 MAVLink，也不启动 ROS 控制、YOLO 或机械爪。

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
ROS_SETUP="/opt/ros/humble/setup.bash"
VENV_SETUP="${PROJECT_DIR}/.venv/bin/activate"
WORKSPACE_SETUP="${PROJECT_DIR}/ros2_ws/install/setup.bash"
ROV_IP="${ROV_IP:-192.168.2.2}"
TOPSIDE_IP="${TOPSIDE_IP:-192.168.2.1}"
QGC_VIDEO_PORT=5600
SOFTWARE_VIDEO_PORT=5700
RECORD_VIDEO_PORT=5704
BRIDGE_PID=""
CLEANED_UP=false

usage() {
  printf '%s\n' \
    '用法：./scripts/start_dataset_recording.sh' \
    '' \
    '用 QGC 和手柄驾驶 ROV，本脚本只录制原始视频。' \
    '开始后回到终端按 Enter 或 Ctrl+C 停止并封装 MKV。'
}
if (($# > 0)); then
  case "$1" in -h|--help) usage; exit 0 ;; *) echo "未知参数：$1" >&2; exit 2 ;; esac
fi

cleanup() {
  local status=$?
  [[ "${CLEANED_UP}" == false ]] || return
  CLEANED_UP=true
  trap - EXIT INT TERM
  if [[ -n "${BRIDGE_PID}" ]] && kill -0 "${BRIDGE_PID}" 2>/dev/null; then
    kill -INT "${BRIDGE_PID}" 2>/dev/null || true
    for _ in {1..50}; do kill -0 "${BRIDGE_PID}" 2>/dev/null || break; sleep 0.1; done
    kill -0 "${BRIDGE_PID}" 2>/dev/null && kill -TERM "${BRIDGE_PID}" 2>/dev/null || true
  fi
  [[ -z "${BRIDGE_PID}" ]] || wait "${BRIDGE_PID}" 2>/dev/null || true
  echo
  echo "录像和软件分流已停止；QGC、手柄和 MAVLink 不受影响。"
  return "${status}"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

for required in "${ROS_SETUP}" "${VENV_SETUP}" "${WORKSPACE_SETUP}"; do
  [[ -r "${required}" ]] || { echo "缺少环境文件：${required}" >&2; exit 1; }
done
set +u
source "${ROS_SETUP}"
source "${VENV_SETUP}"
source "${WORKSPACE_SETUP}"
set -u
export PYTHONNOUSERSITE=1
cd "${PROJECT_DIR}"
for command in python ros2 gst-launch-1.0 ffprobe ss ping ip; do
  command -v "${command}" >/dev/null 2>&1 || { echo "缺少命令 ${command}" >&2; exit 1; }
done
python - <<'PY'
from rov_competition.dataset_recording import RtpMkvRecorder
print("原始 MKV 录像模块可用。")
PY

port_owner() { ss -H -lunp 2>/dev/null | grep -E ":$1([[:space:]]|$)" || true; }
for port in "${SOFTWARE_VIDEO_PORT}" "${RECORD_VIDEO_PORT}"; do
  owner="$(port_owner "${port}")"
  [[ -z "${owner}" ]] || { echo "UDP ${port} 已被占用：" >&2; echo "${owner}" >&2; exit 1; }
done
if ! ip -br -4 addr | grep -Eq "(^|[[:space:]])${TOPSIDE_IP}/24([[:space:]]|$)"; then
  echo "有线网卡未设为 ${TOPSIDE_IP}/24。" >&2
  exit 1
fi
ping -c 3 -W 2 "${ROV_IP}" >/dev/null || { echo "无法连通 ${ROV_IP}" >&2; exit 1; }

SESSION_NAME="$(date +%Y%m%d_%H%M%S)"
SESSION_DIR="${PROJECT_DIR}/output/datasets/${SESSION_NAME}"
[[ ! -e "${SESSION_DIR}" ]] || SESSION_DIR="${SESSION_DIR}_$$"
mkdir -p "${SESSION_DIR}/logs"

if ! pgrep -f '[Qq][Gg]round[Cc]ontrol' >/dev/null 2>&1; then
  QGC_COMMAND="${QGC_EXECUTABLE:-}"
  [[ -n "${QGC_COMMAND}" ]] || QGC_COMMAND="$(command -v QGroundControl 2>/dev/null || true)"
  [[ -n "${QGC_COMMAND}" ]] || QGC_COMMAND="$(command -v qgroundcontrol 2>/dev/null || true)"
  if [[ -n "${QGC_COMMAND}" && -x "${QGC_COMMAND}" ]]; then
    "${QGC_COMMAND}" >"${SESSION_DIR}/logs/qgc.log" 2>&1 &
  elif command -v flatpak >/dev/null 2>&1 && flatpak info org.mavlink.qgroundcontrol >/dev/null 2>&1; then
    flatpak run org.mavlink.qgroundcontrol >"${SESSION_DIR}/logs/qgc.log" 2>&1 &
  else
    echo "请现在手动打开 QGroundControl。"
  fi
fi

QGC_READY=false
for _ in {1..40}; do
  if grep -Eiq 'qgroundcontrol' <<<"$(port_owner "${QGC_VIDEO_PORT}")"; then QGC_READY=true; break; fi
  sleep 0.5
done
if [[ "${QGC_READY}" != true ]]; then
  echo "QGC 未监听默认 UDP H.264 端口 5600。" >&2
  echo "请让 QGC 使用默认 5600 视频并开启 Low Latency。" >&2
  exit 1
fi

echo "============================================================"
echo "ROV 手柄采集：只录像，不控制飞控"
echo "  QGC:      BlueOS -> ${QGC_VIDEO_PORT}"
echo "  Recorder: BlueOS -> ${SOFTWARE_VIDEO_PORT} -> ${RECORD_VIDEO_PORT}"
echo "  Output:   ${SESSION_DIR}"
echo "============================================================"

ros2 run rov_competition rov_stream_bridge \
  --source-port "${SOFTWARE_VIDEO_PORT}" \
  --no-qgc \
  --no-inference \
  --record-host 127.0.0.1 \
  --record-port "${RECORD_VIDEO_PORT}" \
  --no-display \
  >"${SESSION_DIR}/logs/video_bridge.log" 2>&1 &
BRIDGE_PID=$!
sleep 2
if ! kill -0 "${BRIDGE_PID}" 2>/dev/null; then
  tail -n 80 "${SESSION_DIR}/logs/video_bridge.log" >&2 || true
  exit 1
fi

set +e
ros2 run rov_competition rov_dataset_record \
  --session-dir "${SESSION_DIR}" \
  --project-dir "${PROJECT_DIR}" \
  --record-port "${RECORD_VIDEO_PORT}" \
  --payload-type 96
RECORD_STATUS=$?
set -e
echo "采集文件：${SESSION_DIR}"
exit "${RECORD_STATUS}"
