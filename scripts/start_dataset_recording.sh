#!/usr/bin/env bash
set -euo pipefail

# QGC 手柄数据集录像入口：
#   BlueOS RTP/H.264 5600 -> QGC 5701 + Recorder 5702
#
# 本脚本会尝试打开 QGC，但不启动飞控网关、YOLO、键盘控制或机械爪。
# QGC 与手柄如何控制 ROV 完全不由本脚本管理。

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
ROS_SETUP="/opt/ros/humble/setup.bash"
VENV_SETUP="${PROJECT_DIR}/.venv/bin/activate"
WORKSPACE_SETUP="${PROJECT_DIR}/ros2_ws/install/setup.bash"

ROV_IP="${ROV_IP:-192.168.2.2}"
VIDEO_SOURCE_PORT=5600
QGC_VIDEO_PORT=5701
RECORD_VIDEO_PORT=5702

BRIDGE_PID=""
CLEANED_UP=false

usage() {
  cat <<'EOF'
用法：
  ./scripts/start_dataset_recording.sh

使用 QGC 和手柄驾驶 ROV，本脚本只录制原始视频。
启动后立即开始录像；回到终端按 Enter 或 Ctrl+C 停止并封装 MKV。
脚本会自动尝试打开 QGC；首次须把视频设为 UDP H.264 端口 5701。

如艇载 IP 不是 192.168.2.2：
  ROV_IP=实际IP ./scripts/start_dataset_recording.sh

如 QGC 不在常见安装位置：
  QGC_EXECUTABLE=/完整路径/QGroundControl ./scripts/start_dataset_recording.sh
EOF
}

if (($# > 0)); then
  case "$1" in
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "未知参数：$1" >&2
      usage >&2
      exit 2
      ;;
  esac
fi

stop_bridge() {
  [[ -n "${BRIDGE_PID}" ]] || return 0
  if kill -0 "${BRIDGE_PID}" 2>/dev/null; then
    kill -INT "${BRIDGE_PID}" 2>/dev/null || true
    for _ in {1..50}; do
      kill -0 "${BRIDGE_PID}" 2>/dev/null || break
      sleep 0.1
    done
    if kill -0 "${BRIDGE_PID}" 2>/dev/null; then
      kill -TERM "${BRIDGE_PID}" 2>/dev/null || true
    fi
  fi
  wait "${BRIDGE_PID}" 2>/dev/null || true
}

cleanup() {
  local status=$?
  if [[ "${CLEANED_UP}" == true ]]; then
    return
  fi
  CLEANED_UP=true
  trap - EXIT INT TERM
  stop_bridge
  echo
  echo "录像和视频分流已停止；QGC、手柄和 MAVLink 连接未被本脚本关闭。"
  return "${status}"
}

trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

for required in "${ROS_SETUP}" "${VENV_SETUP}" "${WORKSPACE_SETUP}"; do
  if [[ ! -r "${required}" ]]; then
    echo "缺少必要环境文件：${required}" >&2
    echo "请先运行 ./scripts/install.sh 和 ./scripts/check_ros.sh。" >&2
    exit 1
  fi
done

set +u
source "${ROS_SETUP}"
source "${VENV_SETUP}"
source "${WORKSPACE_SETUP}"
set -u
export PYTHONNOUSERSITE=1
cd "${PROJECT_DIR}"

for command in python gst-launch-1.0 ffprobe ss ping; do
  if ! command -v "${command}" >/dev/null 2>&1; then
    echo "缺少命令 ${command}；请重新运行 ./scripts/install.sh。" >&2
    exit 1
  fi
done

python - <<'PY'
from rov_competition.dataset_recording import RtpMkvRecorder

print("原始 MKV 录像模块可用。")
PY

port_owner() {
  local port="$1"
  ss -H -lunp 2>/dev/null | grep -E ":${port}([[:space:]]|$)" || true
}

for port in "${VIDEO_SOURCE_PORT}" "${RECORD_VIDEO_PORT}"; do
  owner="$(port_owner "${port}")"
  if [[ -n "${owner}" ]]; then
    echo "UDP ${port} 已被占用：" >&2
    echo "${owner}" >&2
    exit 1
  fi
done

QGC_OWNER="$(port_owner "${QGC_VIDEO_PORT}")"
if [[ -n "${QGC_OWNER}" ]] && ! grep -Eiq 'qgroundcontrol' <<<"${QGC_OWNER}"; then
  echo "UDP ${QGC_VIDEO_PORT} 已被非 QGC 程序占用：" >&2
  echo "${QGC_OWNER}" >&2
  exit 1
fi

if ! ping -c 3 -W 2 "${ROV_IP}" >/dev/null; then
  echo "无法连通艇载电脑 ${ROV_IP}，未启动录像。" >&2
  exit 1
fi

SESSION_NAME="$(date +%Y%m%d_%H%M%S)"
SESSION_DIR="${PROJECT_DIR}/output/datasets/${SESSION_NAME}"
if [[ -e "${SESSION_DIR}" ]]; then
  SESSION_DIR="${SESSION_DIR}_$$"
fi
mkdir -p "${SESSION_DIR}/logs"

if ! pgrep -f '[Qq][Gg]round[Cc]ontrol' >/dev/null 2>&1; then
  QGC_COMMAND="${QGC_EXECUTABLE:-}"
  if [[ -z "${QGC_COMMAND}" ]]; then
    QGC_COMMAND="$(command -v QGroundControl 2>/dev/null || true)"
  fi
  if [[ -z "${QGC_COMMAND}" ]]; then
    QGC_COMMAND="$(command -v qgroundcontrol 2>/dev/null || true)"
  fi

  if [[ -n "${QGC_COMMAND}" && -x "${QGC_COMMAND}" ]]; then
    echo "正在打开 QGroundControl……"
    "${QGC_COMMAND}" >"${SESSION_DIR}/logs/qgc.log" 2>&1 &
  elif command -v flatpak >/dev/null 2>&1 \
    && flatpak info org.mavlink.qgroundcontrol >/dev/null 2>&1; then
    echo "正在打开 QGroundControl（Flatpak）……"
    flatpak run org.mavlink.qgroundcontrol \
      >"${SESSION_DIR}/logs/qgc.log" 2>&1 &
  else
    echo "未找到 QGC 可执行文件，请现在手动打开 QGroundControl。"
  fi
fi

# 已配置过的 QGC 通常会在几秒内恢复 5701。先自动等待；只有首次
# 设置未完成时才要求人工确认，以后每天运行无需重复输入第二条命令。
QGC_READY=false
for _ in {1..40}; do
  QGC_OWNER="$(port_owner "${QGC_VIDEO_PORT}")"
  if grep -Eiq 'qgroundcontrol' <<<"${QGC_OWNER}"; then
    QGC_READY=true
    break
  fi
  sleep 0.5
done

if [[ "${QGC_READY}" != true ]]; then
  echo
  echo "QGC 尚未监听 UDP ${QGC_VIDEO_PORT}。"
  echo "首次设置：Video Source = UDP H.264，端口 = ${QGC_VIDEO_PORT}，开启 Low Latency。"
  if [[ ! -t 0 ]]; then
    echo "当前不是交互终端，无法等待首次 QGC 设置。" >&2
    exit 1
  fi
  read -r -p "设置完成、QGC 已显示画面后按 Enter 继续……" _
  QGC_OWNER="$(port_owner "${QGC_VIDEO_PORT}")"
  if ! grep -Eiq 'qgroundcontrol' <<<"${QGC_OWNER}"; then
    echo "仍未确认 QGC 监听 UDP ${QGC_VIDEO_PORT}，未开始录像。" >&2
    [[ -z "${QGC_OWNER}" ]] || echo "${QGC_OWNER}" >&2
    exit 1
  fi
fi

# QGC 若仍保留默认 5600 视频源，会与分流器争抢原始数据。QGC 启动后
# 再次检查，确保 5600 和录像端口只由本脚本稍后占用。
for port in "${VIDEO_SOURCE_PORT}" "${RECORD_VIDEO_PORT}"; do
  owner="$(port_owner "${port}")"
  if [[ -n "${owner}" ]]; then
    echo "UDP ${port} 在 QGC 启动后被占用：" >&2
    echo "${owner}" >&2
    if [[ "${port}" == "${VIDEO_SOURCE_PORT}" ]]; then
      echo "请把 QGC 视频端口从 5600 改成 ${QGC_VIDEO_PORT}。" >&2
    fi
    exit 1
  fi
done

echo "============================================================"
echo "ROV 手柄采集：只录像，不控制飞控"
echo "  Video: ${VIDEO_SOURCE_PORT} -> QGC ${QGC_VIDEO_PORT} + Recorder ${RECORD_VIDEO_PORT}"
echo "  Output: ${SESSION_DIR}"
echo "  Stop:   回到本终端按 Enter 或 Ctrl+C"
echo "============================================================"

ros2 run rov_competition rov_stream_bridge \
  --source-port "${VIDEO_SOURCE_PORT}" \
  --qgc-host 127.0.0.1 \
  --qgc-port "${QGC_VIDEO_PORT}" \
  --no-inference \
  --record-host 127.0.0.1 \
  --record-port "${RECORD_VIDEO_PORT}" \
  --no-display \
  >"${SESSION_DIR}/logs/video_bridge.log" 2>&1 &
BRIDGE_PID=$!

sleep 2
if ! kill -0 "${BRIDGE_PID}" 2>/dev/null; then
  echo "视频分流启动失败：" >&2
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

if ((RECORD_STATUS != 0)); then
  echo "录像工具以状态 ${RECORD_STATUS} 退出，请查看 ${SESSION_DIR}/logs。" >&2
fi
echo "采集文件：${SESSION_DIR}"
exit "${RECORD_STATUS}"
