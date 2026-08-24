#!/usr/bin/env bash
set -euo pipefail

# 一键键盘数据集采集（默认端口版）：
#   BlueOS MAVLink -> QGC 14550（直连） + ROS 14551（直连）
#   BlueOS Video   -> QGC 5600（直连） + Software 5700
#   Software 5700  -> Recorder 5704
#
# 脚本不启动中间转发器、YOLO 或机械爪；八推进器混控仍由 ArduSub 完成。

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
ROS_SETUP="/opt/ros/humble/setup.bash"
VENV_SETUP="${PROJECT_DIR}/.venv/bin/activate"
WORKSPACE_SETUP="${PROJECT_DIR}/ros2_ws/install/setup.bash"
ROBOT_CONFIG="${PROJECT_DIR}/config/robot.yaml"
DATASET_CONFIG="${PROJECT_DIR}/config/dataset.yaml"
DATASET_EXAMPLE="${PROJECT_DIR}/ros2_ws/src/rov_competition/config/dataset.example.yaml"

ROV_IP="${ROV_IP:-192.168.2.2}"
TOPSIDE_IP="${TOPSIDE_IP:-192.168.2.1}"
QGC_MAVLINK_PORT=14550
ROS_MAVLINK_PORT=14551
QGC_VIDEO_PORT=5600
SOFTWARE_VIDEO_PORT=5700
RECORD_VIDEO_PORT=5704

GATEWAY_PID=""
BRIDGE_PID=""
CLEANED_UP=false

usage() {
  printf '%s\n' \
    '用法：./scripts/start_dataset_collection.sh' \
    '' \
    '首次需在 BlueOS 配置 14550/14551 两路 MAVLink 和 5600/5700 两路视频。' \
    '日常运行不再修改 QGC 端口，也不需要额外的 MAVLink 转发进程。'
}

if (($# > 0)); then
  case "$1" in
    -h|--help) usage; exit 0 ;;
    *) echo "未知参数：$1" >&2; usage >&2; exit 2 ;;
  esac
fi

stop_owned_process() {
  local pid="$1" label="$2"
  [[ -n "${pid}" ]] || return 0
  if kill -0 "${pid}" 2>/dev/null; then
    kill -INT "${pid}" 2>/dev/null || true
    for _ in {1..50}; do
      kill -0 "${pid}" 2>/dev/null || break
      sleep 0.1
    done
    if kill -0 "${pid}" 2>/dev/null; then
      echo "${label} 未在 5 秒内退出，发送 SIGTERM。" >&2
      kill -TERM "${pid}" 2>/dev/null || true
    fi
  fi
  wait "${pid}" 2>/dev/null || true
}

cleanup() {
  local status=$?
  if [[ "${CLEANED_UP}" == true ]]; then return; fi
  CLEANED_UP=true
  trap - EXIT INT TERM
  # 前台键盘窗口先完成上锁/急停，再回收本脚本所有的后台进程。
  stop_owned_process "${BRIDGE_PID}" "视频分流器"
  stop_owned_process "${GATEWAY_PID}" "飞控网关"
  echo
  echo "数据集采集后台进程已停止；QGroundControl 由操作员继续管理。"
  return "${status}"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

for required in "${ROS_SETUP}" "${VENV_SETUP}" "${WORKSPACE_SETUP}" "${ROBOT_CONFIG}"; do
  if [[ ! -r "${required}" ]]; then
    echo "缺少必要文件：${required}" >&2
    echo "请先运行 ./scripts/install.sh 和 ./scripts/check_ros.sh。" >&2
    exit 1
  fi
done
if [[ ! -r "${DATASET_CONFIG}" ]]; then
  cp -- "${DATASET_EXAMPLE}" "${DATASET_CONFIG}"
  echo "已生成：${DATASET_CONFIG}"
fi

set +u
source "${ROS_SETUP}"
source "${VENV_SETUP}"
source "${WORKSPACE_SETUP}"
set -u
export PYTHONNOUSERSITE=1
cd "${PROJECT_DIR}"

for command in python ros2 gst-launch-1.0 ffprobe ss ping ip timeout; do
  if ! command -v "${command}" >/dev/null 2>&1; then
    echo "缺少命令 ${command}；请重新运行 ./scripts/install.sh。" >&2
    exit 1
  fi
done

python - <<'PY'
import pygame
import pymavlink
import rclpy
import yaml
print(f"键盘窗口环境正常：Pygame {pygame.version.ver}")
print("本采集模式不导入 Ultralytics/Torch，不需要权重。")
PY

python - "${ROBOT_CONFIG}" "${DATASET_CONFIG}" "${ROS_MAVLINK_PORT}" <<'PY'
import sys
from rov_competition.config import load_dataset_config, load_robot_config
robot = load_robot_config(sys.argv[1])
dataset = load_dataset_config(sys.argv[2])
errors = list(dataset.readiness_errors(robot))
expected_port = int(sys.argv[3])
if not robot.connection_uri.strip().endswith(f":{expected_port}"):
    errors.append(f"robot.yaml 的 mavlink.connection_uri 必须监听 {expected_port}")
if errors:
    print("数据集配置尚不允许实艇：", file=sys.stderr)
    for error in errors:
        print(f"  - {error}", file=sys.stderr)
    raise SystemExit(2)
print("实艇配置门检查通过。")
PY

port_owner() {
  local port="$1"
  ss -H -lunp 2>/dev/null | grep -E ":${port}([[:space:]]|$)" || true
}
ensure_free_port() {
  local port="$1" owner
  owner="$(port_owner "${port}")"
  if [[ -n "${owner}" ]]; then
    echo "UDP ${port} 已被占用，请先在旧终端 Ctrl+C：" >&2
    echo "${owner}" >&2
    exit 1
  fi
}
qgc_owns_port() {
  grep -Eiq 'qgroundcontrol' <<<"$(port_owner "$1")"
}
start_qgc_if_needed() {
  if pgrep -f '[Qq][Gg]round[Cc]ontrol' >/dev/null 2>&1; then return; fi
  local executable="${QGC_EXECUTABLE:-}"
  [[ -n "${executable}" ]] || executable="$(command -v QGroundControl 2>/dev/null || true)"
  [[ -n "${executable}" ]] || executable="$(command -v qgroundcontrol 2>/dev/null || true)"
  if [[ -n "${executable}" && -x "${executable}" ]]; then
    "${executable}" >"${SESSION_DIR}/logs/qgc.log" 2>&1 &
  elif command -v flatpak >/dev/null 2>&1 \
    && flatpak info org.mavlink.qgroundcontrol >/dev/null 2>&1; then
    flatpak run org.mavlink.qgroundcontrol >"${SESSION_DIR}/logs/qgc.log" 2>&1 &
  else
    echo "请现在手动打开 QGroundControl。"
  fi
}

if ! ip -br -4 addr | grep -Eq "(^|[[:space:]])${TOPSIDE_IP}/24([[:space:]]|$)"; then
  echo "有线网卡未设为 ${TOPSIDE_IP}/24，禁止启动。" >&2
  echo "请在 Ubuntu 网络设置中将有线 IPv4 改为手动 ${TOPSIDE_IP}，掩码 255.255.255.0。" >&2
  exit 1
fi
if ! ping -c 3 -W 2 "${ROV_IP}" >/dev/null; then
  echo "无法连通艇载电脑 ${ROV_IP}，禁止启动。" >&2
  exit 1
fi
for port in "${ROS_MAVLINK_PORT}" "${SOFTWARE_VIDEO_PORT}" "${RECORD_VIDEO_PORT}"; do
  ensure_free_port "${port}"
done
if ros2 node list 2>/dev/null | grep -Eq '^/(rov_vehicle_gateway|rov_dataset_drive|rov_autonomy|rov_search_perception|rov_search_approach_test)$'; then
  echo "检测到旧控制/自主节点，请先在原终端 Ctrl+C：" >&2
  ros2 node list 2>/dev/null | grep -E '^/(rov_vehicle_gateway|rov_dataset_drive|rov_autonomy|rov_search_perception|rov_search_approach_test)$' >&2
  exit 1
fi

SESSION_NAME="$(date +%Y%m%d_%H%M%S)"
SESSION_DIR="${PROJECT_DIR}/output/datasets/${SESSION_NAME}"
[[ ! -e "${SESSION_DIR}" ]] || SESSION_DIR="${SESSION_DIR}_$$"
mkdir -p "${SESSION_DIR}/logs"
start_qgc_if_needed

QGC_READY=false
for _ in {1..40}; do
  if qgc_owns_port "${QGC_MAVLINK_PORT}" && qgc_owns_port "${QGC_VIDEO_PORT}"; then
    QGC_READY=true
    break
  fi
  sleep 0.5
done
if [[ "${QGC_READY}" != true ]]; then
  echo >&2
  echo "QGC 未同时监听默认端口 ${QGC_MAVLINK_PORT} 和 ${QGC_VIDEO_PORT}。" >&2
  echo "请开启 QGC 的 UDP 自动连接、默认 5600 视频与 Low Latency。" >&2
  echo "同时确认 BlueOS 已向 ${TOPSIDE_IP} 发送 14550/14551 和 5600/5700。" >&2
  exit 1
fi

echo "============================================================"
echo "ROV 键盘数据集采集（无 YOLO、无机械爪）"
echo "  MAVLink: BlueOS -> QGC ${QGC_MAVLINK_PORT} + ROS ${ROS_MAVLINK_PORT}"
echo "  Video:   BlueOS -> QGC ${QGC_VIDEO_PORT} + Software ${SOFTWARE_VIDEO_PORT} -> MKV ${RECORD_VIDEO_PORT}"
echo "  Session: ${SESSION_DIR}"
echo "============================================================"

ros2 launch rov_competition telemetry.launch.py \
  robot_config:="${ROBOT_CONFIG}" \
  enable_actuation:=true \
  enable_ros_arming:=true \
  preflight_output_dir:="${SESSION_DIR}/logs/preflight" \
  >"${SESSION_DIR}/logs/gateway.log" 2>&1 &
GATEWAY_PID=$!
if ! timeout 45 bash -c 'until ros2 node list 2>/dev/null | grep -qx /rov_vehicle_gateway; do sleep 0.25; done'; then
  echo "飞控网关 45 秒内未就绪：" >&2
  tail -n 100 "${SESSION_DIR}/logs/gateway.log" >&2 || true
  exit 1
fi

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
  echo "视频分流启动失败：" >&2
  tail -n 80 "${SESSION_DIR}/logs/video_bridge.log" >&2 || true
  exit 1
fi

echo "后台就绪。窗口将检查录像、ALT_HOLD、链路与预检，然后要求确认词。"
set +e
ros2 run rov_competition rov_dataset_drive \
  --robot-config "${ROBOT_CONFIG}" \
  --dataset-config "${DATASET_CONFIG}" \
  --session-dir "${SESSION_DIR}" \
  --project-dir "${PROJECT_DIR}" \
  --record-port "${RECORD_VIDEO_PORT}" \
  --payload-type 96 \
  --execute
DRIVE_STATUS=$?
set -e
if ((DRIVE_STATUS != 0)); then
  echo "键盘采集工具以状态 ${DRIVE_STATUS} 退出。" >&2
fi
echo "采集文件：${SESSION_DIR}"
exit "${DRIVE_STATUS}"
