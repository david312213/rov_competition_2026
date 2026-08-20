#!/usr/bin/env bash
set -euo pipefail

# 一键水池数据集采集：
#   BlueOS MAVLink 14550 -> MAVProxy -> ROS 14551 + QGC 14552
#   BlueOS RTP/H.264 5600 -> 原始包分流 -> QGC 5701 + 录像器 5702
#   ROS 飞控网关 + 键盘控制窗口
#
# 脚本不启动 YOLO，不读取权重，不开启机械爪。

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
ROS_SETUP="/opt/ros/humble/setup.bash"
VENV_SETUP="${PROJECT_DIR}/.venv/bin/activate"
WORKSPACE_SETUP="${PROJECT_DIR}/ros2_ws/install/setup.bash"
ROBOT_CONFIG="${PROJECT_DIR}/config/robot.yaml"
DATASET_CONFIG="${PROJECT_DIR}/config/dataset.yaml"
DATASET_EXAMPLE="${PROJECT_DIR}/ros2_ws/src/rov_competition/config/dataset.example.yaml"

ROV_IP="${ROV_IP:-192.168.2.2}"
MAVLINK_SOURCE_PORT=14550
ROS_MAVLINK_PORT=14551
QGC_MAVLINK_PORT=14552
VIDEO_SOURCE_PORT=5600
QGC_VIDEO_PORT=5701
RECORD_VIDEO_PORT=5702

MAVPROXY_PID=""
GATEWAY_PID=""
BRIDGE_PID=""
CLEANED_UP=false

usage() {
  printf '%s\n' \
    '用法：./scripts/start_dataset_collection.sh' \
    '' \
    '首次运行会生成 config/dataset.yaml：绝对上限 1.40 m，相对下潜上限 1.20 m。' \
    '可用 ROV_IP=... 临时替换默认艇载地址 192.168.2.2。'
}

if (($# > 0)); then
  case "$1" in
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "本脚本不接受其他参数：$1" >&2
      usage >&2
      exit 2
      ;;
  esac
fi

stop_owned_process() {
  local pid="$1"
  local label="$2"
  [[ -n "${pid}" ]] || return 0
  kill -0 "${pid}" 2>/dev/null || {
    wait "${pid}" 2>/dev/null || true
    return 0
  }

  kill -INT "${pid}" 2>/dev/null || true
  for _ in {1..50}; do
    kill -0 "${pid}" 2>/dev/null || break
    sleep 0.1
  done
  if kill -0 "${pid}" 2>/dev/null; then
    echo "${label} 未在 5 秒内退出，发送 SIGTERM。" >&2
    kill -TERM "${pid}" 2>/dev/null || true
  fi
  wait "${pid}" 2>/dev/null || true
}

cleanup() {
  local status=$?
  if [[ "${CLEANED_UP}" == true ]]; then
    return
  fi
  CLEANED_UP=true
  trap - EXIT INT TERM

  # 前台键盘节点已先完成上锁/急停。这里只回收本脚本启动的后台进程。
  stop_owned_process "${BRIDGE_PID}" "视频分流器"
  stop_owned_process "${GATEWAY_PID}" "飞控网关"
  stop_owned_process "${MAVPROXY_PID}" "MAVProxy"

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
  if [[ ! -r "${DATASET_EXAMPLE}" ]]; then
    echo "缺少数据集配置模板：${DATASET_EXAMPLE}" >&2
    exit 1
  fi
  cp -- "${DATASET_EXAMPLE}" "${DATASET_CONFIG}"
  echo "已生成：${DATASET_CONFIG}"
  echo "已按当前场地（水深至少 1.50 m）设置 maximum_depth_m=1.40。"
  echo "相对启动深度最多允许继续下潜 1.20 m。"
  echo "请打开文件核对场地后重新运行；更换水池必须重新填写。"
  exit 2
fi

# ROS 2 Humble 的环境脚本不保证兼容 nounset，只在 source 时临时关闭。
set +u
source "${ROS_SETUP}"
source "${VENV_SETUP}"
source "${WORKSPACE_SETUP}"
set -u
export PYTHONNOUSERSITE=1
cd "${PROJECT_DIR}"

for command in python ros2 gst-launch-1.0 ffprobe ss ping timeout mavproxy.py; do
  if ! command -v "${command}" >/dev/null 2>&1; then
    echo "缺少命令 ${command}；请重新运行 ./scripts/install.sh。" >&2
    exit 1
  fi
done
MAVPROXY_BIN="$(command -v mavproxy.py)"

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
    errors.append(
        f"robot.yaml 的 mavlink.connection_uri 必须是 ROS 独立端口 {expected_port}"
    )
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
  local port="$1"
  local owner
  owner="$(port_owner "${port}")"
  if [[ -n "${owner}" ]]; then
    echo "UDP ${port} 已被占用：" >&2
    echo "${owner}" >&2
    exit 1
  fi
}

ensure_qgc_or_free() {
  local port="$1"
  local owner
  owner="$(port_owner "${port}")"
  if [[ -n "${owner}" ]] && ! grep -Eiq 'qgroundcontrol' <<<"${owner}"; then
    echo "UDP ${port} 已被非 QGC 程序占用：" >&2
    echo "${owner}" >&2
    exit 1
  fi
}

for port in \
  "${MAVLINK_SOURCE_PORT}" \
  "${ROS_MAVLINK_PORT}" \
  "${VIDEO_SOURCE_PORT}" \
  "${RECORD_VIDEO_PORT}"; do
  ensure_free_port "${port}"
done
ensure_qgc_or_free "${QGC_MAVLINK_PORT}"
ensure_qgc_or_free "${QGC_VIDEO_PORT}"

if ros2 node list 2>/dev/null \
  | grep -Eq '^/(rov_vehicle_gateway|rov_dataset_drive|rov_autonomy)$'; then
  echo "检测到旧控制/自主节点，请先在原终端 Ctrl+C：" >&2
  ros2 node list 2>/dev/null \
    | grep -E '^/(rov_vehicle_gateway|rov_dataset_drive|rov_autonomy)$' >&2
  exit 1
fi

if ! ping -c 3 -W 2 "${ROV_IP}" >/dev/null; then
  echo "无法连通艇载电脑 ${ROV_IP}，禁止启动。" >&2
  exit 1
fi

SESSION_NAME="$(date +%Y%m%d_%H%M%S)"
SESSION_DIR="${PROJECT_DIR}/output/datasets/${SESSION_NAME}"
if [[ -e "${SESSION_DIR}" ]]; then
  SESSION_DIR="${SESSION_DIR}_$$"
fi
mkdir -p "${SESSION_DIR}/logs"

echo "============================================================"
echo "ROV 数据集采集（无 YOLO、无机械爪）"
echo "  MAVLink: ${MAVLINK_SOURCE_PORT} -> ROS ${ROS_MAVLINK_PORT} + QGC ${QGC_MAVLINK_PORT}"
echo "  Video:   ${VIDEO_SOURCE_PORT} -> QGC ${QGC_VIDEO_PORT} + MKV ${RECORD_VIDEO_PORT}"
echo "  Session: ${SESSION_DIR}"
echo "============================================================"

(
  cd "${SESSION_DIR}/logs"
  exec "${MAVPROXY_BIN}" \
    --master="udpin:0.0.0.0:${MAVLINK_SOURCE_PORT}" \
    --out="udpout:127.0.0.1:${ROS_MAVLINK_PORT}" \
    --out="udpout:127.0.0.1:${QGC_MAVLINK_PORT}" \
    --logfile="mavlink.tlog" \
    --non-interactive \
    --cmd="set mavfwd True"
) >"${SESSION_DIR}/logs/mavproxy.log" 2>&1 &
MAVPROXY_PID=$!
sleep 2
if ! kill -0 "${MAVPROXY_PID}" 2>/dev/null; then
  echo "MAVProxy 启动失败：" >&2
  tail -n 60 "${SESSION_DIR}/logs/mavproxy.log" >&2 || true
  exit 1
fi

if ! pgrep -f '[Qq][Gg]round[Cc]ontrol' >/dev/null 2>&1; then
  QGC_COMMAND="${QGC_EXECUTABLE:-}"
  if [[ -z "${QGC_COMMAND}" ]]; then
    QGC_COMMAND="$(command -v QGroundControl 2>/dev/null || true)"
  fi
  if [[ -z "${QGC_COMMAND}" ]]; then
    QGC_COMMAND="$(command -v qgroundcontrol 2>/dev/null || true)"
  fi
  if [[ -n "${QGC_COMMAND}" && -x "${QGC_COMMAND}" ]]; then
    "${QGC_COMMAND}" >"${SESSION_DIR}/logs/qgc.log" 2>&1 &
  elif command -v flatpak >/dev/null 2>&1 \
    && flatpak info org.mavlink.qgroundcontrol >/dev/null 2>&1; then
    flatpak run org.mavlink.qgroundcontrol \
      >"${SESSION_DIR}/logs/qgc.log" 2>&1 &
  else
    echo "请现在手动打开 QGroundControl。"
  fi
fi

echo
echo "请在 QGC 中确认："
echo "  - MAVLink UDP 监听端口 ${QGC_MAVLINK_PORT}"
echo "  - Video Source = UDP H.264，端口 ${QGC_VIDEO_PORT}，Low Latency 开启"
echo "  - QGC 可正常显示飞控，且可手动上锁"
if [[ ! -t 0 ]]; then
  echo "一键采集必须在交互终端中运行。" >&2
  exit 1
fi
read -r -p "配置完成后按 Enter 继续（尚不会解锁）……" _

QGC_MAV_OWNER="$(port_owner "${QGC_MAVLINK_PORT}")"
QGC_VIDEO_OWNER="$(port_owner "${QGC_VIDEO_PORT}")"
if ! grep -Eiq 'qgroundcontrol' <<<"${QGC_MAV_OWNER}"; then
  echo "未确认 QGC 监听 UDP ${QGC_MAVLINK_PORT}，禁止继续。" >&2
  echo "${QGC_MAV_OWNER}" >&2
  exit 1
fi
if ! grep -Eiq 'qgroundcontrol' <<<"${QGC_VIDEO_OWNER}"; then
  echo "未确认 QGC 监听视频 UDP ${QGC_VIDEO_PORT}，禁止继续。" >&2
  echo "${QGC_VIDEO_OWNER}" >&2
  exit 1
fi

# QGC 有时会保留一条默认 14550/5600 链路。多个进程即使能通过
# SO_REUSEADDR 同时绑定 UDP 端口，也可能分抢数据包。观察到 QGC
# 的正确目标端口后，再次严格检查专用端口。
MAV_SOURCE_OWNER="$(port_owner "${MAVLINK_SOURCE_PORT}")"
if ! grep -Fq "pid=${MAVPROXY_PID}," <<<"${MAV_SOURCE_OWNER}"; then
  echo "UDP ${MAVLINK_SOURCE_PORT} 不是由本次 MAVProxy 独立接收：" >&2
  echo "${MAV_SOURCE_OWNER}" >&2
  exit 1
fi
if grep -Eiq 'qgroundcontrol' <<<"${MAV_SOURCE_OWNER}"; then
  echo "QGC 仍在监听 ${MAVLINK_SOURCE_PORT}，请删除默认 14550 通信链路，只保留 ${QGC_MAVLINK_PORT}。" >&2
  exit 1
fi
for port in "${ROS_MAVLINK_PORT}" "${VIDEO_SOURCE_PORT}" "${RECORD_VIDEO_PORT}"; do
  ensure_free_port "${port}"
done

ros2 launch rov_competition telemetry.launch.py \
  robot_config:="${ROBOT_CONFIG}" \
  enable_actuation:=true \
  enable_ros_arming:=true \
  preflight_output_dir:="${SESSION_DIR}/logs/preflight" \
  >"${SESSION_DIR}/logs/gateway.log" 2>&1 &
GATEWAY_PID=$!

if ! timeout 45 bash -c '
  until ros2 node list 2>/dev/null | grep -qx /rov_vehicle_gateway; do
    sleep 0.25
  done
'; then
  echo "飞控网关 45 秒内未就绪：" >&2
  tail -n 100 "${SESSION_DIR}/logs/gateway.log" >&2 || true
  exit 1
fi
if ! kill -0 "${GATEWAY_PID}" 2>/dev/null; then
  echo "飞控网关已退出：" >&2
  tail -n 100 "${SESSION_DIR}/logs/gateway.log" >&2 || true
  exit 1
fi

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

echo
echo "后台就绪。下一步会检查录像、ALT_HOLD、深度和预检，然后要求确认词。"
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
