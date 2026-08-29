#!/usr/bin/env bash
set -euo pipefail

# 0.2.0rc2 搜索—接近/抓取位置标定共用的一键入口。
# QGC 直接使用 14550/5600；ROS 网关使用 14551；软件视频从
# 5700 原样复制到 YOLO 5702 和录像器 5704，不需要额外的 MAVLink 转发进程。

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
ROS_SETUP="/opt/ros/humble/setup.bash"
VENV_SETUP="${PROJECT_DIR}/.venv/bin/activate"
WORKSPACE_SETUP="${PROJECT_DIR}/ros2_ws/install/setup.bash"
ROBOT_CONFIG="${PROJECT_DIR}/config/robot.yaml"
DATASET_CONFIG="${PROJECT_DIR}/config/dataset.yaml"
DATASET_EXAMPLE="${PROJECT_DIR}/ros2_ws/src/rov_competition/config/dataset.example.yaml"
AUTONOMY_TEMPLATE="${PROJECT_DIR}/ros2_ws/src/rov_competition/config/autonomy.yaml"
AUTONOMY_LOCAL="${PROJECT_DIR}/config/autonomy.local.yaml"
AUTONOMY_CONFIG="${AUTONOMY_CONFIG:-${AUTONOMY_TEMPLATE}}"
TARGETS_CONFIG="${PROJECT_DIR}/ros2_ws/src/rov_competition/config/targets.yaml"
SEARCH_CONFIG="${PROJECT_DIR}/ros2_ws/src/rov_competition/config/search_test.yaml"
CLUSTER_CONFIG="${PROJECT_DIR}/ros2_ws/src/rov_competition/config/cluster_collection.yaml"

ROV_IP="${ROV_IP:-192.168.2.2}"
TOPSIDE_IP="${TOPSIDE_IP:-192.168.2.1}"
QGC_MAVLINK_PORT=14550
ROS_MAVLINK_PORT=14551
QGC_VIDEO_PORT=5600
SOFTWARE_VIDEO_PORT=5700
YOLO_VIDEO_PORT=5702
RECORD_VIDEO_PORT=5704
WORKFLOW="auto_approach"

GATEWAY_PID=""
BRIDGE_PID=""
PERCEPTION_PID=""
VIEWER_PID=""
CLEANED_UP=false

usage() {
  printf '%s\n' \
    '用法：./scripts/start_search_approach_test.sh [--workflow auto_approach|manual_grasp_calibration|cluster_collection]' \
    '' \
    'auto_approach：自动下潜、扫描、对准、接近，然后回收。' \
    'manual_grasp_calibration：自动对准后停车，进入 WASD 抓取位置标定。' \
    'cluster_collection：触底、离底搜索、群体靠近和每群三次盲抓。' \
    '三个模式都会在解锁前询问下潜 power 和确认词。' \
    '单目标流程离底 0.10m；群体收集流程每次重新触底后离底 0.15m。'
}
while (($# > 0)); do
  case "$1" in
    --workflow)
      (($# >= 2)) || { echo "--workflow 缺少取值" >&2; exit 2; }
      WORKFLOW="$2"
      shift 2
      ;;
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
done
if [[ "${WORKFLOW}" != "auto_approach" && "${WORKFLOW}" != "manual_grasp_calibration" && "${WORKFLOW}" != "cluster_collection" ]]; then
  echo "未知工作流：${WORKFLOW}" >&2
  exit 2
fi

stop_owned_process() {
  local pid="$1" label="$2"
  [[ -n "${pid}" ]] || return 0
  if kill -0 "${pid}" 2>/dev/null; then
    kill -INT "${pid}" 2>/dev/null || true
    for _ in {1..70}; do kill -0 "${pid}" 2>/dev/null || break; sleep 0.1; done
    if kill -0 "${pid}" 2>/dev/null; then
      echo "${label} 未正常退出，发送 SIGTERM。" >&2
      kill -TERM "${pid}" 2>/dev/null || true
    fi
  fi
  wait "${pid}" 2>/dev/null || true
}
cleanup() {
  local status=$?
  [[ "${CLEANED_UP}" == false ]] || return
  CLEANED_UP=true
  trap - EXIT INT TERM
  # 前台测试节点已经先回中并上锁/急停。
  stop_owned_process "${VIEWER_PID}" "YOLO 带框查看器"
  stop_owned_process "${PERCEPTION_PID}" "YOLO 感知"
  stop_owned_process "${BRIDGE_PID}" "视频分流器"
  stop_owned_process "${GATEWAY_PID}" "飞控网关"
  echo
  echo "搜索测试后台进程已停止；QGC 继续由操作员管理。"
  return "${status}"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

for required in \
  "${ROS_SETUP}" "${VENV_SETUP}" "${WORKSPACE_SETUP}" "${ROBOT_CONFIG}" \
  "${AUTONOMY_TEMPLATE}" "${TARGETS_CONFIG}" "${SEARCH_CONFIG}" "${CLUSTER_CONFIG}"; do
  [[ -r "${required}" ]] || { echo "缺少必要文件：${required}" >&2; exit 1; }
done
if [[ ! -r "${DATASET_CONFIG}" ]]; then
  cp -- "${DATASET_EXAMPLE}" "${DATASET_CONFIG}"
  echo "已生成 ${DATASET_CONFIG}；请确认参数后重新运行。"
  exit 2
fi

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
for command in python ros2 gst-launch-1.0 ffprobe ss ping ip timeout; do
  command -v "${command}" >/dev/null 2>&1 || { echo "缺少命令 ${command}" >&2; exit 1; }
done
if ! ros2 pkg prefix rqt_image_view >/dev/null 2>&1; then
  echo "缺少 YOLO 带框查看器 rqt_image_view。" >&2
  echo "请先执行：sudo apt install ros-humble-rqt-image-view" >&2
  exit 1
fi
if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
  echo "当前没有图形桌面环境，无法打开 YOLO 带框查看器。" >&2
  exit 1
fi

python - <<'PY'
import cv2
import numpy
import pygame
import torch
import ultralytics
if int(numpy.__version__.split(".", 1)[0]) >= 2:
    raise RuntimeError(f"现场环境要求 NumPy 1.x，当前 {numpy.__version__}")
if not torch.cuda.is_available():
    raise RuntimeError("搜索测试必须使用 CUDA GPU，不允许静默退回 CPU")
print(f"视觉环境正常：OpenCV {cv2.__version__}, GPU {torch.cuda.get_device_name(0)}")
print(f"操作窗口正常：Pygame {pygame.version.ver}")
PY

ros2 run rov_competition rov_field_setup \
  --project-dir "${PROJECT_DIR}" weights verify

if ! ros2 run rov_competition rov_field_setup \
  --project-dir "${PROJECT_DIR}" timing verify; then
  echo "活动自主配置仍是旧版；请先运行：" >&2
  echo "  ./scripts/start_tomorrow_test.sh timing" >&2
  echo "按提示输入 APPLY BALANCED TIMEOUTS 后，再重新启动本测试。" >&2
  exit 2
fi

port_owner() { ss -H -lunp 2>/dev/null | grep -E ":$1([[:space:]]|$)" || true; }
ensure_free_port() {
  local owner
  owner="$(port_owner "$1")"
  [[ -z "${owner}" ]] || { echo "UDP $1 已被占用：" >&2; echo "${owner}" >&2; exit 1; }
}
qgc_owns_port() { grep -Eiq 'qgroundcontrol' <<<"$(port_owner "$1")"; }

if ! ip -br -4 addr | grep -Eq "(^|[[:space:]])${TOPSIDE_IP}/24([[:space:]]|$)"; then
  echo "有线网卡未设为 ${TOPSIDE_IP}/24。" >&2
  exit 1
fi
ping -c 3 -W 2 "${ROV_IP}" >/dev/null || { echo "无法连通 ${ROV_IP}" >&2; exit 1; }
for port in "${ROS_MAVLINK_PORT}" "${SOFTWARE_VIDEO_PORT}" "${YOLO_VIDEO_PORT}" "${RECORD_VIDEO_PORT}"; do
  ensure_free_port "${port}"
done
if ros2 node list 2>/dev/null | grep -Eq '^/(rov_vehicle_gateway|rov_dataset_drive|rov_autonomy|rov_search_perception|rov_search_approach_test|rov_cluster_collection_test)$'; then
  echo "检测到旧的飞控/感知/控制节点，请先 Ctrl+C。" >&2
  exit 1
fi

SESSION_NAME="$(date +%Y%m%d_%H%M%S)"
if [[ "${WORKFLOW}" == "manual_grasp_calibration" ]]; then
  SESSION_ROOT="${PROJECT_DIR}/output/grasp_tests"
  MODE_TITLE="抓取位置标定"
elif [[ "${WORKFLOW}" == "cluster_collection" ]]; then
  SESSION_ROOT="${PROJECT_DIR}/output/cluster_collection_tests"
  MODE_TITLE="群体盲抓与跳跃式离底搜索"
else
  SESSION_ROOT="${PROJECT_DIR}/output/search_tests"
  MODE_TITLE="搜索—自动接近"
fi
SESSION_DIR="${SESSION_ROOT}/${SESSION_NAME}"
[[ ! -e "${SESSION_DIR}" ]] || SESSION_DIR="${SESSION_DIR}_$$"
mkdir -p "${SESSION_DIR}/logs"
RUNTIME_ROBOT_CONFIG="${SESSION_DIR}/resolved_robot.yaml"
if [[ "${WORKFLOW}" == "manual_grasp_calibration" || "${WORKFLOW}" == "cluster_collection" ]]; then
  RUNTIME_MODE="manual"
else
  RUNTIME_MODE="auto"
fi
ros2 run rov_competition rov_field_setup \
  --project-dir "${PROJECT_DIR}" runtime-config \
  --mode "${RUNTIME_MODE}" \
  --output "${RUNTIME_ROBOT_CONFIG}" \
  | tee "${SESSION_DIR}/logs/runtime_config.log"

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
  if qgc_owns_port "${QGC_MAVLINK_PORT}" && qgc_owns_port "${QGC_VIDEO_PORT}"; then QGC_READY=true; break; fi
  sleep 0.5
done
if [[ "${QGC_READY}" != true ]]; then
  echo "QGC 未同时监听默认 ${QGC_MAVLINK_PORT}/${QGC_VIDEO_PORT}。" >&2
  echo "请开启 QGC 的 UDP 自动连接、默认 5600 视频与 Low Latency。" >&2
  exit 1
fi

echo "============================================================"
echo "ROV 0.2.0rc2 ${MODE_TITLE}水池测试"
echo "  MAVLink: QGC ${QGC_MAVLINK_PORT} | ROS ${ROS_MAVLINK_PORT}"
echo "  Video:   QGC ${QGC_VIDEO_PORT} | Software ${SOFTWARE_VIDEO_PORT} -> YOLO ${YOLO_VIDEO_PORT} + MKV ${RECORD_VIDEO_PORT}"
echo "  Output:  ${SESSION_DIR}"
echo "============================================================"

ros2 launch rov_competition telemetry.launch.py \
  robot_config:="${RUNTIME_ROBOT_CONFIG}" \
  enable_actuation:=true \
  enable_ros_arming:=true \
  preflight_output_dir:="${SESSION_DIR}/logs/preflight" \
  >"${SESSION_DIR}/logs/gateway.log" 2>&1 &
GATEWAY_PID=$!
if ! timeout 45 bash -c 'until ros2 node list 2>/dev/null | grep -qx /rov_vehicle_gateway; do sleep 0.25; done'; then
  tail -n 100 "${SESSION_DIR}/logs/gateway.log" >&2 || true
  exit 1
fi

ros2 run rov_competition rov_stream_bridge \
  --source-port "${SOFTWARE_VIDEO_PORT}" \
  --no-qgc \
  --inference-host 127.0.0.1 \
  --inference-port "${YOLO_VIDEO_PORT}" \
  --record-host 127.0.0.1 \
  --record-port "${RECORD_VIDEO_PORT}" \
  --no-display \
  >"${SESSION_DIR}/logs/video_bridge.log" 2>&1 &
BRIDGE_PID=$!
sleep 2
kill -0 "${BRIDGE_PID}" 2>/dev/null || { tail -n 80 "${SESSION_DIR}/logs/video_bridge.log" >&2; exit 1; }

ros2 launch rov_competition perception_only.launch.py \
  robot_config:="${RUNTIME_ROBOT_CONFIG}" \
  autonomy_config:="${AUTONOMY_CONFIG}" \
  targets_config:="${TARGETS_CONFIG}" \
  ai_port:="${YOLO_VIDEO_PORT}" \
  >"${SESSION_DIR}/logs/perception.log" 2>&1 &
PERCEPTION_PID=$!
if ! timeout 45 bash -c 'until ros2 node list 2>/dev/null | grep -qx /rov_search_perception; do sleep 0.25; done'; then
  tail -n 100 "${SESSION_DIR}/logs/perception.log" >&2 || true
  exit 1
fi

# rqt_image_view 的插件参数使用“基础话题 传输方式”。直接选中
# compressed，避免队员每次手工选择话题，也避免把 transport-specific
# 话题当成普通 Image 话题订阅。
if ! timeout 30 bash -c \
  'until ros2 topic list 2>/dev/null | grep -qx /rov/annotated_image/compressed; do sleep 0.25; done'; then
  echo "30 秒内未发现 YOLO 带框图像话题。" >&2
  tail -n 100 "${SESSION_DIR}/logs/perception.log" >&2 || true
  exit 1
fi
if [[ "${WORKFLOW}" == "cluster_collection" ]]; then
  VIEW_BASE_TOPIC="/rov/cluster_image"
  # 群体叠加发布者由后面的前台测试节点创建。查看器先在
  # 后台等待该话题出现，避免话题还不存在时 rqt 直接退出。
  bash -c '
    topic="$1"
    until ros2 topic list 2>/dev/null | grep -qx "${topic}/compressed"; do
      sleep 0.25
    done
    exec ros2 run rqt_image_view rqt_image_view "${topic} compressed"
  ' _ "${VIEW_BASE_TOPIC}" \
    >"${SESSION_DIR}/logs/annotated_viewer.log" 2>&1 &
  VIEW_TOPIC="${VIEW_BASE_TOPIC} compressed"
else
  VIEW_BASE_TOPIC="/rov/annotated_image"
  VIEW_TOPIC="/rov/annotated_image compressed"
  ros2 run rqt_image_view rqt_image_view \
    "${VIEW_TOPIC}" \
    >"${SESSION_DIR}/logs/annotated_viewer.log" 2>&1 &
fi
VIEWER_PID=$!
sleep 2
if ! kill -0 "${VIEWER_PID}" 2>/dev/null; then
  echo "YOLO 带框查看器启动失败。" >&2
  tail -n 100 "${SESSION_DIR}/logs/annotated_viewer.log" >&2 || true
  exit 1
fi
if [[ "${WORKFLOW}" == "cluster_collection" ]]; then
  echo "群体节点就绪后将自动打开带框画面：${VIEW_TOPIC}"
else
  echo "YOLO 带框画面已自动打开：${VIEW_TOPIC}"
fi
echo "QGC 继续显示 5600 原始画面；关闭带框窗口不会中止任务。"

set +e
if [[ "${WORKFLOW}" == "cluster_collection" ]]; then
  ros2 run rov_competition rov_cluster_collection_test \
    --robot-config "${RUNTIME_ROBOT_CONFIG}" \
    --dataset-config "${DATASET_CONFIG}" \
    --autonomy-config "${AUTONOMY_CONFIG}" \
    --cluster-config "${CLUSTER_CONFIG}" \
    --session-dir "${SESSION_DIR}" \
    --project-dir "${PROJECT_DIR}" \
    --record-port "${RECORD_VIDEO_PORT}" \
    --execute
  TEST_STATUS=$?
else
  ros2 run rov_competition rov_search_approach_test \
    --robot-config "${RUNTIME_ROBOT_CONFIG}" \
    --dataset-config "${DATASET_CONFIG}" \
    --autonomy-config "${AUTONOMY_CONFIG}" \
    --targets-config "${TARGETS_CONFIG}" \
    --search-config "${SEARCH_CONFIG}" \
    --session-dir "${SESSION_DIR}" \
    --project-dir "${PROJECT_DIR}" \
    --record-port "${RECORD_VIDEO_PORT}" \
    --workflow "${WORKFLOW}" \
    --execute
  TEST_STATUS=$?
fi
set -e
echo "测试文件：${SESSION_DIR}"
exit "${TEST_STATUS}"
