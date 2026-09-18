#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
CONFIG="${PROJECT_DIR}/config/blind_grab.local.yaml"
[[ -r "${CONFIG}" ]] || CONFIG="${PROJECT_DIR}/ros2_ws/src/rov_competition/config/blind_grab.yaml"
MODE="${1:---offline}"

if [[ "${MODE}" != "--offline" && "${MODE}" != "--live" && "${MODE}" != "--live-manual" ]]; then
  echo "用法：./scripts/check_official_ros.sh [--offline|--live|--live-manual]" >&2
  exit 2
fi

for setup in /opt/ros/humble/setup.bash "${PROJECT_DIR}/.venv/bin/activate" "${PROJECT_DIR}/ros2_ws/install/setup.bash"; do
  if [[ ! -r "${setup}" ]]; then
    echo "缺少环境文件：${setup}" >&2
    echo "请先运行：./scripts/install.sh" >&2
    exit 1
  fi
  set +u
  source "${setup}"
  set -u
done
export PYTHONNOUSERSITE=1
export PYTHONPATH="${PROJECT_DIR}/ros2_ws/src/rov_competition${PYTHONPATH:+:${PYTHONPATH}}"
cd "${PROJECT_DIR}"

readarray -t OFFICIAL_ENDPOINT < <(python - "${CONFIG}" <<'PY'
import sys
from rov_competition.blind_grab_config import load_blind_grab_config
settings = load_blind_grab_config(sys.argv[1]).official_ros
print(settings.server_ip)
print(settings.server_port)
print(str(settings.enabled).lower())
PY
)
SERVER_IP="${OFFICIAL_ENDPOINT[0]}"
SERVER_PORT="${OFFICIAL_ENDPOINT[1]}"
ENABLED="${OFFICIAL_ENDPOINT[2]}"

[[ "${ENABLED}" == "true" ]] || {
  echo "官方ROS转发在 ${CONFIG} 中被关闭。" >&2
  exit 1
}

ros2 interface show ros2_topic_forwarding/msg/RobotDataMessage >/dev/null
if ! ros2 pkg executables ros2_topic_forwarding | grep -q ' topic_forwarding$'; then
  echo "官方 ros2_topic_forwarding 未正确安装。请重新运行 ./scripts/install.sh" >&2
  exit 1
fi
python -m rov_competition.blind_grab_runtime --config "${CONFIG}" --dry-run >/dev/null
python -m rov_competition.official_data_only_runtime --config "${CONFIG}" --dry-run >/dev/null

case "${SERVER_PORT}" in
  40198) TEAM_LABEL="一队" ;;
  40197) TEAM_LABEL="二队（ddhyzx2）" ;;
  *) TEAM_LABEL="自定义端口" ;;
esac
VIDEO_URL="rtmp://${SERVER_IP}/ros/${SERVER_PORT}"

echo "官方ROS离线检查通过："
echo "  队伍：${TEAM_LABEL}"
echo "  消息：ros2_topic_forwarding/msg/RobotDataMessage"
echo "  节点：ros2_topic_forwarding topic_forwarding"
echo "  数据：${SERVER_IP}:${SERVER_PORT}"
echo "  视频：${VIDEO_URL}"

if [[ "${MODE}" == "--offline" ]]; then
  echo "盲抓启动后检查：./scripts/check_official_ros.sh --live"
  echo "手柄人工驾驶上传时检查：./scripts/check_official_ros.sh --live-manual"
  exit 0
fi

NODES="$(ros2 node list 2>/dev/null || true)"
if [[ "${MODE}" == "--live-manual" ]]; then
  DATA_NODE="/rov_official_data_only"
  DATA_LABEL="手柄人工驾驶数据专用"
else
  DATA_NODE="/rov_blind_grab_official_data"
  DATA_LABEL="盲抓"
fi

grep -qx "${DATA_NODE}" <<<"${NODES}" || {
  echo "未找到官方数据发布节点 ${DATA_NODE}" >&2
  exit 1
}
grep -qx '/topic_forwarding' <<<"${NODES}" || {
  echo "未找到官方TCP转发节点 /topic_forwarding" >&2
  exit 1
}

if [[ "${MODE}" == "--live-manual" ]]; then
  NODE_INFO="$(ros2 node info "${DATA_NODE}" 2>/dev/null || true)"
  if grep -Eq '(^|[[:space:]])/(cmd_vel|cmd_accel):' <<<"${NODE_INFO}"; then
    echo "${DATA_NODE} 意外创建了运动话题发布器，停止本次检查。" >&2
    exit 1
  fi
  echo "  OK：数据专用节点未发布 /cmd_vel 或 /cmd_accel"
  REQUIRED_TOPICS=(/robot_data)
  OPTIONAL_TOPICS=(/imu /magnetometer /pressure /joy /cmd_vel /cmd_accel)
else
  REQUIRED_TOPICS=(/cmd_vel /cmd_accel /robot_data)
  OPTIONAL_TOPICS=(/imu /magnetometer /pressure /joy)
fi

for topic in "${REQUIRED_TOPICS[@]}"; do
  if ! timeout 6 ros2 topic echo "${topic}" --once >/dev/null 2>&1; then
    echo "6秒内没有收到 ${topic}" >&2
    exit 1
  fi
  echo "  OK：${topic} 正在发布"
done

for topic in "${OPTIONAL_TOPICS[@]}"; do
  if timeout 2 ros2 topic echo "${topic}" --once >/dev/null 2>&1; then
    echo "  OK：${topic} 收到现有真实数据，官方节点会转发"
  else
    echo "  INFO：${topic} 当前无数据；出现真实遥测或ROS手柄/运动话题后才转发"
  fi
done

if command -v ss >/dev/null 2>&1; then
  if ! ss -H -tn state established 2>/dev/null | grep -Eq ":${SERVER_PORT}([[:space:]]|$)"; then
    echo "没有看到到 ${SERVER_IP}:${SERVER_PORT} 的 ESTABLISHED TCP 连接。" >&2
    echo "检查网络、队伍端口和主办方服务器状态。" >&2
    exit 1
  fi
  echo "  OK：TCP 已连接 ${SERVER_IP}:${SERVER_PORT}"
else
  echo "  提示：系统没有 ss，跳过TCP连接表检查。"
fi

echo "官方ROS现场链路检查通过（${DATA_LABEL}模式）。"
