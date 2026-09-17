#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
CONFIG="${PROJECT_DIR}/config/blind_grab.local.yaml"
[[ -r "${CONFIG}" ]] || CONFIG="${PROJECT_DIR}/ros2_ws/src/rov_competition/config/blind_grab.yaml"
MODE="${1:---offline}"

if [[ "${MODE}" != "--offline" && "${MODE}" != "--live" ]]; then
  echo "用法：./scripts/check_official_ros.sh [--offline|--live]" >&2
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
  echo "正式启动后，另开终端运行：./scripts/check_official_ros.sh --live"
  exit 0
fi

NODES="$(ros2 node list 2>/dev/null || true)"
grep -qx '/rov_blind_grab_official_data' <<<"${NODES}" || {
  echo "未找到官方数据发布节点 /rov_blind_grab_official_data" >&2
  exit 1
}
grep -qx '/topic_forwarding' <<<"${NODES}" || {
  echo "未找到官方TCP转发节点 /topic_forwarding" >&2
  exit 1
}

for topic in /cmd_vel /cmd_accel /robot_data; do
  if ! timeout 6 ros2 topic echo "${topic}" --once >/dev/null 2>&1; then
    echo "6秒内没有收到 ${topic}" >&2
    exit 1
  fi
  echo "  OK：${topic} 正在发布"
done

for topic in /imu /magnetometer /pressure /joy; do
  if timeout 2 ros2 topic echo "${topic}" --once >/dev/null 2>&1; then
    echo "  OK：${topic} 收到真实数据"
  else
    echo "  INFO：${topic} 当前无数据；对应真实遥测/手柄出现后才发布"
  fi
done

if command -v ss >/dev/null 2>&1; then
  if ! ss -H -tn state established 2>/dev/null | grep -Eq ":${SERVER_PORT}([[:space:]]|$)"; then
    echo "没有看到到 ${SERVER_IP}:${SERVER_PORT} 的 ESTABLISHED TCP 连接。" >&2
    echo "盲抓仍会继续，但平台计分数据尚未确认送达；检查网络和平台端口。" >&2
    exit 1
  fi
  echo "  OK：TCP 已连接 ${SERVER_IP}:${SERVER_PORT}"
else
  echo "  提示：系统没有 ss，跳过TCP连接表检查。"
fi

echo "官方ROS现场链路检查通过。"
