#!/usr/bin/env bash
set -euo pipefail

# 机械爪候选档案的独立上锁台架测试。
# 本脚本不启动 ROS 飞控网关、不解锁、不写飞控参数。

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
ROS_SETUP="/opt/ros/humble/setup.bash"
VENV_SETUP="${PROJECT_DIR}/.venv/bin/activate"
WORKSPACE_SETUP="${PROJECT_DIR}/ros2_ws/install/setup.bash"
ROBOT_CONFIG="${PROJECT_DIR}/config/robot.yaml"
ROV_IP="${ROV_IP:-192.168.2.2}"
TOPSIDE_IP="${TOPSIDE_IP:-192.168.2.1}"
ROS_MAVLINK_PORT=14551

usage() {
  cat <<'EOF'
用法：
  ./scripts/start_gripper_test.sh dalian
  ./scripts/start_gripper_test.sh rst

顺序必须是：断开推进器 -> dalian -> 上锁断电检查接线 -> rst。
RST 档案包含 500us，工具会要求第二确认词。
EOF
}

if (($# != 1)); then
  usage >&2
  exit 2
fi
PROFILE="$1"
if [[ "${PROFILE}" != "dalian" && "${PROFILE}" != "rst" ]]; then
  echo "档案只能是 dalian 或 rst。" >&2
  exit 2
fi

for required in "${ROS_SETUP}" "${VENV_SETUP}" "${WORKSPACE_SETUP}" "${ROBOT_CONFIG}"; do
  [[ -r "${required}" ]] || { echo "缺少必要文件：${required}" >&2; exit 1; }
done

set +u
source "${ROS_SETUP}"
source "${VENV_SETUP}"
source "${WORKSPACE_SETUP}"
set -u
export PYTHONNOUSERSITE=1
cd "${PROJECT_DIR}"

if ! ip -br -4 addr | grep -Eq "(^|[[:space:]])${TOPSIDE_IP}/24([[:space:]]|$)"; then
  echo "有线网卡未设为 ${TOPSIDE_IP}/24。" >&2
  exit 1
fi
ping -c 3 -W 2 "${ROV_IP}" >/dev/null \
  || { echo "无法连通艇载电脑 ${ROV_IP}。" >&2; exit 1; }

OWNER="$(ss -H -lunp 2>/dev/null | grep -E ":${ROS_MAVLINK_PORT}([[:space:]]|$)" || true)"
if [[ -n "${OWNER}" ]]; then
  echo "UDP ${ROS_MAVLINK_PORT} 已被占用；先关闭 ROS 飞控网关或旧测试：" >&2
  echo "${OWNER}" >&2
  exit 1
fi
if ros2 node list 2>/dev/null | grep -Eq '^/(rov_vehicle_gateway|rov_dataset_drive|rov_search_approach_test|rov_cluster_collection_test)$'; then
  echo "检测到旧控制节点，请先在原终端 Ctrl+C。" >&2
  exit 1
fi

SESSION_DIR="${PROJECT_DIR}/output/gripper_tests/$(date +%Y%m%d_%H%M%S)_${PROFILE}"
[[ ! -e "${SESSION_DIR}" ]] || SESSION_DIR="${SESSION_DIR}_$$"
mkdir -p "${SESSION_DIR}"

echo "============================================================"
echo "ROV 机械爪候选测试：${PROFILE}"
echo "  MAVLink: BlueOS -> ${TOPSIDE_IP}:${ROS_MAVLINK_PORT}"
echo "  飞控：必须上锁"
echo "  推进器：必须物理断开动力或信号"
echo "  记录：${SESSION_DIR}"
echo "============================================================"

exec ros2 run rov_competition rov_gripper_test "${PROFILE}" \
  --config "${ROBOT_CONFIG}" \
  --output-dir "${SESSION_DIR}" \
  --execute
