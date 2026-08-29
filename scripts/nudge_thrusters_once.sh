#!/usr/bin/env bash
set -euo pipefail

# 一键执行一次极低功率点动，用于临时打断电调的空闲蜂鸣计时。
# 脚本自己加载 ROS 环境、启动 MAVLink 网关、开许可、正常解锁、执行
# 1 秒点动，随后回中、上锁并关闭自己启动的网关。绝不后台循环。

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
ROS_SETUP="/opt/ros/humble/setup.bash"
VENV_SETUP="${PROJECT_DIR}/.venv/bin/activate"
WORKSPACE_SETUP="${PROJECT_DIR}/ros2_ws/install/setup.bash"
ROBOT_CONFIG="${PROJECT_DIR}/config/robot.yaml"

ROV_IP="${ROV_IP:-192.168.2.2}"
ROS_MAVLINK_PORT=14551
MOTION="${1:-forward}"
VALUE="0.05"
DURATION_S="1.0"
CONFIRMATION="NUDGE ROV ONCE"

GATEWAY_PID=""
REUSE_EXISTING_GATEWAY=false
CONTROL_TOUCHED=false
CLEANED_UP=false
SESSION_DIR="${PROJECT_DIR}/output/thruster_nudges/$(date +%Y%m%d_%H%M%S)"

usage() {
  cat <<'EOF'
用法：
  ./scripts/nudge_thrusters_once.sh [forward|backward|left|right|up|down]

例子：
  ./scripts/nudge_thrusters_once.sh forward

固定动作强度为 0.05，固定持续 1.0 秒。脚本会自行启动和关闭 ROS 飞控
网关，并完成运行许可、正常解锁、点动、回中和上锁；不会后台定时运行。

一次性前提：BlueOS 已向岸上电脑 UDP 14551 发送 MAVLink，robot.yaml
已经通过实艇预检并允许真实输出与 ROS 解锁。
EOF
}

if (($# > 1)); then
  usage >&2
  exit 2
fi
case "${MOTION}" in
  forward|backward|left|right|up|down) ;;
  -h|--help) usage; exit 0 ;;
  *)
    echo "方向错误：${MOTION}" >&2
    usage >&2
    exit 2
    ;;
esac

for required in "${ROS_SETUP}" "${VENV_SETUP}" "${WORKSPACE_SETUP}" "${ROBOT_CONFIG}"; do
  if [[ ! -r "${required}" ]]; then
    echo "缺少必要文件：${required}" >&2
    echo "请先完成环境安装、colcon 编译和 config/robot.yaml 配置。" >&2
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

for command in ros2 ss ping timeout; do
  if ! command -v "${command}" >/dev/null 2>&1; then
    echo "缺少命令 ${command}；请重新运行安装和 ROS 检查脚本。" >&2
    exit 1
  fi
done

mkdir -p "${SESSION_DIR}"

best_effort_lock() {
  # 无论主流程在哪一步失败，都尝试正常上锁并关闭运行时许可。
  if ros2 service list 2>/dev/null | grep -qx '/rov/control/set_armed'; then
    timeout 5s ros2 service call /rov/control/set_armed \
      rov_interfaces/srv/SetArmed \
      "{arm: false, confirmation: ''}" >/dev/null 2>&1 || true
  fi
  if ros2 service list 2>/dev/null | grep -qx '/rov/control/set_enabled'; then
    timeout 5s ros2 service call /rov/control/set_enabled \
      std_srvs/srv/SetBool "{data: false}" >/dev/null 2>&1 || true
  fi
}

stop_gateway() {
  [[ -n "${GATEWAY_PID}" ]] || return 0
  if kill -0 "${GATEWAY_PID}" 2>/dev/null; then
    kill -INT "${GATEWAY_PID}" 2>/dev/null || true
    for _ in {1..50}; do
      kill -0 "${GATEWAY_PID}" 2>/dev/null || break
      sleep 0.1
    done
    if kill -0 "${GATEWAY_PID}" 2>/dev/null; then
      kill -TERM "${GATEWAY_PID}" 2>/dev/null || true
    fi
  fi
  wait "${GATEWAY_PID}" 2>/dev/null || true
}

cleanup() {
  local status=$?
  if [[ "${CLEANED_UP}" == true ]]; then return; fi
  CLEANED_UP=true
  trap - EXIT INT TERM
  if [[ "${CONTROL_TOUCHED}" == true ]]; then
    best_effort_lock
  fi
  stop_gateway
  if [[ -n "${GATEWAY_PID}" ]]; then
    echo "日志：${SESSION_DIR}/gateway.log"
  elif [[ "${REUSE_EXISTING_GATEWAY}" == true ]]; then
    echo "已上锁并关闭本次许可；原有飞控网关继续运行。"
  fi
  return "${status}"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

call_service_expect_success() {
  local label="$1" service="$2" type="$3" request="$4" output
  if ! output="$(timeout 12s ros2 service call "${service}" "${type}" "${request}" 2>&1)"; then
    echo "${label}调用失败或超时：" >&2
    echo "${output}" >&2
    return 1
  fi
  echo "${output}"
  if ! grep -q 'success=True' <<<"${output}"; then
    echo "${label}被网关拒绝，未继续执行。" >&2
    return 1
  fi
}

if ! ping -c 2 -W 2 "${ROV_IP}" >/dev/null; then
  echo "无法连通艇载电脑 ${ROV_IP}；请先连接网线并恢复 192.168.2.1/24。" >&2
  exit 1
fi

ACTIVE_CONTROL_NODES="$(ros2 node list 2>/dev/null | grep -E \
  '^/(rov_dataset_drive|rov_search_approach_test|rov_cluster_collection_test)$' || true)"
if [[ -n "${ACTIVE_CONTROL_NODES}" ]]; then
  echo "发现 WASD/自主控制节点，不能同时点动：" >&2
  echo "${ACTIVE_CONTROL_NODES}" >&2
  exit 1
fi

if ros2 node list 2>/dev/null | grep -qx '/rov_vehicle_gateway'; then
  REUSE_EXISTING_GATEWAY=true
  echo "发现现有 /rov_vehicle_gateway；将在确认其已上锁且权限正确后复用。"
else
  PORT_OWNER="$(ss -H -lunp 2>/dev/null | grep -E ":${ROS_MAVLINK_PORT}([[:space:]]|$)" || true)"
  if [[ -n "${PORT_OWNER}" ]]; then
    echo "UDP ${ROS_MAVLINK_PORT} 被未知程序占用，无法安全启动网关：" >&2
    echo "${PORT_OWNER}" >&2
    exit 1
  fi
fi

cat <<EOF
============================================================
推进器一键低功率点动
  方向：${MOTION}
  指令：${VALUE}
  时间：${DURATION_S} 秒
  MAVLink：BlueOS -> UDP ${ROS_MAVLINK_PORT}
============================================================

它会真的驱动参与该动作的推进器，不能保证艇体完全不移动。
它只会暂时重置相关电调的空闲蜂鸣计时；其他电调仍可能叫，
并且到达电调自己的待机时间后，蜂鸣还会再次出现。

执行前必须同时满足：
  1. ROV 已完全浸没并可靠固定；
  2. 人、线缆和衣物远离全部桨叶；
  3. QGC 可立即人工上锁，安全员可立即断电；
  4. 飞控当前上锁，模式符合 robot.yaml，实艇参数已经通过预检；
  5. 当前没有 QGC 手柄、WASD 或自主节点发送控制命令。
EOF

printf '\n确认全部满足后，输入 %s：' "${CONFIRMATION}"
IFS= read -r answer
if [[ "${answer}" != "${CONFIRMATION}" ]]; then
  echo "确认词不匹配，未启动网关，也未发送任何运动命令。"
  exit 2
fi

GATEWAY_READY=false
if [[ "${REUSE_EXISTING_GATEWAY}" == true ]]; then
  echo "[1/5] 复用现有 ROS/MAVLink 飞控网关并核对状态……"
  if ros2 service list 2>/dev/null | grep -qx '/rov/control/set_enabled'; then
    GATEWAY_READY=true
  fi
else
  echo "[1/5] 启动 ROS/MAVLink 飞控网关并执行只读预检……"
  ros2 launch rov_competition telemetry.launch.py \
    robot_config:="${ROBOT_CONFIG}" \
    enable_actuation:=true \
    enable_ros_arming:=true \
    preflight_output_dir:="${SESSION_DIR}/preflight" \
    >"${SESSION_DIR}/gateway.log" 2>&1 &
  GATEWAY_PID=$!

  for _ in {1..100}; do
    if ! kill -0 "${GATEWAY_PID}" 2>/dev/null; then
      echo "飞控网关提前退出：" >&2
      tail -n 50 "${SESSION_DIR}/gateway.log" >&2 || true
      exit 1
    fi
    if ros2 service list 2>/dev/null | grep -qx '/rov/control/set_enabled'; then
      GATEWAY_READY=true
      break
    fi
    sleep 0.2
  done
fi
if [[ "${GATEWAY_READY}" != true ]]; then
  if [[ "${REUSE_EXISTING_GATEWAY}" == true ]]; then
    echo "现有网关节点没有提供控制服务；请在其原终端 Ctrl+C 后重试。" >&2
  else
    echo "20 秒内没有等到飞控网关服务；请查看 ${SESSION_DIR}/gateway.log" >&2
  fi
  exit 1
fi

STATUS_OUTPUT="$(timeout 5s ros2 topic echo /rov/control/status --once 2>&1 || true)"
echo "${STATUS_OUTPUT}"
for expected in \
  'configuration_allows_actuation: true' \
  'configuration_allows_ros_arming: true' \
  'preflight_passed: true' \
  'armed: false'; do
  if ! grep -q "${expected}" <<<"${STATUS_OUTPUT}"; then
    echo "网关状态不满足“${expected}”，拒绝点动。" >&2
    if [[ "${REUSE_EXISTING_GATEWAY}" == true ]]; then
      echo "若权限为 false，说明现有网关是只读启动；在原终端 Ctrl+C 后重新运行本脚本即可。" >&2
    fi
    exit 1
  fi
done

echo "[2/5] 开启本次运行许可……"
CONTROL_TOUCHED=true
call_service_expect_success \
  "运行许可" /rov/control/set_enabled std_srvs/srv/SetBool "{data: true}"

echo "[3/5] 使用普通安全命令正常解锁……"
call_service_expect_success \
  "ROS 解锁" /rov/control/set_armed rov_interfaces/srv/SetArmed \
  "{arm: true, confirmation: 'ARM ROV'}"

echo "[4/5] 执行 ${VALUE} / ${DURATION_S}s 的一次性 ${MOTION} 点动……"
ros2 run rov_competition rov_axis_test \
  --motion "${MOTION}" \
  --value "${VALUE}" \
  --duration "${DURATION_S}" \
  --config "${ROBOT_CONFIG}" \
  --execute \
  --confirm "MOVE ROV"

echo "[5/5] 点动完成；再次确认上锁、关闭许可并退出网关……"
best_effort_lock
echo "完成。"
