#!/usr/bin/env bash
set -euo pipefail

# 0.2.0rc2 明日联调总入口。这是交互式向导，不会把配置、爪子、
# 自动运动连成一条无人值守的危险流程。每个阶段都要单独确认。

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
PACKAGE_SOURCE="${PROJECT_DIR}/ros2_ws/src/rov_competition"
ROS_SETUP="/opt/ros/humble/setup.bash"
VENV_SETUP="${PROJECT_DIR}/.venv/bin/activate"
WORKSPACE_SETUP="${PROJECT_DIR}/ros2_ws/install/setup.bash"

ROV_IP="${ROV_IP:-192.168.2.2}"
TOPSIDE_IP="${TOPSIDE_IP:-192.168.2.1}"

usage() {
  cat <<'EOF'
用法：
  ./scripts/start_tomorrow_test.sh
  ./scripts/start_tomorrow_test.sh status
  ./scripts/start_tomorrow_test.sh qgc
  ./scripts/start_tomorrow_test.sh gripper [dalian|rst]
  ./scripts/start_tomorrow_test.sh weights [rollback]
  ./scripts/start_tomorrow_test.sh timing
  ./scripts/start_tomorrow_test.sh search
  ./scripts/start_tomorrow_test.sh calibrate

无参数时显示中文菜单。这个向导不会关闭 QGroundControl。
EOF
}

port_owner() {
  ss -H -lunp 2>/dev/null | grep -E ":$1([[:space:]]|$)" || true
}

require_free_port() {
  local port="$1" owner
  owner="$(port_owner "${port}")"
  if [[ -n "${owner}" ]]; then
    printf 'UDP %s 已被占用：\n%s\n' "${port}" "${owner}" >&2
    return 1
  fi
}

qgc_owns_port() {
  grep -Eiq 'qgroundcontrol' <<<"$(port_owner "$1")"
}

load_project_environment() {
  local required
  for required in "${ROS_SETUP}" "${VENV_SETUP}" "${WORKSPACE_SETUP}"; do
    if [[ ! -r "${required}" ]]; then
      printf '缺少环境文件：%s\n' "${required}" >&2
      printf '请先在 Ubuntu 22.04 运行 ./scripts/install.sh 和 ./scripts/check_ros.sh。\n' >&2
      return 1
    fi
  done
  set +u
  source "${ROS_SETUP}"
  source "${VENV_SETUP}"
  source "${WORKSPACE_SETUP}"
  set -u
  export PYTHONNOUSERSITE=1
  export PYTHONPATH="${PACKAGE_SOURCE}${PYTHONPATH:+:${PYTHONPATH}}"
  cd "${PROJECT_DIR}"
}

field_setup() {
  python -m rov_competition.field_setup --project-dir "${PROJECT_DIR}" "$@"
}

start_qgc_if_needed() {
  if pgrep -f '[Qq][Gg]round[Cc]ontrol' >/dev/null 2>&1; then
    printf 'QGroundControl 已在运行。\n'
    return 0
  fi
  local executable="${QGC_EXECUTABLE:-}"
  [[ -n "${executable}" ]] || executable="$(command -v QGroundControl 2>/dev/null || true)"
  [[ -n "${executable}" ]] || executable="$(command -v qgroundcontrol 2>/dev/null || true)"
  if [[ -n "${executable}" && -x "${executable}" ]]; then
    nohup "${executable}" >"${XDG_RUNTIME_DIR:-/tmp}/rov-qgc.log" 2>&1 &
    disown || true
  elif command -v flatpak >/dev/null 2>&1 \
    && flatpak info org.mavlink.qgroundcontrol >/dev/null 2>&1; then
    nohup flatpak run org.mavlink.qgroundcontrol \
      >"${XDG_RUNTIME_DIR:-/tmp}/rov-qgc.log" 2>&1 &
    disown || true
  else
    printf '没有自动找到 QGroundControl，请手动打开。\n'
  fi
}

check_topside_network() {
  if ! ip -br -4 addr | grep -Eq "(^|[[:space:]])${TOPSIDE_IP}/24([[:space:]]|$)"; then
    printf '岸上有线网卡不是 %s/24。\n' "${TOPSIDE_IP}" >&2
    return 1
  fi
  if ! ping -c 3 -W 2 "${ROV_IP}" >/dev/null; then
    printf '无法连通 BlueOS %s，先检查网线和以太网地址。\n' "${ROV_IP}" >&2
    return 1
  fi
}

receive_one_udp_packet() {
  local port="$1" label="$2"
  if python3 - "${port}" <<'PY'
import socket
import sys

port = int(sys.argv[1])
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind(("0.0.0.0", port))
sock.settimeout(4.0)
try:
    payload, _ = sock.recvfrom(65535)
except socket.timeout:
    raise SystemExit(1)
finally:
    sock.close()
raise SystemExit(0 if payload else 1)
PY
  then
    printf '  OK：%s UDP %s 收到数据。\n' "${label}" "${port}"
  else
    printf '  失败：%s UDP %s 在4秒内没有数据。\n' "${label}" "${port}" >&2
    return 1
  fi
}

show_status() {
  printf 'ROV 0.2.0rc2 明日联调状态\n'
  printf '工程：%s\n' "${PROJECT_DIR}"
  if command -v git >/dev/null 2>&1; then
    git -C "${PROJECT_DIR}" status --short --branch || true
    git -C "${PROJECT_DIR}" log -1 --oneline || true
  fi
  printf '网络：\n'
  ip -br -4 addr 2>/dev/null || true
  if ping -c 1 -W 1 "${ROV_IP}" >/dev/null 2>&1; then
    printf '  BlueOS %s：可达\n' "${ROV_IP}"
  else
    printf '  BlueOS %s：不可达\n' "${ROV_IP}"
  fi
  printf '端口：\n'
  local port owner
  for port in 14550 14551 5600 5700 5702 5704; do
    owner="$(port_owner "${port}")"
    if [[ -n "${owner}" ]]; then
      printf '  %s：%s\n' "${port}" "${owner}"
    else
      printf '  %s：未占用\n' "${port}"
    fi
  done
  if load_project_environment 2>/dev/null; then
    printf '本地激活记录：\n'
    field_setup status || true
  else
    printf '本地激活记录：环境未完成，暂时无法检查。\n'
  fi
}

configure_qgc() {
  for command in ip ping ss python3; do
    command -v "${command}" >/dev/null 2>&1 \
      || { printf '缺少命令 %s。\n' "${command}" >&2; return 1; }
  done
  printf '%s\n' \
    'BlueOS 只需配置一次：' \
    '  MAVLink UDP Client  -> 192.168.2.1:14550（QGC）' \
    '  MAVLink UDP Client  -> 192.168.2.1:14551（ROS）' \
    '  删除/停用旧的 14552 Endpoint' \
    '  相同摄像头 UDP -> udp://192.168.2.1:5600（QGC）' \
    '  相同摄像头 UDP -> udp://192.168.2.1:5700（软件）' \
    '' \
    'QGC 只需配置一次：' \
    '  删除手动 14552 Comm Link，开启 UDP 自动连接' \
    '  Video Source = UDP h.264，URL = 0.0.0.0:5600' \
    '  开启 Low Latency Mode'

  check_topside_network
  if command -v xdg-open >/dev/null 2>&1; then
    xdg-open "http://${ROV_IP}" >/dev/null 2>&1 &
  fi
  start_qgc_if_needed
  printf '\n配置完后，请确认 QGC 有实时遥测、实时画面和人工上锁能力。\n'
  local confirmation
  read -r -p "完整输入 QGC READY 继续: " confirmation
  [[ "${confirmation}" == "QGC READY" ]] \
    || { printf '未确认 QGC，本阶段停止。\n'; return 2; }

  if ! qgc_owns_port 14550; then
    printf 'QGC 没有监听默认 MAVLink UDP 14550。\n' >&2
    return 1
  fi
  if ! qgc_owns_port 5600; then
    printf 'QGC 没有监听默认视频 UDP 5600。\n' >&2
    return 1
  fi
  local port
  for port in 14551 5700 5702 5704; do
    require_free_port "${port}"
  done
  receive_one_udp_packet 14551 'ROS MAVLink 源'
  receive_one_udp_packet 5700 '软件视频源'
  printf 'QGC/BlueOS 默认端口验证通过；本阶段没有启动 ROS 或控制机器人。\n'
}

test_and_activate_gripper() {
  load_project_environment
  local profile="${1:-}"
  if [[ -z "${profile}" ]]; then
    printf '%s\n' '1) dalian（先测）' '2) rst（仅在断电核对接线后测）'
    read -r -p "选择 [1/2]: " profile
    case "${profile}" in
      1) profile=dalian ;;
      2) profile=rst ;;
      *) printf '无效选择。\n' >&2; return 2 ;;
    esac
  fi
  [[ "${profile}" == "dalian" || "${profile}" == "rst" ]] \
    || { printf '机械爪档案只能是 dalian 或 rst。\n' >&2; return 2; }

  printf '\n开始前必须：飞控上锁，推进器动力或信号物理断开。\n'
  set +e
  "${SCRIPT_DIR}/start_gripper_test.sh" "${profile}"
  local test_status=$?
  set -e
  if ((test_status != 0)); then
    printf '机械爪测试未通过（退出码 %s），robot.yaml 不会修改。\n' "${test_status}" >&2
    return "${test_status}"
  fi

  local result confirmation expected
  result="$(field_setup gripper latest --profile "${profile}")"
  field_setup gripper activate --profile "${profile}" --result "${result}"
  if [[ "${profile}" == "dalian" ]]; then
    expected='ACTIVATE DALIAN GRIPPER'
  else
    expected='ACTIVATE RST GRIPPER'
  fi
  printf '通过记录：%s\n' "${result}"
  read -r -p "若要备份并启用该档案，完整输入 ${expected}: " confirmation
  if [[ "${confirmation}" != "${expected}" ]]; then
    printf '未执行激活；测试记录已保留。\n'
    return 0
  fi
  field_setup gripper activate \
    --profile "${profile}" --result "${result}" \
    --confirmation "${confirmation}" --execute
}

manage_weights() {
  load_project_environment
  local action="${1:-install}"
  if [[ "${action}" == "rollback" ]]; then
    field_setup weights rollback
    local confirmation
    read -r -p "确认交换当前/上一份权重，完整输入 ROLLBACK WEIGHT: " confirmation
    [[ "${confirmation}" == 'ROLLBACK WEIGHT' ]] \
      || { printf '已取消回滚。\n'; return 0; }
    field_setup weights rollback --confirmation "${confirmation}" --execute
    return
  fi
  [[ "${action}" == "install" ]] \
    || { printf 'weights 只支持 install 或 rollback。\n' >&2; return 2; }

  printf '注意：.pt 可能包含 Python pickle，只能选择你们自己训练或确认可信的文件。\n'
  local source confirmation
  read -r -p "请拖入或输入新 best.pt 完整路径: " source
  source="${source#\'}"; source="${source%\'}"
  source="${source#\"}"; source="${source%\"}"
  source="${source//\\ / }"
  field_setup weights install --source "${source}"
  read -r -p "验证通过。完整输入 INSTALL NEW WEIGHT 替换: " confirmation
  [[ "${confirmation}" == 'INSTALL NEW WEIGHT' ]] \
    || { printf '已取消替换，旧权重未改变。\n'; return 0; }
  field_setup weights install --source "${source}" \
    --confirmation "${confirmation}" --execute
}

apply_balanced_timing() {
  load_project_environment
  printf '%s\n' \
    '这个阶段只更新本机 Git 忽略的 robot.yaml 和 dataset.yaml：' \
    '  - 放宽通信、遥测和感知抖动阈值' \
    '  - 保持 0.50s 运动发布者看门狗' \
    '  - 不连接飞控，不修改任何 ArduSub 参数'
  field_setup timing preview
  local confirmation
  read -r -p \
    "确认备份并应用，完整输入 APPLY BALANCED TIMEOUTS: " confirmation
  if [[ "${confirmation}" != 'APPLY BALANCED TIMEOUTS' ]]; then
    printf '已取消；本地配置没有改变。\n'
    return 0
  fi
  field_setup timing apply \
    --confirmation "${confirmation}" --execute
  field_setup timing verify
}

run_search() {
  load_project_environment
  exec "${SCRIPT_DIR}/start_search_approach_test.sh"
}

run_calibration() {
  load_project_environment
  exec "${SCRIPT_DIR}/start_grasp_position_test.sh"
}

dispatch() {
  local command="${1:-}"
  shift || true
  case "${command}" in
    status) show_status ;;
    qgc) configure_qgc ;;
    gripper) test_and_activate_gripper "${1:-}" ;;
    weights) manage_weights "${1:-install}" ;;
    timing) apply_balanced_timing ;;
    search) run_search ;;
    calibrate) run_calibration ;;
    -h|--help|help) usage ;;
    *) printf '未知子命令：%s\n' "${command}" >&2; usage >&2; return 2 ;;
  esac
}

if (($# > 0)); then
  dispatch "$@"
  exit $?
fi

while true; do
  printf '\n%s\n' \
    'ROV 0.2.0rc2 明日联调向导' \
    '  1) 查看环境/端口/激活状态' \
    '  2) 恢复并验证 QGC + BlueOS 默认端口' \
    '  3) 测试并启用机械爪候选档案' \
    '  4) 验证并替换新权重' \
    '  5) 应用平衡型超时（不连接飞控）' \
    '  6) 运行自动搜索—对准—接近（不动爪子）' \
    '  7) 运行搜索—对准—人工抓取标定' \
    '  0) 退出'
  read -r -p "请选择 [0-7]: " choice
  case "${choice}" in
    1) dispatch status ;;
    2) dispatch qgc ;;
    3) dispatch gripper ;;
    4) dispatch weights ;;
    5) dispatch timing ;;
    6) dispatch search ;;
    7) dispatch calibrate ;;
    0) exit 0 ;;
    *) printf '请输入 0到7。\n' ;;
  esac
done
