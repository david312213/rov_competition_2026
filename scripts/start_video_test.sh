#!/usr/bin/env bash
set -euo pipefail

# 一键视频测试入口：
#   1. ROS 启动 5600 -> 5701/5702 原始 RTP 分流；
#   2. ROS 启动 YOLO 感知，但故意不启动飞控网关；
#   3. 尽量自动打开 QGroundControl 和 rqt_image_view。
# 本脚本永远不解锁、不启动任务，也没有向推进器发送命令的通道。

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
ROS_SETUP="/opt/ros/humble/setup.bash"
VENV_SETUP="${PROJECT_DIR}/.venv/bin/activate"
WORKSPACE_SETUP="${PROJECT_DIR}/ros2_ws/install/setup.bash"

START_QGC=true
START_VIEWER=true

usage() {
  cat <<'EOF'
用法：
  ./scripts/start_video_test.sh [--no-qgc] [--no-viewer]

默认行为：启动视频分流、YOLO，并尝试打开 QGC 和带框查看器。
--no-qgc     不自动打开 QGroundControl
--no-viewer  不自动打开 rqt_image_view

如 QGC 不在 PATH，可这样指定：
  QGC_EXECUTABLE=/完整路径/QGroundControl.AppImage ./scripts/start_video_test.sh
EOF
}

while (($# > 0)); do
  case "$1" in
    --no-qgc)
      START_QGC=false
      ;;
    --no-viewer)
      START_VIEWER=false
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
  shift
done

for required_file in \
  "${ROS_SETUP}" \
  "${VENV_SETUP}" \
  "${WORKSPACE_SETUP}" \
  "${PROJECT_DIR}/config/robot.yaml" \
  "${PROJECT_DIR}/ros2_ws/src/rov_competition/config/autonomy.yaml" \
  "${PROJECT_DIR}/ros2_ws/src/rov_competition/config/targets.yaml" \
  "${PROJECT_DIR}/ros2_ws/src/rov_competition/models/seafood_yolo26x.pt"; do
  if [[ ! -r "${required_file}" ]]; then
    echo "缺少必要文件：${required_file}" >&2
    echo "请先完成安装、构建、实艇配置和权重复制。" >&2
    exit 1
  fi
done

# ROS 2 Humble 的环境脚本不保证兼容 nounset，因此只在 source 时临时关闭。
set +u
source "${ROS_SETUP}"
source "${VENV_SETUP}"
source "${WORKSPACE_SETUP}"
set -u
export PYTHONNOUSERSITE=1
cd "${PROJECT_DIR}"

port_owner() {
  local port="$1"
  ss -H -lunp 2>/dev/null | grep -E ":${port}([[:space:]]|$)" || true
}

ensure_free_port() {
  local port="$1"
  local owner
  owner="$(port_owner "${port}")"
  if [[ -n "${owner}" ]]; then
    echo "UDP ${port} 已被占用，先关闭旧视频进程：" >&2
    echo "${owner}" >&2
    exit 1
  fi
}

ensure_free_port 5600
ensure_free_port 5702

QGC_PORT_OWNER="$(port_owner 5701)"
if [[ -n "${QGC_PORT_OWNER}" ]] \
  && ! grep -Eiq 'qgroundcontrol' <<<"${QGC_PORT_OWNER}"; then
  echo "UDP 5701 已被非 QGC 程序占用：" >&2
  echo "${QGC_PORT_OWNER}" >&2
  exit 1
fi

if ros2 node list 2>/dev/null \
  | grep -Eq '^/(rov_autonomy|rov_stream_bridge|rov_vehicle_gateway)$'; then
  echo "检测到旧的视频节点或飞控网关，请先在原终端按 Ctrl+C：" >&2
  ros2 node list 2>/dev/null \
    | grep -E '^/(rov_autonomy|rov_stream_bridge|rov_vehicle_gateway)$' >&2
  exit 1
fi

LAUNCH_PID=""
VIEWER_PID=""

cleanup() {
  local status=$?
  trap - EXIT INT TERM

  if [[ -n "${VIEWER_PID}" ]] && kill -0 "${VIEWER_PID}" 2>/dev/null; then
    kill -INT "${VIEWER_PID}" 2>/dev/null || true
  fi
  if [[ -n "${LAUNCH_PID}" ]] && kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    kill -INT "${LAUNCH_PID}" 2>/dev/null || true
  fi

  [[ -z "${VIEWER_PID}" ]] || wait "${VIEWER_PID}" 2>/dev/null || true
  [[ -z "${LAUNCH_PID}" ]] || wait "${LAUNCH_PID}" 2>/dev/null || true
  echo
  echo "视频分流和 YOLO 已停止；QGC 如仍打开可继续用于人工观察。"
  exit "${status}"
}

trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

echo "============================================================"
echo "ROV 一键视频测试（绝不会连接飞控网关或驱动推进器）"
echo "  输入：树莓派 RTP/H.264 -> UDP 5600"
echo "  QGC：UDP 5701"
echo "  YOLO：UDP 5702"
echo "  停止：回到本终端按 Ctrl+C"
echo "============================================================"

ros2 launch rov_competition video_test.launch.py \
  robot_config:="${PROJECT_DIR}/config/robot.yaml" \
  autonomy_config:="${PROJECT_DIR}/ros2_ws/src/rov_competition/config/autonomy.yaml" \
  targets_config:="${PROJECT_DIR}/ros2_ws/src/rov_competition/config/targets.yaml" \
  source_port:=5600 \
  qgc_port:=5701 \
  ai_port:=5702 &
LAUNCH_PID=$!

sleep 2
if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
  echo "ROS 视频启动失败，请查看上方完整报错。" >&2
  wait "${LAUNCH_PID}"
fi

if "${START_QGC}"; then
  if pgrep -f '[Qq][Gg]round[Cc]ontrol' >/dev/null 2>&1; then
    echo "QGroundControl 已经运行，继续使用现有窗口。"
  else
    QGC_COMMAND="${QGC_EXECUTABLE:-}"
    if [[ -z "${QGC_COMMAND}" ]]; then
      QGC_COMMAND="$(command -v QGroundControl 2>/dev/null || true)"
    fi
    if [[ -z "${QGC_COMMAND}" ]]; then
      QGC_COMMAND="$(command -v qgroundcontrol 2>/dev/null || true)"
    fi

    if [[ -n "${QGC_COMMAND}" && -x "${QGC_COMMAND}" ]]; then
      QGC_LOG="${XDG_RUNTIME_DIR:-/tmp}/rov_qgc_$$.log"
      "${QGC_COMMAND}" >"${QGC_LOG}" 2>&1 &
      echo "已打开 QGroundControl；启动日志：${QGC_LOG}"
    else
      echo "没有自动找到 QGC，请手动打开并选择 UDP H.264 端口 5701。"
    fi
  fi
fi

if "${START_VIEWER}"; then
  if ros2 pkg prefix rqt_image_view >/dev/null 2>&1; then
    (
      sleep 4
      exec ros2 run rqt_image_view rqt_image_view
    ) &
    VIEWER_PID=$!
    echo "带框查看器将在几秒后打开；请选择 /rov/annotated_image/compressed。"
  else
    echo "未安装 rqt_image_view；仍可用 ros2 topic hz 检查识别帧率。"
  fi
fi

set +e
wait "${LAUNCH_PID}"
LAUNCH_STATUS=$?
set -e
exit "${LAUNCH_STATUS}"
