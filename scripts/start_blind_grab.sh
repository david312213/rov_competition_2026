#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

# 只加载环境，不调用原搜索脚本、预检、许可或自动解锁流程。
# 缺少ROS、模型或视频时，定时沉底蛇形盲抓仍按单调时钟继续。
for setup in /opt/ros/humble/setup.bash "${PROJECT_DIR}/.venv/bin/activate" "${PROJECT_DIR}/ros2_ws/install/setup.bash"; do
  if [[ -f "${setup}" ]]; then
    set +u
    source "${setup}"
    set -u
  fi
done
export PYTHONNOUSERSITE=1
export PYTHONPATH="${PROJECT_DIR}/ros2_ws/src/rov_competition${PYTHONPATH:+:${PYTHONPATH}}"
CONFIG="${PROJECT_DIR}/config/blind_grab.local.yaml"
if [[ ! -f "${CONFIG}" ]]; then
  CONFIG="${PROJECT_DIR}/ros2_ws/src/rov_competition/config/blind_grab.yaml"
fi
"${SCRIPT_DIR}/apply_blind_grab_power_10.sh" --config "${CONFIG}"
cd "${PROJECT_DIR}"
if command -v ros2 >/dev/null 2>&1; then
  if ! ros2 pkg executables ros2_topic_forwarding 2>/dev/null | grep -q ' topic_forwarding$'; then
    echo "警告：官方 ros2_topic_forwarding 尚未安装；盲抓会继续，但官方数据只能等待重试。" >&2
    echo "首次使用请先运行 ./scripts/install.sh" >&2
  fi
else
  echo "警告：当前终端没有 ros2；盲抓会继续，官方ROS线程将持续重试。" >&2
fi
exec "${BLIND_GRAB_PYTHON:-python3}" -m rov_competition.blind_grab_runtime --config "${CONFIG}" "$@"
