#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

# 只加载环境，不调用原搜索脚本、预检、许可或自动解锁流程。
# 缺少ROS环境时，Python主控制仍可运行，检测缺失会按30秒规则降级。
for setup in /opt/ros/humble/setup.bash "${PROJECT_DIR}/.venv/bin/activate" "${PROJECT_DIR}/ros2_ws/install/setup.bash"; do
  if [[ -f "${setup}" ]]; then
    set +u
    source "${setup}"
    set -u
  fi
done
export PYTHONPATH="${PROJECT_DIR}/ros2_ws/src/rov_competition${PYTHONPATH:+:${PYTHONPATH}}"
CONFIG="${PROJECT_DIR}/config/blind_grab.local.yaml"
if [[ ! -f "${CONFIG}" ]]; then
  CONFIG="${PROJECT_DIR}/ros2_ws/src/rov_competition/config/blind_grab.yaml"
fi
cd "${PROJECT_DIR}"
exec "${BLIND_GRAB_PYTHON:-python3}" -m rov_competition.blind_grab_runtime --config "${CONFIG}" "$@"
