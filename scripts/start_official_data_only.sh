#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

# 只加载官方数据链所需环境；不启动盲抓、视频、YOLO或飞控控制节点。
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

cd "${PROJECT_DIR}"
if command -v ros2 >/dev/null 2>&1; then
  if ! ros2 pkg executables ros2_topic_forwarding 2>/dev/null | grep -q ' topic_forwarding$'; then
    echo "警告：官方 ros2_topic_forwarding 尚未构建；程序会持续重试。" >&2
    echo "首次使用请先运行 ./scripts/install.sh" >&2
  fi
else
  echo "警告：当前终端没有 ros2；请先完成安装，否则无法上传主办方数据。" >&2
fi

exec "${OFFICIAL_DATA_PYTHON:-python3}" -m rov_competition.official_data_only_runtime \
  --config "${CONFIG}" "$@"
