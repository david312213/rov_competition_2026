#!/usr/bin/env bash
set -euo pipefail

# 独立桌面抽帧工具：不加载 ROS 环境，不连接飞控，不读取权重。
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
PYTHON_BIN="${PROJECT_DIR}/.venv/bin/python"
SOURCE_PACKAGE="${PROJECT_DIR}/ros2_ws/src/rov_competition"

if [[ ! -x "${PYTHON_BIN}" ]]; then
  echo "缺少工程虚拟环境：${PYTHON_BIN}" >&2
  echo "请先运行 ./scripts/install.sh。" >&2
  exit 1
fi

export PYTHONNOUSERSITE=1
export ROV_PROJECT_DIR="${PROJECT_DIR}"
export PYTHONPATH="${SOURCE_PACKAGE}${PYTHONPATH:+:${PYTHONPATH}}"

if ! "${PYTHON_BIN}" - <<'PY'
import cv2
import tkinter

print(f"抽帧环境正常：OpenCV {cv2.__version__}")
PY
then
  echo "抽帧界面依赖不完整，请重新运行 ./scripts/install.sh。" >&2
  exit 1
fi

exec "${PYTHON_BIN}" -m rov_competition.frame_extractor_gui \
  --project-dir "${PROJECT_DIR}" "$@"
