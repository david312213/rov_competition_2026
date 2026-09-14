#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
PYTHON_BIN="${PROJECT_DIR}/.venv/bin/python"

if [[ ! -x "${PYTHON_BIN}" ]]; then
  PYTHON_BIN=python3
fi

cd "${PROJECT_DIR}"
echo "[1/4] 运行纯软件测试……"
"${PYTHON_BIN}" -m pytest -q

echo "[2/4] 检查 Python 语法……"
find ros2_ws/src scripts -name '*.py' -print0 \
  | xargs -0 "${PYTHON_BIN}" -m py_compile

echo "[3/4] 检查 Shell 语法……"
bash -n scripts/*.sh

echo "[4/4] 检查 ROS package.xml……"
"${PYTHON_BIN}" - <<'PY'
from pathlib import Path
from xml.etree import ElementTree

for path in Path("ros2_ws/src").glob("*/package.xml"):
    ElementTree.parse(path)
    print(f"  OK: {path}")
PY

echo "离线检查全部通过；本命令不会连接或驱动实艇。"
