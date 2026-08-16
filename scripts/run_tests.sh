#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
PYTHON_BIN="${PROJECT_DIR}/.venv/bin/python"

if [[ ! -x "${PYTHON_BIN}" ]]; then
  PYTHON_BIN=python3
fi

cd "${PROJECT_DIR}"
"${PYTHON_BIN}" -m pytest -q
find ros2_ws/src/rov_competition -name '*.py' -print0 \
  | xargs -0 "${PYTHON_BIN}" -m py_compile

