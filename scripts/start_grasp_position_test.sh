#!/usr/bin/env bash
set -euo pipefail

# 自动下潜、扫描和偏航对准；稳定后停车并进入 WASD 人工抓取位置标定。
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec "${SCRIPT_DIR}/start_search_approach_test.sh" \
  --workflow manual_grasp_calibration \
  "$@"
