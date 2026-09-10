#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export COVERAGE_CONFIG="${COVERAGE_CONFIG:-${SCRIPT_DIR}/../config/coverage.local.yaml}"
if [[ ! -r "${COVERAGE_CONFIG}" ]]; then
  printf '请复制并填写 ros2_ws/src/rov_competition/config/coverage.example.yaml 到 config/coverage.local.yaml\n' >&2
  exit 2
fi
exec bash "${SCRIPT_DIR}/start_search_approach_test.sh" --workflow cluster_collection "$@"
