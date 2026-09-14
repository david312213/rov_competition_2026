#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
printf '%s\n' '完整单目标斜向抓取与底部抬爪投放：仅允许已标定 dual_dof 档案。'
export ROV_REQUIRE_DUAL_GRASP=1
exec "${SCRIPT_DIR}/start_search_approach_test.sh" --workflow cluster_collection "$@"
