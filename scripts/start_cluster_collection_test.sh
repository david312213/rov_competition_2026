#!/usr/bin/env bash
set -euo pipefail

# 独立群体盲抓入口；共用默认端口、GPU 感知、录像和清理逻辑。
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec "${SCRIPT_DIR}/start_search_approach_test.sh" \
  --workflow cluster_collection "$@"
