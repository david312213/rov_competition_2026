#!/usr/bin/env bash
set -euo pipefail

# 无机械爪群体搜索—接近入口；共用默认端口、GPU 感知、录像和清理逻辑。
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
printf "%s\n" "模式：无机械爪群体搜索—接近；不会发送机械爪或盲抓下降命令。"
exec "${SCRIPT_DIR}/start_search_approach_test.sh" \
  --workflow cluster_approach "$@"
