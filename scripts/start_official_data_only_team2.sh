#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
"${SCRIPT_DIR}/set_official_team.sh" 2
exec "${SCRIPT_DIR}/start_official_data_only.sh" "$@"
