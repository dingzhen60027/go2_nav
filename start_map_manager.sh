#!/bin/bash
# Local map workspace: versioning, previews, activation and rollback.

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PORT="${MAP_MANAGER_PORT:-8765}"

source /opt/ros/humble/setup.bash
source "$SCRIPT_DIR/install/setup.bash"

export GO2_NAV_ROOT="$SCRIPT_DIR"

echo "=========================================="
echo "  Go2 Map Workspace"
echo "  http://127.0.0.1:${PORT}"
echo "=========================================="

exec python3 -m uvicorn tools.map_manager.backend.app:app \
  --app-dir "$SCRIPT_DIR" \
  --host 127.0.0.1 \
  --port "$PORT"
