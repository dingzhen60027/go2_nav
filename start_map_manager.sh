#!/bin/bash
# Local map workspace: versioning, previews, activation and rollback.

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PORT="${MAP_MANAGER_PORT:-8765}"
PYTHON_BIN="${MAP_MANAGER_PYTHON:-$SCRIPT_DIR/build/env/map_manager_venv/bin/python}"

source /opt/ros/humble/setup.bash
source "$SCRIPT_DIR/install/setup.bash"

if [ ! -x "$PYTHON_BIN" ]; then
  echo "ERROR: Map Manager Python not found: $PYTHON_BIN" >&2
  echo "Run setup_go2.sh first, or set MAP_MANAGER_PYTHON." >&2
  exit 1
fi

export GO2_NAV_ROOT="$SCRIPT_DIR"

echo "=========================================="
echo "  Go2 Map Workspace"
echo "  http://127.0.0.1:${PORT}"
echo "=========================================="

exec "$PYTHON_BIN" -m uvicorn tools.map_manager.backend.app:app \
  --app-dir "$SCRIPT_DIR" \
  --host 127.0.0.1 \
  --port "$PORT"
