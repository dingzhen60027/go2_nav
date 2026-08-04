#!/bin/bash
# Isolated MID360 mapping backend using liangheming/FASTLIO2_ROS2.

set -e

CLEANED=0
cleanup() {
    [ "$CLEANED" -eq 1 ] && return
    CLEANED=1
    trap - SIGINT SIGTERM EXIT
    echo ""
    echo "Stopping FAST-LIO2 and saving PCD..."
    jobs -pr | xargs -r kill -INT 2>/dev/null || true
    for _ in $(seq 1 80); do
        [ -z "$(jobs -pr)" ] && break
        sleep 0.1
    done
    jobs -pr | xargs -r kill -TERM 2>/dev/null || true
    sleep 1
    jobs -pr | xargs -r kill -KILL 2>/dev/null || true
    echo "FAST-LIO2 stopped and cleaned."
}
shutdown() {
    cleanup
    exit 0
}
trap shutdown SIGINT SIGTERM
trap cleanup EXIT

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
source /opt/ros/humble/setup.bash
source "$SCRIPT_DIR/install/setup.bash"

if ! ros2 pkg prefix fastlio2 >/dev/null 2>&1 || ! ros2 pkg prefix go2_mapping >/dev/null 2>&1; then
    echo "ERROR: FAST-LIO2 mapping modules are not built."
    echo "Run: colcon build --symlink-install --packages-up-to go2_mapping"
    exit 1
fi

PCD_DIR="${GO2_MAPPING_OUTPUT_DIR:-$SCRIPT_DIR/src/faster-lio/PCD}"
mkdir -p "$PCD_DIR"

echo "=========================================="
echo "  FAST-LIO2 MID360 Mapping"
echo "  Output: $PCD_DIR"
echo "=========================================="
echo "Keep the robot still until IMU initialization completes."

echo "[1/2] Livox MID360..."
ros2 launch livox_ros_driver2 msg_MID360_launch.py &
LIVOX_PID=$!
sleep 3
if ! kill -0 "$LIVOX_PID" 2>/dev/null; then
    echo "ERROR: Livox MID360 driver exited during startup."
    exit 1
fi

echo "[2/2] FAST-LIO2 + map capture..."
ros2 launch go2_mapping fastlio2_mid360_mapping.launch.py output_dir:="$PCD_DIR" &
MAPPING_PID=$!

set +e
wait -n "$LIVOX_PID" "$MAPPING_PID"
STATUS=$?
set -e
echo "ERROR: A FAST-LIO2 mapping child exited unexpectedly (status=$STATUS)."
cleanup
[ "$STATUS" -eq 0 ] && STATUS=1
exit "$STATUS"
