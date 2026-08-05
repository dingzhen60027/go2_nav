#!/bin/bash
# Isolated MID360 mapping backend using liangheming/FASTLIO2_ROS2.

set -eo pipefail

CLEANED=0
LIVOX_PID=""
MAPPING_PID=""
PCD_DIR=""
# map_capture creates the relative service name "save" in /fastlio2.
SAVE_SERVICE="/fastlio2/save"

save_fastlio2_capture() {
    local deadline service_type response resolved_path size_bytes

    if [ -z "$MAPPING_PID" ] || ! kill -0 "$MAPPING_PID" 2>/dev/null; then
        echo "ERROR: FAST-LIO2 mapping process is not running; cannot save PCD."
        return 1
    fi

    echo "Requesting FAST-LIO2 map capture save..."
    deadline=$((SECONDS + 10))
    service_type=""
    while [ "$SECONDS" -lt "$deadline" ]; do
        service_type="$(timeout 2 ros2 service type "$SAVE_SERVICE" 2>/dev/null || true)"
        [ "$service_type" = "std_srvs/srv/Trigger" ] && break
        sleep 0.2
    done
    if [ "$service_type" != "std_srvs/srv/Trigger" ]; then
        echo "ERROR: FAST-LIO2 save service is unavailable: $SAVE_SERVICE"
        return 1
    fi

    if response="$(timeout 45 ros2 service call \
        "$SAVE_SERVICE" std_srvs/srv/Trigger '{}' 2>&1)"; then
        printf '%s\n' "$response"
    else
        printf '%s\n' "$response"
        echo "ERROR: FAST-LIO2 save service call failed."
        return 1
    fi

    # The service only returns after the binary PCD and the scans.pcd symlink
    # have both been written. Keep a short filesystem check here so Web never
    # reports a successful stop when the expected artifact is missing.
    deadline=$((SECONDS + 5))
    while [ "$SECONDS" -lt "$deadline" ] && [ ! -s "$PCD_DIR/scans.pcd" ]; do
        sleep 0.1
    done
    if [ ! -s "$PCD_DIR/scans.pcd" ]; then
        echo "ERROR: Save service returned without creating $PCD_DIR/scans.pcd"
        return 1
    fi

    resolved_path="$(readlink -f "$PCD_DIR/scans.pcd")"
    size_bytes="$(stat -Lc '%s' "$PCD_DIR/scans.pcd")"
    echo "FAST-LIO2 PCD saved: $resolved_path ($size_bytes bytes)"
}

cleanup() {
    local save_status=0

    [ "$CLEANED" -eq 1 ] && return
    CLEANED=1
    trap - SIGINT SIGTERM EXIT
    echo ""
    echo "Stopping FAST-LIO2 and saving PCD..."

    # A background ros2 launch process can ignore the shell's SIGINT. Request
    # the capture explicitly while its ROS node and accumulated points are
    # still alive; destructor-only saving loses the map when launch escalates.
    if [ -n "$MAPPING_PID" ] && kill -0 "$MAPPING_PID" 2>/dev/null; then
        if save_fastlio2_capture; then
            save_status=0
        else
            save_status=$?
        fi
    fi

    jobs -pr | xargs -r kill -INT 2>/dev/null || true
    for _ in $(seq 1 80); do
        [ -z "$(jobs -pr)" ] && break
        sleep 0.1
    done
    jobs -pr | xargs -r kill -TERM 2>/dev/null || true
    sleep 1
    jobs -pr | xargs -r kill -KILL 2>/dev/null || true
    if [ "$save_status" -ne 0 ]; then
        echo "ERROR: FAST-LIO2 stopped, but its PCD was not saved."
        return "$save_status"
    fi
    echo "FAST-LIO2 stopped and PCD save was verified."
}
shutdown() {
    local status=0
    if cleanup; then
        status=0
    else
        status=$?
    fi
    exit "$status"
}
trap shutdown SIGINT SIGTERM
trap cleanup EXIT

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
export GO2_NAV_ROOT="$SCRIPT_DIR"
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
if cleanup; then
    CLEANUP_STATUS=0
else
    CLEANUP_STATUS=$?
fi
[ "$STATUS" -eq 0 ] && STATUS=1
[ "$CLEANUP_STATUS" -ne 0 ] && STATUS="$CLEANUP_STATUS"
exit "$STATUS"
