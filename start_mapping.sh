#!/bin/bash
# start_mapping.sh — 启动 MID360 + FASTer-LIO 建图

set -e

CLEANED=0
MAPPING_PID=""
cleanup() {
    [ "$CLEANED" -eq 1 ] && return
    CLEANED=1
    trap - SIGINT SIGTERM EXIT
    echo "Stopping FASTer-LIO and saving PCD..."
    if [ -n "$MAPPING_PID" ] && kill -0 "$MAPPING_PID" 2>/dev/null; then
        kill -INT "$MAPPING_PID" 2>/dev/null || true
        for _ in $(seq 1 600); do
            kill -0 "$MAPPING_PID" 2>/dev/null || break
            sleep 0.1
        done
    fi
    jobs -pr | xargs -r kill -INT 2>/dev/null || true
    for _ in $(seq 1 150); do
        [ -z "$(jobs -pr)" ] && break
        sleep 0.1
    done
    jobs -pr | xargs -r kill -TERM 2>/dev/null || true
    sleep 1
    jobs -pr | xargs -r kill -KILL 2>/dev/null || true
}
shutdown() {
    cleanup
    exit 0
}
trap shutdown SIGINT SIGTERM
trap cleanup EXIT

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
export GO2_NAV_ROOT="$SCRIPT_DIR"
source "$SCRIPT_DIR/install/setup.bash"

PCD_DIR="${GO2_MAPPING_OUTPUT_DIR:-$SCRIPT_DIR/src/faster-lio/PCD}"
mkdir -p "$PCD_DIR"

echo "=========================================="
echo "  FASTer-LIO MID360 Mapping"
echo "  $(date '+%Y-%m-%d %H:%M:%S')"
echo "  Output: $PCD_DIR"
echo "=========================================="
echo ""

# 1. 清理旧进程
echo "[1/4] 清理旧进程..."
for p in run_mapping_online livox_ros_driver2_node rviz2 pcd2pgm map_saver; do
    pid=$(pgrep -f "$p" 2>/dev/null || true)
    [ -n "$pid" ] && echo "  -> kill $p (PID: $pid)" && kill -INT $pid 2>/dev/null && sleep 0.3 && \
        (kill -0 $pid 2>/dev/null && kill -9 $pid 2>/dev/null || true)
done
sleep 1
echo "  完成"
echo ""

# 2. 网络检查
echo "[2/4] 检查 MID360 网络..."
if ping -c 1 -W 1 192.168.123.20 &>/dev/null; then
    echo "  MID360 在线 ✓ (192.168.123.20)"
else
    echo "  WARNING: 无法 ping 通 192.168.123.20"
fi
echo ""

# 3. 启动建图
echo "[3/4] 启动 FASTer-LIO..."
echo "  启动后让机器狗静止约 3-5 秒"
echo "  等终端出现 'IMU Initial Done' 再开始走"
echo ""

ros2 launch faster_lio mapping_mid360.launch.py &
LAUNCH_PID=$!
for _ in $(seq 1 100); do
    MAPPING_PID="$(pgrep -n -f '/faster_lio/run_mapping_online' 2>/dev/null || true)"
    [ -n "$MAPPING_PID" ] && break
    kill -0 "$LAUNCH_PID" 2>/dev/null || break
    sleep 0.1
done
if [ -z "$MAPPING_PID" ]; then
    echo "ERROR: FASTer-LIO mapping node did not start."
    exit 1
fi
wait "$LAUNCH_PID"
