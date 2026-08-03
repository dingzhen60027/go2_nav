#!/bin/bash
# start_mapping.sh — 启动 MID360 + FASTer-LIO 建图

set -e

CLEANED=0
cleanup() {
    [ "$CLEANED" -eq 1 ] && return
    CLEANED=1
    trap - SIGINT SIGTERM EXIT
    jobs -pr | xargs -r kill -INT 2>/dev/null || true
    sleep 1
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
source "$SCRIPT_DIR/install/setup.bash"

echo "=========================================="
echo "  FASTer-LIO MID360 Mapping"
echo "  $(date '+%Y-%m-%d %H:%M:%S')"
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
wait $!
