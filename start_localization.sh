#!/bin/bash
# start_localization.sh — 一键启动 MID360 驱动 + Fast ICP 定位 + RViz

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

ACTIVE_MAP_DIR="${GO2_MAP_DIR:-$SCRIPT_DIR/maps/active}"
if [ ! -f "$ACTIVE_MAP_DIR/manifest.yaml" ] || [ ! -f "$ACTIVE_MAP_DIR/localization.pcd" ]; then
    echo "ERROR: 没有完整的已激活地图包。"
    echo "先运行 ./start_map_manager.sh，在网页中选择并激活地图。"
    exit 1
fi
ICP_MAP_PCD="$(readlink -f "$ACTIVE_MAP_DIR/localization.pcd")"
ACTIVE_MAP_ID="$(basename "$(readlink -f "$ACTIVE_MAP_DIR")")"

for p in livox_ros_driver2_node fast_icp_loc_node rviz2; do
    pid=$(pgrep -f "$p" 2>/dev/null || true)
    [ -n "$pid" ] && kill -9 $pid 2>/dev/null
done
sleep 1

echo "=============================="
echo "  Fast ICP Localization"
echo "  地图版本: $ACTIVE_MAP_ID"
echo "  ICP: $ICP_MAP_PCD"
echo "=============================="

echo "[1/3] Livox MID360 驱动..."
ros2 launch livox_ros_driver2 msg_MID360_launch.py &
sleep 3

echo "[2/4] Static TF (base_link→livox_frame)..."
ros2 run tf2_ros static_transform_publisher 0 0 0.3 0 0 0 base_link livox_frame &
sleep 1

echo "[3/4] Fast ICP 定位..."
ros2 launch fast_icp_loc fast_icp_loc.launch.py map_pcd:="$ICP_MAP_PCD" &
sleep 2

echo "[4/4] RViz2..."
rviz2 -d "$SCRIPT_DIR/src/fast_icp_loc/rviz/fast_icp_loc.rviz" &

echo ""
echo "启动完成! 在 RViz 用 '2D Pose Estimate' 给初始位姿"
echo ""

wait
