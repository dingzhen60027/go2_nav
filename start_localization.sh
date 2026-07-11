#!/bin/bash
# start_localization.sh — 一键启动 MID360 驱动 + Fast ICP 定位 + RViz

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
source "$SCRIPT_DIR/install/setup.bash"

for p in livox_ros_driver2_node fast_icp_loc_node rviz2; do
    pid=$(pgrep -f "$p" 2>/dev/null || true)
    [ -n "$pid" ] && kill -9 $pid 2>/dev/null
done
sleep 1

echo "=============================="
echo "  Fast ICP Localization"
echo "=============================="

echo "[1/3] Livox MID360 驱动..."
ros2 launch livox_ros_driver2 msg_MID360_launch.py &
sleep 3

echo "[2/4] Static TF (base_link→livox_frame)..."
ros2 run tf2_ros static_transform_publisher 0 0 0.3 0 0 0 base_link livox_frame &
sleep 1

echo "[3/4] Fast ICP 定位..."
ros2 launch fast_icp_loc fast_icp_loc.launch.py &
sleep 2

echo "[4/4] RViz2..."
rviz2 -d "$SCRIPT_DIR/src/fast_icp_loc/rviz/fast_icp_loc.rviz" &

echo ""
echo "启动完成! 在 RViz 用 '2D Pose Estimate' 给初始位姿"
echo ""

trap "kill 0 2>/dev/null; exit 0" SIGINT SIGTERM
wait
