#!/bin/bash
# start_navigation.sh — 一键启动: MID360 + ICP 定位 + Nav2 导航 + RViz

CLEANED=0
cleanup() {
    [ "$CLEANED" -eq 1 ] && return
    CLEANED=1
    trap - SIGINT SIGTERM EXIT
    echo ""
    echo "Shutting down..."
    jobs -pr | xargs -r kill -INT 2>/dev/null || true
    sleep 1
    jobs -pr | xargs -r kill -TERM 2>/dev/null || true
    sleep 1
    jobs -pr | xargs -r kill -KILL 2>/dev/null || true
    pkill -f "component_container_isolated" 2>/dev/null || true
    pkill -f "nav2_container" 2>/dev/null || true
    echo "Done."
}
shutdown() {
    cleanup
    exit 0
}
trap shutdown SIGINT SIGTERM
trap cleanup EXIT

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
export GO2_NAV_ROOT="$SCRIPT_DIR"
source /opt/ros/humble/setup.bash
source "$SCRIPT_DIR/install/setup.bash"

ACTIVE_MAP_DIR="${GO2_MAP_DIR:-$SCRIPT_DIR/maps/active}"
if [ ! -f "$ACTIVE_MAP_DIR/manifest.yaml" ] || \
   [ ! -f "$ACTIVE_MAP_DIR/map.yaml" ] || \
   [ ! -f "$ACTIVE_MAP_DIR/localization.pcd" ]; then
    echo "ERROR: 没有完整的已激活地图包。"
    echo "先运行 ./start_map_manager.sh，在网页中选择并激活地图。"
    exit 1
fi
NAV_MAP_YAML="$(readlink -f "$ACTIVE_MAP_DIR/map.yaml")"
ICP_MAP_PCD="$(readlink -f "$ACTIVE_MAP_DIR/localization.pcd")"
ACTIVE_MAP_ID="$(basename "$(readlink -f "$ACTIVE_MAP_DIR")")"

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
IFACE="${GO2_IFACE:-enx6c1ff7bc241e}"
export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces>
    <NetworkInterface name=\"${IFACE}\" priority=\"default\" multicast=\"default\" />
</Interfaces></General></Domain></CycloneDDS>"
echo "DDS bound to: ${IFACE}"

echo "=============================="
echo "  FAST ICP + Nav2 导航"
echo "  地图版本: $ACTIVE_MAP_ID"
echo "  Nav2: $NAV_MAP_YAML"
echo "  ICP:  $ICP_MAP_PCD"
echo "=============================="

# 1. Livox MID360 驱动
echo "[1/6] Livox MID360..."
ros2 launch livox_ros_driver2 msg_MID360_launch.py &
sleep 3

# 2. Static TF
echo "[2/6] Static TF (map→odom, base_link→livox_frame)..."
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom &
ros2 run tf2_ros static_transform_publisher 0 0 0.3 0 0 0 base_link livox_frame &
sleep 1

# 3. Fast ICP 定位
echo "[3/6] Fast ICP 定位..."
ros2 launch fast_icp_loc fast_icp_loc.launch.py map_pcd:="$ICP_MAP_PCD" &
sleep 2

# 4. Go2 velocity adapter + bridge
echo "[4/6] Go2 cmd adapter + bridge..."
ros2 run go2_bridge go2_cmd_adapter &
ros2 run go2_bridge go2_bridge &
sleep 1

# 5. Nav2
echo "[5/6] Nav2 导航栈..."
ros2 launch nav2_bringup bringup_launch.py \
  params_file:="$SCRIPT_DIR/nav2_config/nav2_params.yaml" \
  use_sim_time:=false \
  autostart:=true \
  map:="$NAV_MAP_YAML" &
sleep 2

# 6. RViz
echo "[6/6] RViz2..."
export LIBGL_ALWAYS_SOFTWARE=1
rviz2 -d "$SCRIPT_DIR/nav2_config/nav2.rviz" &

echo ""
echo "启动完成!"
echo "  - 在 RViz 用 '2D Pose Estimate' 给初始位姿"
echo "  - 用 'Nav2 Goal' 下发导航目标点"
echo "  - Ctrl+C 退出并清理所有进程"
echo ""

wait
