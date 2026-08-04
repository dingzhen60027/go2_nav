#!/bin/bash

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
source /opt/ros/humble/setup.bash
source "$SCRIPT_DIR/install/setup.bash"
set -u

ACTIVE_MAP_DIR="${GO2_MAP_DIR:-$SCRIPT_DIR/maps/active}"
if [ ! -f "$ACTIVE_MAP_DIR/manifest.yaml" ] || \
   [ ! -f "$ACTIVE_MAP_DIR/localization.pcd" ]; then
    echo "ERROR: 没有完整的已激活地图包。"
    echo "先运行 ./start_map_manager.sh，在网页中选择并激活地图。"
    exit 1
fi

if ! ros2 pkg prefix robot_localization >/dev/null 2>&1; then
    echo "ERROR: robot_localization 尚未安装。"
    echo "运行: sudo apt install ros-humble-robot-localization"
    exit 1
fi

ICP_MAP_PCD="$(readlink -f "$ACTIVE_MAP_DIR/localization.pcd")"
ACTIVE_MAP_ID="$(basename "$(readlink -f "$ACTIVE_MAP_DIR")")"
START_LIVOX="${GO2_START_LIVOX:-auto}"
if [ "$START_LIVOX" = "auto" ]; then
    if pgrep -f '[/]livox_ros_driver2_node' >/dev/null || \
       timeout 3 ros2 topic info /livox/lidar 2>/dev/null | \
       rg -q "Publisher count: [1-9]"; then
        START_LIVOX="false"
        LIVOX_MODE="复用已运行的驱动"
    else
        START_LIVOX="true"
        LIVOX_MODE="由本次定位启动并托管"
    fi
elif [ "$START_LIVOX" = "true" ]; then
    LIVOX_MODE="由本次定位强制启动并托管"
elif [ "$START_LIVOX" = "false" ]; then
    LIVOX_MODE="不启动驱动"
else
    echo "ERROR: GO2_START_LIVOX 只能是 auto、true 或 false。"
    exit 1
fi

echo "========================================"
echo "  Go2 Dual-EKF Fused Localization"
echo "  地图版本: $ACTIVE_MAP_ID"
echo "  ICP: $ICP_MAP_PCD"
echo "  Livox: $LIVOX_MODE"
echo "========================================"

exec ros2 launch go2_localization fused_localization.launch.py \
    map_pcd:="$ICP_MAP_PCD" \
    sport_state_topic:="${GO2_SPORT_STATE_TOPIC:-/sportmodestate}" \
    start_livox:="$START_LIVOX" \
    use_rviz:="${GO2_USE_RVIZ:-true}"
