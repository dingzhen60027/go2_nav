#!/bin/bash
# Independent navigation stack for the dual-EKF fused-localization pipeline.
# The legacy start_navigation.sh remains the pure-ICP navigation entry point.

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
export GO2_NAV_ROOT="$SCRIPT_DIR"
source /opt/ros/humble/setup.bash
source "$SCRIPT_DIR/install/setup.bash"
set -u

# Bind every process in this workflow to the physical Go2 DDS interface.
# start_fused_localization.sh already does this for its own descendants, but
# the command adapter, Unitree bridge and Nav2 are siblings launched here and
# otherwise inherit no CYCLONEDDS_URI from the Web service.
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
GO2_DDS_IFACE="${GO2_IFACE:-enx6c1ff7bc241e}"
if ! ip link show "$GO2_DDS_IFACE" >/dev/null 2>&1; then
    echo "ERROR: Go2 DDS network interface does not exist: $GO2_DDS_IFACE"
    echo "Set GO2_IFACE to the interface connected to the robot."
    exit 1
fi
if [ "$(cat "/sys/class/net/${GO2_DDS_IFACE}/carrier" 2>/dev/null || echo 0)" != "1" ]; then
    echo "ERROR: Go2 DDS network interface has no carrier: $GO2_DDS_IFACE"
    echo "请连接并打开机器狗网线，确认该接口变为 UP 后再启动融合导航。"
    exit 1
fi
export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces>
    <NetworkInterface name=\"${GO2_DDS_IFACE}\" priority=\"default\" multicast=\"default\" />
</Interfaces></General></Domain></CycloneDDS>"

CHILD_PIDS=()
CLEANED=0

remember_child() {
    CHILD_PIDS+=("$1")
}

cleanup() {
    local pid deadline alive
    [ "$CLEANED" -eq 1 ] && return
    CLEANED=1
    trap - SIGINT SIGTERM EXIT
    echo ""
    echo "正在停止融合定位导航..."

    for pid in "${CHILD_PIDS[@]}"; do
        kill -INT -- "-$pid" 2>/dev/null || true
    done

    deadline=$((SECONDS + 8))
    while [ "$SECONDS" -lt "$deadline" ]; do
        alive=0
        for pid in "${CHILD_PIDS[@]}"; do
            if kill -0 "$pid" 2>/dev/null; then
                alive=1
                break
            fi
        done
        [ "$alive" -eq 0 ] && break
        sleep 0.2
    done

    for pid in "${CHILD_PIDS[@]}"; do
        kill -TERM -- "-$pid" 2>/dev/null || true
    done
    sleep 0.5
    for pid in "${CHILD_PIDS[@]}"; do
        kill -KILL -- "-$pid" 2>/dev/null || true
        wait "$pid" 2>/dev/null || true
    done
    echo "融合定位导航已停止。"
}

on_signal() {
    cleanup
    exit 0
}

trap on_signal SIGINT SIGTERM
trap cleanup EXIT

ACTIVE_MAP_DIR="${GO2_MAP_DIR:-$SCRIPT_DIR/maps/active}"
NAV_MAP_YAML="$ACTIVE_MAP_DIR/map.yaml"
ICP_MAP_PCD="$ACTIVE_MAP_DIR/localization.pcd"
FUSED_NAV_PARAMS="$SCRIPT_DIR/nav2_config/nav2_fused_params.yaml"
FUSED_RVIZ_CONFIG="$SCRIPT_DIR/nav2_config/nav2_fused.rviz"

if [ ! -f "$ACTIVE_MAP_DIR/manifest.yaml" ] || \
   [ ! -f "$NAV_MAP_YAML" ] || \
   [ ! -f "$ICP_MAP_PCD" ]; then
    echo "ERROR: 没有完整的已激活地图包。"
    echo "先启动地图管理器，在 Web 中选择并激活同时包含 2D/3D 数据的地图。"
    exit 1
fi
for required_file in "$FUSED_NAV_PARAMS" "$FUSED_RVIZ_CONFIG"; do
    if [ ! -f "$required_file" ]; then
        echo "ERROR: 融合导航配置不存在: $required_file"
        exit 1
    fi
done
for required_package in go2_bridge go2_localization nav2_bringup robot_localization; do
    if ! ros2 pkg prefix "$required_package" >/dev/null 2>&1; then
        echo "ERROR: ROS 2 软件包不可用: $required_package"
        exit 1
    fi
done

# A legacy pure-ICP stack owns conflicting TF edges. Never silently kill it or
# create two authorities for map->odom / base_link->livox_frame.
if pgrep -af '[f]ast_icp_loc_node' >/dev/null 2>&1 || \
   pgrep -af '[s]tatic_transform_publisher.*(map[[:space:]]+odom|base_link[[:space:]]+livox_frame)' >/dev/null 2>&1; then
    echo "ERROR: 检测到纯 ICP 定位或其静态 TF 仍在运行。"
    echo "请先在 Web 中停止当前流程，必要时使用‘清理所有进程’，再启动融合导航。"
    exit 1
fi
if pgrep -af '[f]used_icp_matcher|[e]kf_node.*__node:=(ekf_local|ekf_global)' >/dev/null 2>&1; then
    echo "ERROR: 检测到另一套融合定位仍在运行，不能重复启动 TF 发布者。"
    echo "请先停止旧流程或使用 Web 的‘清理所有进程’。"
    exit 1
fi

NAV_MAP_YAML="$(readlink -f "$NAV_MAP_YAML")"
ICP_MAP_PCD="$(readlink -f "$ICP_MAP_PCD")"
ACTIVE_MAP_ID="$(basename "$(readlink -f "$ACTIVE_MAP_DIR")")"

wait_for_topic_publisher() {
    local topic="$1"
    local label="$2"
    local owner_pid="$3"
    local timeout_seconds="$4"
    local deadline=$((SECONDS + timeout_seconds))
    local info
    while [ "$SECONDS" -lt "$deadline" ]; do
        if ! kill -0 "$owner_pid" 2>/dev/null; then
            wait "$owner_pid" 2>/dev/null || true
            echo "ERROR: $label 启动期间融合定位进程已退出。"
            return 1
        fi
        info="$(timeout 2 ros2 topic info "$topic" 2>/dev/null || true)"
        if printf '%s\n' "$info" | rg -q 'Publisher count: [1-9][0-9]*'; then
            return 0
        fi
        sleep 0.5
    done
    echo "ERROR: 等待 $label 超时（话题 $topic 没有发布者）。"
    return 1
}

wait_for_navigation_action() {
    local owner_pid="$1"
    local deadline=$((SECONDS + 45))
    local info
    while [ "$SECONDS" -lt "$deadline" ]; do
        if ! kill -0 "$owner_pid" 2>/dev/null; then
            wait "$owner_pid" 2>/dev/null || true
            echo "ERROR: Nav2 激活期间进程已退出。"
            return 1
        fi
        info="$(timeout 2 ros2 action info /navigate_to_pose 2>/dev/null || true)"
        if printf '%s\n' "$info" | rg -q 'Action servers: [1-9][0-9]*'; then
            return 0
        fi
        sleep 0.5
    done
    echo "ERROR: Nav2 在 45 秒内没有激活 /navigate_to_pose。"
    return 1
}

wait_for_go2_state() {
    if timeout 8 ros2 topic echo \
        --qos-reliability best_effort --once \
        /sportmodestate unitree_go/msg/SportModeState \
        >/dev/null 2>&1; then
        return 0
    fi
    echo "ERROR: Go2 SportModeState data is not arriving on $GO2_DDS_IFACE."
    echo "Check the robot cable, GO2_IFACE and CycloneDDS configuration."
    return 1
}

wait_for_sport_api_publisher() {
    local owner_pid="$1"
    local deadline=$((SECONDS + 10))
    local info
    while [ "$SECONDS" -lt "$deadline" ]; do
        if ! kill -0 "$owner_pid" 2>/dev/null; then
            wait "$owner_pid" 2>/dev/null || true
            echo "ERROR: Go2 command bridge exited during DDS validation."
            return 1
        fi
        info="$(timeout 2 ros2 topic info /api/sport/request 2>/dev/null || true)"
        if printf '%s\n' "$info" | rg -q 'Publisher count: [1-9][0-9]*'; then
            return 0
        fi
        sleep 0.5
    done
    echo "ERROR: Go2 command bridge did not publish /api/sport/request."
    return 1
}

echo "========================================"
echo "  Go2 融合定位 + Nav2 导航"
echo "  地图版本: $ACTIVE_MAP_ID"
echo "  2D 地图:  $NAV_MAP_YAML"
echo "  ICP 地图: $ICP_MAP_PCD"
echo "  TF: map --EKF(global)--> odom --EKF(local)--> base_footprint -> base_link"
echo "  Nav2 里程计: /localization/odometry/local"
echo "  Go2 DDS: $GO2_DDS_IFACE"
echo "========================================"

echo "[0/5] 验证机器狗 DDS 数据链路..."
wait_for_go2_state

echo "[1/5] 启动独立融合定位..."
setsid env GO2_USE_RVIZ=false "$SCRIPT_DIR/start_fused_localization.sh" &
FUSION_PID=$!
remember_child "$FUSION_PID"

wait_for_topic_publisher /localization/odometry/local "本地 EKF 里程计" "$FUSION_PID" 30
wait_for_topic_publisher /scan_obstacles "去地面局部障碍物点云" "$FUSION_PID" 30

echo "[2/5] 启动 Go2 速度适配器..."
setsid ros2 run go2_bridge go2_cmd_adapter &
ADAPTER_PID=$!
remember_child "$ADAPTER_PID"

echo "[3/5] 启动 Go2 命令桥..."
setsid ros2 run go2_bridge go2_bridge &
BRIDGE_PID=$!
remember_child "$BRIDGE_PID"
wait_for_sport_api_publisher "$BRIDGE_PID"

echo "[4/5] 启动融合定位专用 Nav2..."
setsid ros2 launch nav2_bringup bringup_launch.py \
    params_file:="$FUSED_NAV_PARAMS" \
    use_sim_time:=false \
    autostart:=true \
    map:="$NAV_MAP_YAML" &
NAV2_PID=$!
remember_child "$NAV2_PID"

wait_for_navigation_action "$NAV2_PID"

echo "[5/5] 启动融合导航 RViz2..."
export LIBGL_ALWAYS_SOFTWARE=1
setsid rviz2 -d "$FUSED_RVIZ_CONFIG" &
RVIZ_PID=$!
remember_child "$RVIZ_PID"

echo ""
echo "融合导航已就绪。"
echo "  - 在 RViz 用 ‘2D Pose Estimate’ 给融合定位初始位姿"
echo "  - 用 ‘Nav2 Goal’ 或 Web 多目标点工具下发导航任务"
echo "  - Web 停止、清理按钮或 Ctrl+C 均会回收本次启动的子进程"
echo ""

# RViz may be closed independently; localization, command output, and Nav2 are
# critical. If any critical owner exits, fail the workflow and clean the rest.
while true; do
    for entry in \
        "$FUSION_PID:融合定位" \
        "$ADAPTER_PID:速度适配器" \
        "$BRIDGE_PID:命令桥" \
        "$NAV2_PID:Nav2"; do
        pid="${entry%%:*}"
        label="${entry#*:}"
        if ! kill -0 "$pid" 2>/dev/null; then
            wait "$pid" 2>/dev/null || exit_code=$?
            echo "ERROR: $label 异常退出（退出码 ${exit_code:-0}），正在停止整套融合导航。"
            exit 1
        fi
    done
    sleep 1
done
