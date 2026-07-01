#!/bin/bash

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "========== PCD → PGM 转换 =========="
echo ""

# 1. source 环境
echo "[1/3] 加载 ROS2 环境..."
source "$SCRIPT_DIR/install/setup.bash"
echo ""

# 2. 启动 pcd2pgm
echo "[2/3] 启动 pcd2pgm (加载 PCD → 发布 /map)..."
PCD_FILE="${1:-$SCRIPT_DIR/src/faster-lio/PCD/scans.pcd}"
echo "  PCD 文件: $PCD_FILE"

ros2 run pcd2pgm pcd2pgm_node \
  --ros-args \
  -p pcd_file:="$PCD_FILE" \
  -p map_resolution:=0.05 \
  -p thre_z_min:=-0.3 \
  -p thre_z_max:=0.5 \
  -p thre_radius:=0.1 \
  -p thres_point_count:=10 \
  -p flag_pass_through:=false \
  -p map_topic_name:=map &
PCD2PGM_PID=$!

# 等 pcd2pgm 加载完成
sleep 2

# 3. 保存 PGM
echo ""
echo "[3/3] 保存地图到 maps/..."
mkdir -p "$SCRIPT_DIR/maps"
ros2 run nav2_map_server map_saver_cli -f "$SCRIPT_DIR/maps/map" &
MAP_SAVER_PID=$!
wait $MAP_SAVER_PID 2>/dev/null || true

# 清理
kill $PCD2PGM_PID 2>/dev/null || true
wait $PCD2PGM_PID 2>/dev/null || true

echo ""
echo "========== 完成 =========="
echo "地图文件:"
ls -lh "$SCRIPT_DIR/maps/"*
