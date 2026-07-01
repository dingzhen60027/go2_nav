#!/bin/bash
# pcd2pgm.sh — 用 pcd2pgm 包将 PCD 转为 2D 导航地图, 自动命名不覆盖
# 用法: ./pcd2pgm.sh [PCD文件]

set -e

PCD_DIR="/home/wjg/go2_nav/src/faster-lio/PCD"
OUT_DIR="/home/wjg/go2_nav/maps"
PCD_FILE="${1:-${PCD_DIR}/scans.pcd}"

if [ ! -f "$PCD_FILE" ]; then
    echo "错误: 未找到 $PCD_FILE"
    exit 1
fi

mkdir -p "$OUT_DIR"
TIMESTAMP=$(date +%m%d_%H%M)

# 杀旧进程
pkill -9 -f pcd2pgm 2>/dev/null || true
sleep 1

source /home/wjg/go2_nav/install/setup.bash

echo "===== PCD → PGM (pcd2pgm) ====="
echo "PCD: $PCD_FILE ($(du -h "$PCD_FILE" | cut -f1))"
echo ""

# 启动 pcd2pgm
ros2 run pcd2pgm pcd2pgm_node --ros-args \
  -p pcd_file:="$PCD_FILE" \
  -p map_resolution:=0.05 \
  -p thre_z_min:=0.2 \
  -p thre_z_max:=4.0 \
  -p thre_radius:=0.1 \
  -p thres_point_count:=2 \
  -p flag_pass_through:=false \
  -p map_topic_name:=map &
PID=$!

sleep 5

# 保存地图
ros2 run nav2_map_server map_saver_cli \
  -f "${OUT_DIR}/map_${TIMESTAMP}" \
  -t map --fmt pgm 2>&1 || true

kill $PID 2>/dev/null; wait $PID 2>/dev/null
echo ""
echo "完成: ${OUT_DIR}/map_${TIMESTAMP}.pgm"
