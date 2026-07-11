#!/bin/bash
# pcd2pgm.sh — PCD → 2D 导航地图
# 输出:
#   maps/map_TIMESTAMP.{pgm,yaml}     ← 2D 导航地图
#   maps/clean/pcd_icp_latest.pcd     ← ICP 定位地图（动态滤波后，未切片）

set -e

PCD_DIR="/home/wjg/go2_nav/src/faster-lio/PCD"
OUT_DIR="/home/wjg/go2_nav/maps"
PCD_FILE="${1:-${PCD_DIR}/scans.pcd}"

if [ ! -f "$PCD_FILE" ]; then
    echo "错误: 未找到 $PCD_FILE"
    exit 1
fi

mkdir -p "$OUT_DIR/clean"
TIMESTAMP=$(date +%m%d_%H%M)

pkill -9 -f "pcd2pgm_node" 2>/dev/null || true
pkill -9 -f "rviz2" 2>/dev/null || true
sleep 1

source /home/wjg/go2_nav/install/setup.bash

# 临时中间文件（用完删除）
TMP_STAT="/tmp/pcd_stat_${TIMESTAMP}.pcd"
TMP_RADIUS="${OUT_DIR}/clean/pcd_radius_${TIMESTAMP}.pcd"
TMP_SLICED="/tmp/pcd_sliced_${TIMESTAMP}.pcd"

# ===== Step 1: 统计离群点滤波 =====
echo "[1/4] 统计离群点滤波（去除稀疏动态点）..."
pcl_outlier_removal "$PCD_FILE" "$TMP_STAT" \
  -method statistical -mean_k 20 -std_dev_mul 0.5 2>&1 | tail -1

# ===== Step 2: 半径离群点滤波 + 保存 ICP 地图 =====
echo "[2/4] 半径离群点滤波（去除残留噪点）..."
pcl_outlier_removal "$TMP_STAT" "$TMP_RADIUS" \
  -method radius -radius 0.3 -min_pts 4 2>&1 | tail -1

# 保存为 ICP 定位参考地图
ICP_PCD="${OUT_DIR}/clean/pcd_icp_latest.pcd"
cp "$TMP_RADIUS" "$ICP_PCD"
echo "  → ICP 定位地图: $ICP_PCD ($(du -h "$ICP_PCD" | cut -f1))"

# ===== Step 3: z 轴切片 =====
echo "[3/4] z 轴切片 (0.4m ~ 1.5m)..."
pcl_passthrough_filter "$TMP_RADIUS" "$TMP_SLICED" \
  -field z -min 0.4 -max 1.5 2>&1 | tail -1
echo "  → 切片后: $(du -h "$TMP_SLICED" | cut -f1)"

# ===== Step 4: pcd2pgm 投影 2D + rviz =====
echo "[4/4] PCD → PGM..."
ros2 run pcd2pgm pcd2pgm_node --ros-args \
  -p pcd_file:="$TMP_SLICED" \
  -p map_resolution:=0.05 \
  -p thre_z_min:=-5.0 \
  -p thre_z_max:=5.0 \
  -p thre_radius:=0.01 \
  -p thres_point_count:=1 \
  -p flag_pass_through:=false \
  -p map_topic_name:=map &
PID=$!

rviz2 -d "/home/wjg/go2_nav/src/pcd2pgm/rviz/pcd2pgm.rviz" 2>/dev/null &

sleep 6

ros2 run nav2_map_server map_saver_cli \
  -f "${OUT_DIR}/map_${TIMESTAMP}" -t map --fmt pgm 2>&1 || true

# 清理中间文件
rm -f "$TMP_STAT" "$TMP_RADIUS" "$TMP_SLICED"

# Nav2 用 latest 软链接
ln -sf "map_${TIMESTAMP}.yaml" "${OUT_DIR}/map_latest.yaml"
ln -sf "map_${TIMESTAMP}.pgm" "${OUT_DIR}/map_latest.pgm"

echo ""
echo "===== 完成 ====="
echo "  Nav2 地图:     ${OUT_DIR}/map_${TIMESTAMP}.pgm"
echo "  Nav2 引用:     ${OUT_DIR}/map_latest.yaml → map_${TIMESTAMP}.yaml"
echo "  ICP 定位地图:  $ICP_PCD"

trap "kill $PID 2>/dev/null; exit 0" SIGINT SIGTERM
wait $PID 2>/dev/null
