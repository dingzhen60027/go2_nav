#!/bin/bash

set -e

echo "========== FASTer-LIO MID360 建图启动脚本 =========="
echo ""

# ---------- 1. 清理旧进程 ----------
echo "[1/4] 清理旧进程..."
for proc in "run_mapping_online" "livox_ros_driver2_node" "rviz2"; do
    pid=$(pgrep -f "$proc" 2>/dev/null || true)
    if [ -n "$pid" ]; then
        echo "  -> 终止 $proc (PID: $pid)"
        kill -INT "$pid" 2>/dev/null || true
        sleep 0.5
        # 如果没退出，强制杀
        if kill -0 "$pid" 2>/dev/null; then
            kill -9 "$pid" 2>/dev/null || true
        fi
    fi
done
sleep 1
echo "  完成"
echo ""

# ---------- 2. Source ROS2 环境 ----------
echo "[2/4] 加载 ROS2 工作空间..."
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
if [ -f "$SCRIPT_DIR/install/setup.bash" ]; then
    source "$SCRIPT_DIR/install/setup.bash"
    echo "   workspace: $SCRIPT_DIR"
else
    echo "  ERROR: 找不到 $SCRIPT_DIR/install/setup.bash"
    exit 1
fi
echo "  完成"
echo ""

# ---------- 3. 网络检查 ----------
echo "[3/4] 检查 MID360 网络连接..."
if ping -c 1 -W 1 192.168.123.20 &>/dev/null; then
    echo "  MID360 在线 (192.168.123.20)"
else
    echo "  WARNING: 无法 ping 通 192.168.123.20"
    echo "  请确认 MID360 已连接且 IP 配置正确"
fi
echo ""

# ---------- 4. 启动建图 ----------
echo "[4/4] 启动 FASTer-LIO MID360 建图..."
echo ""
echo "  >> 启动后请让机器狗静止约 1 秒，等待 IMU 初始化完成 <<"
echo ""

ros2 launch faster_lio mapping_mid360.launch.py
