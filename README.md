# go2_nav

Unitree Go2 四足机器人自主导航系统，基于 ROS2 Humble + Livox MID360。

## 系统架构

```
MID360 → FASTer-LIO (建图) → PCD → pcd2pgm → 2D PGM 地图
                                   ├→ fast_icp_loc（纯 ICP 独立定位）
                                   └→ go2_localization/fused_icp_matcher
                                      → 运动预测 + ICP + 双 EKF 融合定位
Go2 SDK → go2_bridge → /odom + /cmd_vel
Nav2 → 全局/局部规划器 → go2_bridge → Go2
```

## 硬件

- Unitree Go2 四足机器人
- Livox MID360 激光雷达
- 板载 NVIDIA Jetson (ROS2 主站)

## 软件包

| 包 | 说明 |
|---|---|
| `faster-lio` | FASTer-LIO 紧耦合 LiDAR-IMU 建图 |
| `fast_icp_loc` | 基于 ICP 的实时定位，对接 Nav2 |
| `go2_localization` | 独立融合 ICP、Go2 运动预测与双 EKF 定位 |
| `go2_bridge` | Go2 SDK 桥接，收发 cmd_vel/odom |
| `pcd2pgm` | PCD 点云 → 2D 占据栅格地图 |
| `unitree_api` / `unitree_go` | Unitree Go2 ROS2 消息定义 |
| `livox_ros_driver2` | Livox MID360 ROS2 驱动 |

## 快速开始

### 环境配置

```bash
source setup_go2.sh [网卡名]
# 默认网卡: enx6c1ff7bc241e
```

### 建图

```bash
./start_mapping.sh
```

1. 启动后保持机器狗静止 3-5 秒，等待 "IMU Initial Done"
2. 遥控机器狗在目标区域行走
3. Ctrl+C 结束，PCD 地图保存在 `PCD/` 目录

### 生成 2D 导航地图

推荐使用本地地图工作台，统一归档原始 PCD、生成 ICP/Nav2 地图并管理当前版本：

```bash
./start_map_manager.sh
```

浏览器打开 `http://127.0.0.1:8765`：

1. 点击“归档当前建图”，保存 FASTer-LIO 的最新输出。
2. 从建图会话生成候选地图，自动完成过滤、切片、PGM/YAML 和 3D 预览。
3. 检查 2D/3D 地图后点击“设为当前地图”。

定位 PCD 与 Nav2 PGM/YAML 按版本绑定，导航启动脚本只读取 `maps/active`，不再依赖手工填写路径。

旧的 `pcd2pgm.sh` 保留用于兼容历史流程，新开发流程不要用它切换导航地图。

### 仅定位

```bash
./start_localization.sh
```

用于验证 ICP 定位精度，在 RViz 中用 "2D Pose Estimate" 给定初始位姿。

### 完整导航

```bash
./start_navigation.sh
```

一键启动：MID360 驱动 → ICP 定位 → Go2 桥接 → Nav2 → RViz。

1. RViz 中用 "2D Pose Estimate" 给定初始位姿
2. 用 "Nav2 Goal" 下发导航目标点

## 依赖

- ROS2 Humble
- Nav2
- PCL
- CycloneDDS
- Unitree Go2 SDK (`unitree_sdk2`)
