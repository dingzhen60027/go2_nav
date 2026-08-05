#!/bin/bash
# Go2 ROS2 环境配置 — CycloneDDS 直连 Go2
#
# 用法: source setup_go2.sh [网卡名]
# 默认网卡: enx6c1ff7bc241e (Go2 USB 以太网)

IFACE="${1:-${GO2_IFACE:-enx6c1ff7bc241e}}"
export GO2_NAV_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "=== Go2 ROS2 Environment ==="
echo "Network interface: ${IFACE}"

source /opt/ros/humble/setup.bash
source "${GO2_NAV_ROOT}/install/setup.bash"

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces>
    <NetworkInterface name=\"${IFACE}\" priority=\"default\" multicast=\"default\" />
</Interfaces></General></Domain></CycloneDDS>"

echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}"
echo "Ready."
