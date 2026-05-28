#!/bin/sh
# start_vehicle.sh — 一键启动小车
# 用法: ./start_vehicle.sh [上位机IP] [GPS设备] [配置文件]
# 默认: ./start_vehicle.sh 192.168.1.100 /dev/ttyUSB1 ./config.yaml

PC_IP="${1:-192.168.1.100}"
GPS_DEV="${2:-/dev/ttyUSB1}"
CONFIG="${3:-./config.yaml}"

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "=== 启动小车系统 ==="
echo "上位机IP:  $PC_IP"
echo "GPS设备:   $GPS_DEV"
echo "配置文件:  $CONFIG"
echo "GD32设备:  /dev/ttyUSB0 (固定)"
echo "工作目录:  $SCRIPT_DIR"

# 检查 GD32 USB-TTL 是否存在
if [ ! -e /dev/ttyUSB0 ]; then
    echo "警告: /dev/ttyUSB0 不存在 (GD32未连接?)"
    echo "gd32_bridge 将等待设备出现..."
fi

# 检查 GPS 设备是否存在
if [ ! -e "$GPS_DEV" ]; then
    echo "警告: $GPS_DEV 不存在 (GPS未连接?)"
    echo "gnss_path_control 将等待设备出现..."
fi

# 启动桥接守护进程（后台运行）
echo "启动 gd32_bridge ..."
"$SCRIPT_DIR/gd32_bridge" "$PC_IP" 8766 &
BRIDGE_PID=$!
echo "  PID: $BRIDGE_PID"

# 短暂等待，让 bridge 先创建好 Unix socket
sleep 1

# GPS 设备可能还没准备好，用 until 循环等待
echo "启动 GPS 路径规划 ..."
until cat "$GPS_DEV" 2>/dev/null | "$SCRIPT_DIR/KF-GINS-GnssPathControl" "$CONFIG"; do
    echo "等待 GPS 设备 $GPS_DEV ..." >&2
    sleep 2
done

# GnssPathControl 退出后，清理 bridge
echo "停止 gd32_bridge ..."
kill "$BRIDGE_PID" 2>/dev/null
wait "$BRIDGE_PID" 2>/dev/null
echo "=== 小车系统已停止 ==="
