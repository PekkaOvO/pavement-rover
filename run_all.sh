#!/bin/sh
# 一键启动: rc_receiver + gd32_bridge 同时运行
# 放在 /home/root/ 下
#
# 用法: ./run_all.sh <PC_IP>
#   PC_IP: CarView2 上位机 IP (gd32 图片/数据发到这个地址)

if [ -z "$1" ]; then
    echo "Usage: $0 <PC_IP>"
    echo "  PC_IP: CarView2上位机IP, e.g. 172.20.10.11"
    exit 1
fi

PC_IP=$1

echo "starting rc_receiver (TCP:9876)..."
./run_rc_receiver.sh &
PID_RC=$!

echo "starting gd32_bridge (TCP:8766 -> $PC_IP)..."
./gd32_bridge "$PC_IP" &
PID_GD=$!

echo "=== both running ==="
echo "  rc_receiver  pid=$PID_RC"
echo "  gd32_bridge  pid=$PID_GD"
echo "  press CTRL+C to stop both"

trap "kill $PID_RC $PID_GD 2>/dev/null; echo 'stopped'" INT TERM
wait
