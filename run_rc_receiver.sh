#!/bin/sh
# rc_receiver 启动脚本 (放在 /home/root/ 下与 rc_receiver 同目录)
# 每次改 scale 或 turn_ratio 只需改这里, 不用每次敲一长串参数

PORT=9876
DEV=/dev/tb6612
TURN_RATIO=1.0
LEFT_SCALE=1.0
RIGHT_SCALE=0.8
TIMEOUT=1000
MAX_PCT=40        # 最高速度限制 (防止小车太快)

# 可选: --dry-run 看日志但不驱动电机
DRY_RUN=

exec ./rc_receiver \
    --port $PORT \
    --dev $DEV \
    --turn-ratio $TURN_RATIO \
    --left-scale $LEFT_SCALE \
    --right-scale $RIGHT_SCALE \
    --timeout $TIMEOUT \
    --max-pct $MAX_PCT \
    $DRY_RUN
