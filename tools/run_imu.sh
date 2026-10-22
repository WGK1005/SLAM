#!/usr/bin/env bash
# 启动超核 HI12 IMU 驱动（hipnuc_imu/serial_node）
#
# 用法：
#   tools/run_imu.sh                    # 默认 /dev/ttyUSB0 @ 115200
#   tools/run_imu.sh /dev/ttyUSB0 230400
#
# 为什么需要这个脚本：
#   1) 串口属于 dialout 组，如果当前 shell 是在 usermod -aG dialout 之前启动的，
#      进程组里就没有 dialout → 打开串口会 Permission denied。
#      脚本检测到这种情况会自动用 `sg dialout` 提权。
#   2) 顺手把 ROS 环境按正确顺序 source 好。
set -u
PORT="${1:-/dev/ttyUSB0}"
BAUD="${2:-115200}"

if [ ! -e "$PORT" ]; then
  echo "[run_imu] 找不到串口 $PORT：IMU 是否已 usbipd attach？" >&2
  exit 1
fi

CMD="source /opt/ros/humble/setup.bash && source /home/wgk/fast_lio_ws/install/setup.bash && \
     exec ros2 run hipnuc_imu serial_node --ros-args -p port:=$PORT -p baudrate:=$BAUD"

if id -nG | tr ' ' '\n' | grep -qx dialout; then
  echo "[run_imu] 当前进程已在 dialout 组，直接启动 $PORT @ $BAUD"
  exec bash --noprofile --norc -c "$CMD"
else
  echo "[run_imu] 当前进程不在 dialout 组，用 sg dialout 提权启动 $PORT @ $BAUD"
  exec sg dialout -c "bash --noprofile --norc -c '$CMD'"
fi
