#!/usr/bin/env bash
# 一键停止所有与本次实验相关的程序（FAST-LIO / 雷达驱动 / IMU / RViz / 合成数据）
# 用法：bash ~/fast_lio_ws/tools/stop_all.sh
set -u

echo "=== 终止 ROS 节点 ==="
for pat in fastlio_mapping velodyne_driver_node velodyne_transform_node \
           serial_node rviz2 synthetic_sensor_pub bench_fastlio "bag play"; do
  if pkill -f "$pat" 2>/dev/null; then
    echo "  已终止: $pat"
  fi
done
sleep 2

echo "=== 终端 ros2 CLI 守护进程（只是发现缓存，杀掉无副作用） ==="
if pgrep -f "ros2cli.daemon.daemonize" >/dev/null 2>&1; then
  pkill -f "ros2cli.daemon.daemonize" 2>/dev/null
  sleep 1
  if pgrep -f "ros2cli.daemon.daemonize" >/dev/null 2>&1; then
    pkill -9 -f "ros2cli.daemon.daemonize" 2>/dev/null
    sleep 1
  fi
  if pgrep -f "ros2cli.daemon.daemonize" >/dev/null 2>&1; then
    echo "  ⚠️ ros2 daemon 未终止（不影响任何功能，可忽略）"
  else
    echo "  已终止: ros2 daemon"
  fi
else
  echo "  未运行"
fi

echo "=== 检查结果 ==="
left=$(pgrep -a -f "fastlio|velodyne|serial_node|rviz2|synthetic_sensor|bag play" 2>/dev/null)
if [ -z "$left" ]; then
  echo "  ✅ ROS 节点已全部终止，无残留"
else
  echo "  ⚠️ 仍有残留："; echo "$left" | sed 's/^/    /'
fi
echo
echo "提示：雷达/IMU 硬件本身不会因此停止（雷达仍在转、串口仍接在 WSL）"
