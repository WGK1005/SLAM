#!/usr/bin/env bash
# ==============================================================================
#  WLR-716 2D 激光雷达 ROS 2 驱动一键启动脚本
#  用法:
#    bash run_wlr716.sh [雷达IP] [端口]
#    例如: bash run_wlr716.sh 192.168.0.2 2110
# ==============================================================================
set -e

LIDAR_IP="${1:-192.168.100.104}"
LIDAR_PORT="${2:-2110}"

source /home/wgk/wrl716/ros_env.sh

echo "======================================================================"
echo "  🚀 启动万集 / 华睿 WLR-716 激光雷达 ROS 2 驱动"
echo "  • 目标雷达 IP : $LIDAR_IP"
echo "  • 目标端口    : $LIDAR_PORT"
echo "  • 输出话题 1  : /scan (sensor_msgs/msg/LaserScan, 首重回波)"
echo "  • 输出话题 2  : /scan_TwoEcho (sensor_msgs/msg/LaserScan, 双重回波)"
echo "======================================================================"

ros2 launch wj_716_lidar wj_716_lidar.launch.py hostname:="$LIDAR_IP" port:="$LIDAR_PORT"
