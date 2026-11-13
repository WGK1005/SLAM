#!/usr/bin/env bash
# ==============================================================================
#  wrl716 工作区环境配置脚本
# ==============================================================================
unset PKG_CONFIG_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH AMENT_PREFIX_PATH \
      LD_LIBRARY_PATH PYTHONPATH DDSGEN AMENT_CURRENT_PREFIX

source /opt/ros/humble/setup.bash
source /home/wgk/wrl716/ros2_ws/install/setup.bash

echo "[wrl716] ROS 2 Humble 与 wj_716_lidar 驱动环境已加载"
