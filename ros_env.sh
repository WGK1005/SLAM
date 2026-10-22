#!/usr/bin/env bash
# 干净环境脚本：只加载 ROS 2 Humble + 本工作区
#
# 为什么需要它：
#   你的 shell 里带有 /mnt/d/d2lros2/fastdds_ws 的 PKG_CONFIG_PATH / CMAKE_PREFIX_PATH /
#   LD_LIBRARY_PATH / PYTHONPATH，会污染 colcon 构建和运行期库搜索路径。
#   本脚本先清掉这些变量，再按正确顺序 source。
#
# 用法（每个新终端都执行一次）：
#   source ~/fast_lio_ws/ros_env.sh
#   ros2 launch fast_lio velodyne_hi12_bringup.launch.py

unset PKG_CONFIG_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH AMENT_PREFIX_PATH \
      LD_LIBRARY_PATH PYTHONPATH DDSGEN AMENT_CURRENT_PREFIX

# ── WSL 网络抖动兜底 ────────────────────────────────────────────────
# 这台机器 eth2 的地址会反复变化（172.23.x ↔ 192.168.198.x），而 Fast-DDS 的组播
# 在参与进程创建时就绑死在接口上，地址一变所有节点互相失联——现象是
# `ros2 topic list` 只看到自己、/map_save 服务"不存在"、节点明明在跑却连不上。
# ★ 默认【不开】，而且实测【不要开】：启用后 fastlio_mapping(C++/rclcpp) 在创建 DDS 参与者时
#   直接挂死（日志停在 "p_pre->lidar_type 2" 之后再无输出，ros2 node list 为空）。
#   笔记本和香橙派上均复现。保留此开关仅作历史记录，别用。
# 需要跨机器通信（例如 Windows 上跑 RViz 连 WSL）时：
#     export FASTLIO_DDS_LOOPBACK=0 && source ~/fast_lio_ws/ros_env.sh
_WSDIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ "${FASTLIO_DDS_LOOPBACK:-0}" = "1" ] && [ -f "$_WSDIR/tools/fastdds_local.xml" ]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="$_WSDIR/tools/fastdds_local.xml"
fi
unset _WSDIR

source /opt/ros/humble/setup.bash
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/install/setup.bash"

echo "[ros_env] ROS_DISTRO=$ROS_DISTRO  ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0(unset)}  DDS=$([ -n "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" ] && echo loopback-only || echo 默认)"
echo "[ros_env] fast_lio   -> $(ros2 pkg prefix fast_lio 2>/dev/null || echo '未找到，先 colcon build')"
echo "[ros_env] hipnuc_imu -> $(ros2 pkg prefix hipnuc_imu 2>/dev/null || echo '未找到，先 colcon build')"
