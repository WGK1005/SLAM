#!/usr/bin/env bash
# ==============================================================================
#  纯原版 3D 激光点云 (FAST-LIO2) 一键回放脚本 (无任何高程图，纯点云SLAM)
# ==============================================================================
set -e

# 清除 Windows 挂载盘环境变量干扰
unset PKG_CONFIG_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH AMENT_PREFIX_PATH \
      LD_LIBRARY_PATH PYTHONPATH DDSGEN AMENT_CURRENT_PREFIX

source /opt/ros/humble/setup.bash
source /home/wgk/fast_lio_ws/install/setup.bash

export DISPLAY="${DISPLAY:-:0}"

# 默认回放包：支持参数传入，如 bash replay_pure_pointcloud.sh pc_base2
DEFAULT_BAG="/home/wgk/bags/pi_1021_1050"
BAG_INPUT="${1:-$DEFAULT_BAG}"

if [ ! -d "$BAG_INPUT" ]; then
    # 尝试在 ~/bags/ 查找
    if [ -d "/home/wgk/bags/$BAG_INPUT" ]; then
        BAG_PATH="/home/wgk/bags/$BAG_INPUT"
    else
        echo "❌ 找不到数据包: $BAG_INPUT"
        echo "   可用数据包有: pi_1021_1050, pc_base2, stairs_test_03"
        exit 1
    fi
else
    BAG_PATH="$BAG_INPUT"
fi

BAG_NAME=$(basename "$BAG_PATH")

echo "======================================================================"
echo "  🚀 启动【纯原版 3D 激光点云】回放系统 (FAST-LIO2 纯建图)"
echo "  • 当前回放包: $BAG_NAME ($BAG_PATH)"
echo "  • 节点模式: 纯 3D FAST-LIO SLAM (不加载任何高程图/网格/模型)"
echo "  • 显示内容: /cloud_registered (高精度 3D 去畸变点云) + /path (轨迹)"
echo "  • 安全性: 只读回放，绝不保存或修改任何 PCD 文件"
echo "======================================================================"

cleanup() {
    echo
    echo "正在停止所有后台节点..."
    kill $PID_NORM $PID_FASTLIO 2>/dev/null || true
    pkill -f "ros2 bag play" 2>/dev/null || true
    echo "✅ 纯点云回放已退出"
}
trap cleanup EXIT INT TERM

# 1. 启动时间戳规范化节点
python3 /home/wgk/fast_lio_ws/tools/timestamp_normalizer.py &
PID_NORM=$!

# 2. 启动纯 FAST-LIO2 (自带官方纯点云 RViz2)
ros2 launch fast_lio mapping.launch.py config_file:=velodyne.yaml rviz:=true &
PID_FASTLIO=$!

sleep 4

echo "======================================================================"
echo "  ▶️  开始回放数据包: $BAG_NAME"
echo "  • 按【空格键】暂停/继续"
echo "  • 按【Ctrl + C】退出"
echo "======================================================================"
ros2 bag play "$BAG_PATH" --rate 1.0

echo "======================================================================"
echo "  🎉 数据包回放完成！"
echo "======================================================================"
