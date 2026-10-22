#!/bin/bash
# ==============================================================================
#  开机无先验初值全局重定位 一键执行脚本 (完全独立于 open3d_loc 跟踪系统)
# ==============================================================================
set -e

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$DIR/.." && pwd)"

# 环境变量设置
export DISPLAY="${DISPLAY:-:0}"
source "$WS_DIR/ros_env.sh" 2>/dev/null || true
if [ -f "$WS_DIR/install/setup.bash" ]; then
    source "$WS_DIR/install/setup.bash"
fi

MAP_PATH="/home/wgk/maps/map_1021_1050.pcd"
SCAN_TOPIC="/velodyne_points"
MODE="ros" # 'ros' 或 'offline'
OPEN_RVIZ=false
RVIZ_CFG="$WS_DIR/src/open3d_loc/rviz_cfg/fastlio.rviz"

# 参数解析
EXTRA_ARGS=()
while [[ $# -gt 0 ]]; do
    case $1 in
        --rviz)
            OPEN_RVIZ=true
            shift
            ;;
        --offline)
            MODE="offline"
            shift
            ;;
        --map)
            MAP_PATH="$2"
            shift 2
            ;;
        --topic)
            SCAN_TOPIC="$2"
            shift 2
            ;;
        *)
            EXTRA_ARGS+=("$1")
            shift
            ;;
    esac
done

if [ ! -f "$MAP_PATH" ]; then
    echo -e "\033[1;31m❌ 先验地图不存在: $MAP_PATH\033[0m"
    exit 1
fi

echo -e "\033[1;34m==================================================================\033[0m"
echo -e "\033[1;32m  🚀 开机无先验全局重定位启动器 (独立双方法架构)\033[0m"
echo -e "  先验地图: \033[1;33m$MAP_PATH\033[0m"
echo -e "  工作模式: \033[1;36m$MODE\033[0m"
echo -e "  启动RViz: \033[1;35m$OPEN_RVIZ\033[0m"
echo -e "\033[1;34m==================================================================\033[0m"

# 若需要打开 RViz，在后台启动 RViz2
if [ "$OPEN_RVIZ" = true ]; then
    echo -e "🖥️ \033[1;35m正在启动 RViz2 可视化界面 ($RVIZ_CFG)...\033[0m"
    rviz2 -d "$RVIZ_CFG" >/dev/null 2>&1 &
    RVIZ_PID=$!
    sleep 2
fi

cleanup() {
    if [ -n "$RVIZ_PID" ]; then
        kill "$RVIZ_PID" 2>/dev/null || true
    fi
}
trap cleanup EXIT INT TERM

if [ "$MODE" == "offline" ]; then
    echo -e "\033[1;36m[模式: 纯离线单帧自定位测试]\033[0m"
    "$WS_DIR/tools/test_global_reloc" --map "$MAP_PATH" "${EXTRA_ARGS[@]}"
    if [ "$OPEN_RVIZ" = true ]; then
        echo -e "\n📡 \033[1;32m正在向 RViz 广播匹配全流程点云与位姿...\033[0m"
        echo -e "   - ⚪ 白色/灰白点云 : /map (先验地图)"
        echo -e "   - 🔴 红色点云       : /scan2map (阶段一: 粗匹配候选点云)"
        echo -e "   - 🟢 绿色点云       : /cloud_registered (阶段二: 两级ICP高精咬合)"
        echo -e "   - 🚩 红色位姿与TF   : /initialpose 与 map -> base_link"
        echo -e "   (在 RViz 中可 360° 旋转缩放观察，按 Ctrl+C 退出)\n"
        PYTHONNOUSERSITE=1 python3 "$WS_DIR/tools/publish_reloc_to_rviz.py"
    fi
else
    echo -e "\033[1;36m[模式: ROS 2 在线开机自定位节点 (监听 $SCAN_TOPIC -> 发布 /initialpose, /map, /cloud_registered_map)]\033[0m"
    AUTO_EXIT=true
    if [ "$OPEN_RVIZ" = true ]; then
        AUTO_EXIT=false # RViz 模式下保持运行以维持 TF 与地图广播，按 Ctrl+C 退出
    fi

    ros2 run open3d_loc auto_global_reloc_node \
        --ros-args \
        -p map_path:="$MAP_PATH" \
        -p scan_topic:="$SCAN_TOPIC" \
        -p initialpose_topic:="/initialpose" \
        -p auto_exit:="$AUTO_EXIT" \
        -p publish_map:=true
fi
