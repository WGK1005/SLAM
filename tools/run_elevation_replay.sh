#!/usr/bin/env bash
# =============================================================================
#  run_elevation_replay.sh —— FAST-LIO + elevation_mapping_cupy 一键回放建高程图
# =============================================================================
set -e

WS="/home/wgk/fast_lio_ws"
BAG="${1:-$HOME/bags/pi_1021_1050}"
RVIZ_CFG="$WS/tools/elevation_fastlio.rviz"

if [ ! -d "$BAG" ]; then
  echo "❌ 找不到数据包: $BAG"
  exit 1
fi

source "$WS/ros_env.sh" >/dev/null 2>&1
cd "$WS"

cleanup() {
  echo
  echo "🛑 正在停止高程图仿真节点..."
  pkill -P $$ 2>/dev/null || true
  pkill -f "elevation_mapping_node" 2>/dev/null || true
  pkill -f "fastlio_mapping" 2>/dev/null || true
  pkill -f "timestamp_normalizer" 2>/dev/null || true
  pkill -f "ros2 bag play" 2>/dev/null || true
  pkill -f "rviz2" 2>/dev/null || true
  echo "✅ 高程图节点已安全退出"
}
trap cleanup EXIT INT TERM

echo "════════════════════════════════════════════════════"
echo "  FAST-LIO + elevation_mapping_cupy (GPU 实时高程图)"
echo "  数据包 (BAG): $BAG"
echo "  网格分辨率  : 0.04m (4cm 精细地形)"
echo "  地图范围    : 8.0m x 8.0m 局部随动"
echo "════════════════════════════════════════════════════"

# 1. 启动时间戳规范化节点
echo "[1/4] 启动点云时间戳规范化节点..."
python3 tools/timestamp_normalizer.py --ros-args -p use_sim_time:=true >/tmp/elev_norm.log 2>&1 &
sleep 1

# 2. 启动 FAST-LIO 里程计与运动补偿节点
echo "[2/4] 启动 FAST-LIO 运动补偿与高频里程计..."
ros2 launch fast_lio mapping.launch.py config_file:=velodyne.yaml rviz:=false use_sim_time:=true >/tmp/elev_fastlio.log 2>&1 &
sleep 2

# 3. 启动 GPU 加速高程图节点 (elevation_mapping_cupy)
echo "[3/4] 启动 GPU 2.5D 高程图节点 (RTX 4060 加速)..."
ros2 launch elevation_mapping_cupy elevation_mapping.launch.py \
    robot_config:=vlp16_fastlio.yaml launch_rviz:=false use_sim_time:=true >/tmp/elev_node.log 2>&1 &
sleep 2

# 4. 启动专用 RViz2 显示高程图与法向量
echo "[4/4] 启动 RViz2 (含 GridMap 地形渲染插件)..."
if [ -f "$RVIZ_CFG" ]; then
  rviz2 -d "$RVIZ_CFG" >/dev/null 2>&1 &
else
  rviz2 >/dev/null 2>&1 &
fi

echo "⏳ 等待所有系统节点就绪 (3秒)..."
sleep 3

# 5. 回放数据包
echo "🚀 开始回放数据包..."
echo "────────────────────────────────────────────────────"
echo "  提示："
echo "  · RViz 中可实时查看 2.5D 彩色高程网格与地面起伏"
echo "  · 随时按【Ctrl+C】可一键终止所有程序并清理后台"
echo "────────────────────────────────────────────────────"
ros2 bag play "$BAG" --clock --rate 1 </dev/null

echo "数据包播放完毕，按回车退出..."
read -r _
