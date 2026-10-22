#!/usr/bin/env bash
# =============================================================================
#  sim_replay_loc.sh —— FAST-LIO + open3d_loc 一键回放仿真
# =============================================================================
set -e

WS="/home/wgk/fast_lio_ws"
# 默认使用 10.21 实验室新录制包与对应地图
BAG="${1:-$HOME/bags/pi_1021_1050}"
MAP="${2:-$HOME/maps/map_1021_1050.pcd}"
RVIZ_CFG="$WS/src/open3d_loc/rviz_cfg/fastlio.rviz"

if [ ! -d "$BAG" ]; then
  echo "❌ 找不到数据包: $BAG"
  exit 1
fi

if [ ! -f "$MAP" ]; then
  echo "❌ 找不到先验地图: $MAP"
  exit 1
fi

source "$WS/ros_env.sh" >/dev/null 2>&1
cd "$WS"

cleanup() {
  echo
  echo "🛑 正在停止所有仿真节点..."
  pkill -P $$ 2>/dev/null || true
  pkill -f "timestamp_normalizer" 2>/dev/null || true
  pkill -f "global_localization_node" 2>/dev/null || true
  pkill -f "fastlio_mapping" 2>/dev/null || true
  pkill -f "ros2 bag play" 2>/dev/null || true
  pkill -f "rviz2" 2>/dev/null || true
  pkill -f "static_transform_publisher" 2>/dev/null || true
  echo "✅ 仿真已安全退出"
}
trap cleanup EXIT INT TERM

echo "════════════════════════════════════════════════════"
echo "  FAST-LIO + open3d_loc 一键回放仿真"
echo "  底图 (MAP): $MAP"
echo "  数据包 (BAG): $BAG"
echo "════════════════════════════════════════════════════"

# 1. 启动时间戳规范化节点 (/velodyne_points -> /velodyne_points_norm, /imu/data -> /imu/data_norm)
echo "[1/5] 启动时间戳规范化节点..."
python3 tools/timestamp_normalizer.py >/tmp/sim_norm.log 2>&1 &
sleep 1

# 2. 启动 FAST-LIO 建图与里程计节点
echo "[2/5] 启动 FAST-LIO 节点..."
ros2 launch fast_lio mapping.launch.py config_file:=velodyne.yaml rviz:=false >/tmp/sim_fastlio.log 2>&1 &
sleep 2

# 3. 启动 open3d_loc 全局定位节点与静态 TF
echo "[3/5] 启动 open3d_loc 全局定位节点..."
ros2 launch open3d_loc loc_vlp16.launch.py map_file:="$MAP" >/tmp/sim_loc.log 2>&1 &
sleep 2

# 4. 启动 RViz2
echo "[4/5] 启动 RViz2 专用可视化界面..."
rviz2 -d "$RVIZ_CFG" >/dev/null 2>&1 &

echo "⏳ 等待所有节点就绪 (3秒)..."
sleep 3

# 5. 开始回放数据包（阻塞在前台）
echo "[5/5] 🚀 开始回放数据包..."
echo "────────────────────────────────────────────────────"
echo "  提示："
echo "  · 可以在 RViz 中点击【2D Pose Estimate】给机器人粗坐标"
echo "  · 随时按【Ctrl+C】可一键终止所有程序并清理后台"
echo "────────────────────────────────────────────────────"
ros2 bag play "$BAG" --rate 1 </dev/null

echo "数据包播放完毕，按回车退出仿真..."
read -r _

