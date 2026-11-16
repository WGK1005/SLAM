#!/usr/bin/env bash
# =============================================================================
#  run_rolling_map.sh —— 独立滚动特征匹配与在线“保留新而删旧”演示脚本
#  · 零改动原有开机重定位 (open3d_loc) 与 FAST-LIO
#  · 原先验地图文件严格只读，绝不修改
# =============================================================================
set -e

WS="/home/wgk/fast_lio_ws"
if [ $# -eq 1 ] && [[ "$1" == *.pcd ]]; then
  BAG="$HOME/bags/pi_1021_1050"
  MAP="$1"
else
  BAG="${1:-$HOME/bags/pi_1021_1050}"
  if [ -z "${2:-}" ] && [[ "$(basename "$BAG")" == "stairs_test_03" ]]; then
    MAP="$HOME/maps/stairs_test_03_map.pcd"
  else
    MAP="${2:-$HOME/maps/map_1021_1050.pcd}"
  fi
fi
RVIZ_CFG="$WS/src/rolling_map_updater/rviz/rolling_map.rviz"

if [ ! -d "$BAG" ]; then
  if [ -d "$HOME/bags/$1" ]; then
    BAG="$HOME/bags/$1"
  else
    echo "❌ 找不到数据包: $BAG"
    exit 1
  fi
fi

if [ ! -f "$MAP" ]; then
  echo "❌ 找不到只读先验地图: $MAP"
  exit 1
fi

source "$WS/ros_env.sh" >/dev/null 2>&1
cd "$WS"

cleanup() {
  echo
  echo "🛑 正在停止滚动地图节点..."
  pkill -P $$ 2>/dev/null || true
  pkill -f "rolling_map_node" 2>/dev/null || true
  pkill -f "auto_global_reloc_node" 2>/dev/null || true
  pkill -f "fastlio_mapping" 2>/dev/null || true
  pkill -f "timestamp_normalizer" 2>/dev/null || true
  pkill -f "ros2 bag play" 2>/dev/null || true
  pkill -f "rviz2" 2>/dev/null || true
  echo "✅ 已安全退出（原地图文件未受任何修改）"
}
trap cleanup EXIT INT TERM

echo "════════════════════════════════════════════════════════════"
echo "  滚动特征匹配 + 实时保留新而删旧 (Standalone)"
echo "  数据包 (BAG)     : $BAG"
echo "  只读先验地图     : $MAP"
echo "  滚动活跃窗口     : 12.0m 半径 (8cm 体素, 2m 空间哈希分块)"
echo "════════════════════════════════════════════════════════════"

echo "[1/5] 启动点云时间戳规范化节点..."
python3 tools/timestamp_normalizer.py >/tmp/roll_norm.log 2>&1 &
sleep 1

echo "[2/5] 启动 FAST-LIO 里程计 (仅作为局部运动载体，保持原封不动)..."
ros2 launch fast_lio mapping.launch.py config_file:=velodyne.yaml rviz:=false >/tmp/roll_fastlio.log 2>&1 &
sleep 2

echo "[3/5] 启动现有开机重定位节点 auto_global_reloc (提供 /initialpose)..."
ros2 run open3d_loc auto_global_reloc_node --ros-args \
    -p map_path:="$MAP" \
    -p scan_topic:="/velodyne_points_norm" \
    -p auto_exit:=true \
    -p publish_map:=false >/tmp/roll_reloc.log 2>&1 &
sleep 1

echo "[4/5] 启动独立滚动匹配与实时地图更正节点 (rolling_map_node)..."
OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 ros2 run rolling_map_updater rolling_map_node --ros-args \
    --params-file "$WS/src/rolling_map_updater/config/rolling_map.yaml" \
    -p prior_map_path:="$MAP" >/tmp/roll_updater.log 2>&1 &
sleep 2

echo "[5/5] 启动 RViz2 可视化..."
rviz2 -d "$RVIZ_CFG" >/dev/null 2>&1 &
sleep 2

echo "🚀 开始回放数据包..."
echo "  · 灰白底图 : 只读原先验背景与滚动活跃子图 (内含蓝色已保留新点) (/rolling_map/active_submap)"
echo "  · 蓝色体素 : 原地图没有、实时新增并确认保留的静态点 (/rolling_map/new_points)"
echo "  · 红色旧点 : 原地图有、实时没有（射线穿透判断后删除） (/rolling_map/removed_points)"
echo "  · 绿/红扫描 : 当前帧实时扫描（正常匹配=绿色，匹配不上=红色） (/rolling_map/aligned_scan)"
ros2 bag play "$BAG" --rate 1 </dev/null

echo "回放结束，按回车退出..."
read -r _
