#!/usr/bin/env bash
# =============================================================================
#  run_mapping.sh —— 正式建图一键流程（防止跑完忘记存图导致地图丢失）
#
#  用法:
#     bash ~/fast_lio_ws/tools/run_mapping.sh
#     bash ~/fast_lio_ws/tools/run_mapping.sh --map ~/maps/arena.pcd
#     bash ~/fast_lio_ws/tools/run_mapping.sh --no-rviz
#     bash ~/fast_lio_ws/tools/run_mapping.sh --record            # 顺便录 90 秒包
#     bash ~/fast_lio_ws/tools/run_mapping.sh --record 180        # 录 180 秒
#
#  流程: 起 bringup(含 RViz) → [可选]起录包 → 你走一圈 → 回车 → 调 /map_save
#        → 出预览图 → 报告 bag 大小
#
#  ★ 为什么必须调 /map_save：
#     累积地图只存在内存变量 pcl_wait_pub 里；源码里 pcl_wait_save 的
#     定期落盘代码是被整段注释掉的（laserMapping.cpp:521-548），
#     Ctrl-C 之后内存就没了，PCD 永远写不出来。
#
#  ★ 为什么录包有秒数上限：
#     录制速率约 3.7 MB/s（374MB / 100s）。SD 卡空间有限，
#     不设上限会写满磁盘导致建图崩溃。默认只录前 90 秒。
# =============================================================================
set +u

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MAP="$HOME/maps/map_$(date +%m%d_%H%M).pcd"
RVIZ="true"
RECORD_SEC=0          # 0 = 不录包

while [ $# -gt 0 ]; do
  case "$1" in
    --map)     MAP="$2"; shift 2 ;;
    --no-rviz) RVIZ="false"; shift ;;
    --record)
      # 支持 "--record" 和 "--record 180" 两种写法
      if [[ "${2:-}" =~ ^[0-9]+$ ]]; then RECORD_SEC="$2"; shift 2
      else RECORD_SEC=90; shift; fi ;;
    -h|--help) sed -n '2,24p' "$0"; exit 0 ;;
    *) echo "未知参数: $1"; exit 1 ;;
  esac
done

source "$WS/ros_env.sh" >/dev/null 2>&1
cd "$WS"

# 1) 把 map_file_path 指到本次输出（备份+自动恢复交给 trap）
CFG="$WS/src/FAST_LIO/config/velodyne.yaml"
cp "$CFG" /tmp/run_mapping.cfg.bak
python3 - "$CFG" "$MAP" <<'PY'
import re, sys
p, m = sys.argv[1], sys.argv[2]
s = open(p, encoding='utf-8').read()
s, n = re.subn(r'map_file_path:\s*"[^"]*"', 'map_file_path: "%s"' % m, s)
assert n == 1, 'map_file_path 匹配 %d 处' % n
open(p, 'w', encoding='utf-8').write(s)
PY
rm -f "$MAP"

# 录包目录
BAGDIR=""
if [ "$RECORD_SEC" -gt 0 ]; then
  mkdir -p "$HOME/bags"
  BAGDIR="$HOME/bags/pi_$(date +%m%d_%H%M)"
  rm -rf "$BAGDIR"
fi

cleanup() {
  pkill -f "ros2 bag record" 2>/dev/null
  pkill -f "ros2 launch fast_lio" 2>/dev/null
  pkill -f fastlio_mapping 2>/dev/null
  pkill -f timestamp_normalizer 2>/dev/null
  pkill -f rviz2 2>/dev/null
  pkill -f velodyne_ 2>/dev/null
  pkill -f serial_node 2>/dev/null
  cp /tmp/run_mapping.cfg.bak "$CFG" 2>/dev/null
  rm -f /tmp/run_mapping.cfg.bak
  echo "[mapping] 配置已还原"
}
trap cleanup EXIT INT TERM

echo "════════════════════════════════════════════════════"
echo "  建图输出: $MAP"
if [ -n "$BAGDIR" ]; then
  echo "  录包输出: $BAGDIR  （限时 ${RECORD_SEC} 秒）"
fi
echo "════════════════════════════════════════════════════"

# 录包前的磁盘检查（约 4 MB/s）
if [ -n "$BAGDIR" ]; then
  AVAIL_MB=$(df -BM --output=avail "$HOME" | tail -1 | tr -dc '0-9')
  NEED_MB=$(( RECORD_SEC * 4 + 200 ))
  echo "[mapping] 磁盘剩余 ${AVAIL_MB} MB，录制 ${RECORD_SEC} 秒预计需要 ${NEED_MB} MB"
  if [ "$AVAIL_MB" -lt "$NEED_MB" ]; then
    echo "[mapping] ⚠️ 磁盘可能不够，录包已取消（建图继续）。建议减少 --record 秒数或清理磁盘。"
    BAGDIR=""
  else
    echo "[mapping] ✅ 磁盘够用"
  fi
fi

# 2) 起链路
ros2 launch fast_lio velodyne_hi12_bringup.launch.py rviz:=$RVIZ > /tmp/run_mapping.log 2>&1 &
LP=$!

# 3) 等链路就绪
echo "[mapping] 等待链路就绪 ..."
for i in $(seq 1 30); do
  sleep 1
  if grep -q "Initialize the map kdtree" /tmp/run_mapping.log 2>/dev/null; then
    echo "[mapping] ✅ 链路就绪（FAST-LIO 已完成初始化）"
    break
  fi
  [ "$i" = "30" ] && { echo "[mapping] ❌ 30 秒内没就绪，看 /tmp/run_mapping.log"; tail -20 /tmp/run_mapping.log; exit 1; }
done
grep -q "规范化启动" /tmp/run_mapping.log && echo "[mapping] ✅ 时间戳规范化节点在跑"

# 3.5) 起录包（限时，不影响建图）
if [ -n "$BAGDIR" ]; then
  echo "[mapping] 开始录包 → $BAGDIR （${RECORD_SEC} 秒后自动停止）"
  timeout "$RECORD_SEC" ros2 bag record -o "$BAGDIR" /velodyne_points /imu/data \
    > /tmp/run_mapping_record.log 2>&1 &
  RP=$!
fi

echo
echo "  现在开始走。建议：速度 ≤1 m/s、角速度 ≤90°/s、单段转角 ≤90°、走一段停 1~2 秒。"
echo "  走完回到起点后按【回车】存图（不要先 Ctrl-C）。"
if [ -n "$BAGDIR" ]; then
  echo "  （录包在后台跑，${RECORD_SEC} 秒后自己停；不影响建图）"
fi
echo
read -r _

# 4) 存图
echo "[mapping] 调用 /map_save ..."
python3 - > /tmp/run_mapping_save.log 2>&1 <<'PY'
import os, sys, rclpy
from std_srvs.srv import Trigger
rclpy.init()
n = rclpy.create_node('run_mapping_saver_%d' % os.getpid())
c = n.create_client(Trigger, '/map_save')
if not c.wait_for_service(timeout_sec=30.0):
    print('NO_SERVICE'); sys.exit(2)
f = c.call_async(Trigger.Request())
rclpy.spin_until_future_complete(n, f, timeout_sec=60.0)
ok = f.done() and f.result() is not None and f.result().success
print('RESULT', ok)
n.destroy_node(); rclpy.shutdown()
sys.exit(0 if ok else 3)
PY
echo "[mapping] 存图返回: $(cat /tmp/run_mapping_save.log | tr '\n' ' ')"
sleep 2

# 5) 出预览图
if [ -f "$MAP" ]; then
  echo "[mapping] ✅ 地图已写出: $(ls -lh "$MAP" | awk '{print $5}')  $MAP"
  echo "[mapping] 生成预览图 ..."
  python3 "$WS/tools/render_map.py" --pcd "$MAP" --out "${MAP%.pcd}.png" 2>&1 | tail -5
  echo "[mapping] 预览图: ${MAP%.pcd}.png"
else
  echo "[mapping] ❌ 没有生成 $MAP"
  echo "--- 日志尾部 ---"; tail -25 /tmp/run_mapping.log
fi

# 6) 报告 bag
if [ -n "$BAGDIR" ]; then
  sleep 1
  if [ -d "$BAGDIR" ] && ls "$BAGDIR"/*.db3 >/dev/null 2>&1; then
    echo "[mapping] ✅ bag 已写出: $(du -sh "$BAGDIR" | cut -f1)  $BAGDIR"
    echo "           话题: $(grep -oE '/[a-z_/]+' "$BAGDIR/metadata.yaml" 2>/dev/null | sort -u | tr '\n' ' ')"
    echo "           回放: ros2 bag play $BAGDIR --rate 1"
  else
    echo "[mapping] ⚠️ bag 没写成（看 /tmp/run_mapping_record.log）"
    tail -5 /tmp/run_mapping_record.log 2>/dev/null
  fi
  echo "[mapping] 磁盘剩余: $(df -h "$HOME" | tail -1 | awk '{print $4}')"
fi

echo
echo "[mapping] 按回车结束（会关闭所有节点）"
read -r _ || true
