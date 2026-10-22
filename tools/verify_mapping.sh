#!/usr/bin/env bash
# 端到端验证：完全按生产流程（ros_env.sh + 手动 launch + 规范化节点 + 回放 + map_save）
# 目的：确认"能不能正常建图"，与 scan_time_offset.py 的启动方式无关
# 注意：不要用 set -u。ROS 的 /opt/ros/humble/setup.bash 不是 set -u 安全的
# （会在 AMENT_TRACE_SETUP_FILES 上报 unbound variable），source 它必须先关掉 set -u。
WS=/home/wgk/fast_lio_ws
CFG=$WS/src/FAST_LIO/config/velodyne.yaml
OUT=/home/wgk/maps/verify.pcd

set +u
source $WS/ros_env.sh >/dev/null 2>&1
cd $WS

# 1) 临时把 map_file_path 指向 verify.pcd（备份原配置）
cp "$CFG" /tmp/velodyne.yaml.bak
python3 - "$CFG" <<'PY'
import re, sys
p = sys.argv[1]
s = open(p, encoding='utf-8').read()
s2, n = re.subn(r'map_file_path:\s*"[^"]*"', 'map_file_path: "/home/wgk/maps/verify.pcd"', s)
assert n == 1, f'map_file_path 匹配 {n} 处'
open(p, 'w', encoding='utf-8').write(s2)
print('[verify] map_file_path -> ~/maps/verify.pcd')
PY
rm -f "$OUT"

# 2) 起链路：规范化节点 → FAST-LIO
nohup python3 tools/timestamp_normalizer.py >/tmp/v_norm.log 2>&1 &
NORM=$!
sleep 3
nohup ros2 launch fast_lio mapping.launch.py config_file:=velodyne.yaml rviz:=false >/tmp/v_map.log 2>&1 &
sleep 10

echo "[verify] 节点: $(ros2 node list --no-daemon 2>/dev/null | tr '\n' ' ')"
echo "[verify] 话题: $(ros2 topic list --no-daemon 2>/dev/null | grep -cE 'velodyne|imu|Odometry')"
echo "[verify] /map_save 可见: $(ros2 service list --no-daemon 2>/dev/null | grep -c map_save)"

# 3) 回放整段动态 bag
echo "[verify] 开始回放 61 秒 ..."
timeout 70 ros2 bag play /home/wgk/bags/dyn_1017_1345 --rate 1 >/tmp/v_play.log 2>&1
echo "[verify] 回放结束"

# 4) 存图（rclpy 客户端）
python3 - >/tmp/v_save.log 2>&1 <<'PY'
import os, sys, rclpy
from std_srvs.srv import Trigger
rclpy.init()
n = rclpy.create_node('verify_saver_%d' % os.getpid())
c = n.create_client(Trigger, '/map_save')
if not c.wait_for_service(timeout_sec=30.0):
    print('NO_SERVICE'); sys.exit(2)
f = c.call_async(Trigger.Request())
rclpy.spin_until_future_complete(n, f, timeout_sec=30.0)
print('RESULT', f.done() and f.result() is not None and f.result().success)
sys.exit(0)
PY
echo "[verify] 存图返回: $(cat /tmp/v_save.log | tr '\n' ' ')"
sleep 3

# 5) 收尾
pkill -f timestamp_normalizer 2>/dev/null
pkill -f fastlio_mapping 2>/dev/null
pkill -f "ros2 launch fast_lio" 2>/dev/null
sleep 2
cp /tmp/velodyne.yaml.bak "$CFG"
echo "[verify] 配置已还原"

# 6) 结果
if [ -f "$OUT" ]; then
  echo "[verify] ✅ 出图成功: $(ls -lh "$OUT" | awk '{print $5}')"
  python3 - "$OUT" <<'PY'
import sys, numpy as np
p = sys.argv[1]
fields, sizes, types, points, fmt = [], [], [], 0, None
with open(p, 'rb') as f:
    while True:
        line = f.readline()
        if not line: raise SystemExit('bad header')
        k = line.split()[0].decode(errors='ignore').upper()
        if k == 'FIELDS': fields = [x.decode() for x in line.split()[1:]]
        elif k == 'SIZE': sizes = [int(x) for x in line.split()[1:]]
        elif k == 'TYPE': types = [x.decode() for x in line.split()[1:]]
        elif k == 'POINTS': points = int(line.split()[1])
        elif k == 'DATA':
            fmt = line.split()[1].decode().lower(); break
    tmap = {'F': 'f', 'U': 'u', 'I': 'i'}
    dt = np.dtype([(n, f'<{tmap[t]}{s}') for n, s, t in zip(fields, sizes, types)])
    a = np.frombuffer(f.read(points * dt.itemsize), dtype=dt)
P = np.stack([np.asarray(a['x'], float), np.asarray(a['y'], float), np.asarray(a['z'], float)], 1)
P = P[np.isfinite(P).all(1)]
r = np.linalg.norm(P, axis=1)
print(f'  点数 {len(P)}')
print(f'  x∈[{P[:,0].min():.2f},{P[:,0].max():.2f}] y∈[{P[:,1].min():.2f},{P[:,1].max():.2f}] z∈[{P[:,2].min():.2f},{P[:,2].max():.2f}]')
print(f'  |r|>50m 占比 {(r>50).mean()*100:.1f}%')
PY
else
  echo "[verify] ❌ 没有生成 $OUT"
  echo "--- FAST-LIO 日志尾部 ---"; tail -20 /tmp/v_map.log
  echo "--- 存图日志 ---"; cat /tmp/v_save.log
fi
