#!/usr/bin/env bash
# 压测 FAST-LIO：两档点数/帧率，测 CPU 占用 + /Odometry 实时性
# 用来估算目标板（香橙派 5 Pro / RK3588S）能不能实时
SRC=/home/wgk/fast_lio_ws
CLEAN=(env -u PKG_CONFIG_PATH -u CMAKE_PREFIX_PATH -u COLCON_PREFIX_PATH -u AMENT_PREFIX_PATH \
       -u LD_LIBRARY_PATH -u PYTHONPATH -u DDSGEN -u AMENT_CURRENT_PREFIX bash --noprofile --norc -c)
HZ=$(getconf CLK_TCK); N=$(nproc)

ros_cmd() { "${CLEAN[@]}" "source /opt/ros/humble/setup.bash && source $SRC/install/setup.bash && $1"; }
stop_all() { pkill -f fastlio_mapping 2>/dev/null; pkill -f synthetic_sensor_pub 2>/dev/null; sleep 3; }

bench() {   # $1=az_steps  $2=scan_hz  $3=label
  stop_all
  ros_cmd "exec ros2 run fast_lio fastlio_mapping --ros-args --params-file $SRC/install/fast_lio/share/fast_lio/config/velodyne.yaml" > /tmp/map.log 2>&1 &
  sleep 6
  MPID=$(pgrep -x fastlio_mapping | head -1 | tr -d ' \n')
  if [ -z "$MPID" ]; then echo "!! mapping 启动失败"; tail -5 /tmp/map.log; return; fi
  ros_cmd "exec python3 $SRC/tools/synthetic_sensor_pub.py --az-steps $1 --scan-hz $2" > /tmp/pub_$1.log 2>&1 &
  sleep 15

  t1=$(awk '{print $14+$15}' /proc/$MPID/stat)
  sleep 20
  t2=$(awk '{print $14+$15}' /proc/$MPID/stat)

  echo "=== $3 ==="
  awk -v a=$t1 -v b=$t2 -v hz=$HZ -v n=$N \
    'BEGIN{cpu=(b-a)*100/(hz*20); printf "  FAST-LIO CPU  : %.1f%% 单核  =  整机 %.1f%% (of %d 核)\n", cpu, cpu/n, n}'
  awk '{printf "  常驻内存 RSS  : %.0f MB\n", $1/1024}' < <(ps -o rss= -p $MPID)
  printf "  /Odometry     : "
  ros_cmd "timeout 12 ros2 topic hz /Odometry" 2>/dev/null | grep average | tail -1
  echo
}

echo "本机核数: $N"
bench 912  20 "负载 A: 14592 点/帧 @20Hz → 29.2 万点/秒（滤波后 14.6 万）"
bench 1824 20 "负载 B: 29184 点/帧 @20Hz → 58.4 万点/秒（滤波后 29.2 万）= 真实 VLP-16@1200rpm 量级"
bench 1824 10 "负载 C: 29184 点/帧 @10Hz → 29.2 万点/秒（滤波后 14.6 万）= 把雷达调回 600rpm 的方案"
stop_all
echo "bench done"
