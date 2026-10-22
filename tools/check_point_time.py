#!/usr/bin/env python3
"""检查点云字段与逐点时间是否符合 FAST-LIO 的输入契约

用法：
  python3 tools/check_point_time.py              # 默认 /velodyne_points
  python3 tools/check_point_time.py /points_raw  # 指定话题

判定依据（FAST-LIO ros2 分支 velodyne_handler）：
  * 字段需要 time：Humble 版 velodyne 驱动应给出 x,y,z,intensity,ring,time
  * time 单位是"秒"，含义是相对 scan 起始的偏移 → 10Hz 雷达应落在 [0, 0.1]
  * time 出现负值  → 雷达驱动 timestamp_first_packet 没设 true（scan 时间戳取了最后一个包）
  * time 最大值远大于 0.1 → 单位不是秒 → 检查 fast_lio 的 timestamp_unit（Velodyne 应为 0）
"""
import sys
import time as pytime

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2


class Checker(Node):
    def __init__(self, topic):
        super().__init__('check_point_time')
        self.done = False
        self.create_subscription(PointCloud2, topic, self.cb, qos_profile_sensor_data)

    def cb(self, msg):
        if self.done:
            return
        self.done = True

        names = [f.name for f in msg.fields]
        print(f'frame_id : {msg.header.frame_id}')
        print(f'stamp    : {msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}')
        print(f'点数     : {msg.width} x {msg.height}   point_step={msg.point_step}')
        print(f'字段     : {names}')

        if 'time' not in names:
            print('\n[FAIL] 没有 time 字段。')
            print('       现象：FAST-LIO 只能按方位角推算点时间（去畸变精度差），且会走 given_offset_time=false 分支。')
            print('       排查：velodyne_pointcloud 是否 Humble 官方版；是否被其他驱动/自定义 topic 覆盖。')
            return

        pts = list(pc2.read_points(msg, field_names=('time', 'ring'), skip_nans=True))
        if not pts:
            print('\n[FAIL] 点云为空（skip_nans 后 0 点）')
            return

        times = [p[0] for p in pts]
        rings = sorted({int(p[1]) for p in pts})
        tmin, tmax = min(times), max(times)
        print(f'\ntime     : min={tmin:.6f} max={tmax:.6f} 首点={times[0]:.6f} 末点={times[-1]:.6f} 跨度={tmax - tmin:.6f}s')
        print(f'ring     : {len(rings)} 条线，范围 {rings[0]}~{rings[-1]}')

        ok = True
        if tmin < -1e-9:
            ok = False
            print('\n[FAIL] time 出现负值 → 雷达驱动的 timestamp_first_packet 必须设为 true。')
            print('       原因：velodyne_driver 默认把 scan->header.stamp 设为【最后一个包】的时间，')
            print('       而 velodyne_pointcloud 用该 stamp 当 scan 起始去算 time，于是绝大多数点变成负偏移。')
            print('       修法：config/velodyne_driver.yaml 里 timestamp_first_packet: true 后重启驱动。')
        if tmax <= 0.0:
            ok = False
            print('\n[FAIL] time 全为 0 或负 → 驱动没有输出有效逐点时间。')
        if tmax > 1.0:
            ok = False
            print(f'\n[FAIL] time 最大 {tmax:.6f} 远超一个扫描周期 → 单位很可能不是秒。')
            print('       修法：fast_lio 的 config/velodyne.yaml 里 timestamp_unit 应为 0 (second)。')

        if ok:
            print(f'\n[OK] 字段与逐点时间符合 FAST-LIO 契约：秒制正偏移，跨度 {tmax - tmin:.4f}s（10Hz 一圈≈0.1s）')
            print('     对应 fast_lio 配置：timestamp_unit: 0、scan_line: 16、scan_rate: 10')


def main():
    topic = sys.argv[1] if len(sys.argv) > 1 else '/velodyne_points'
    rclpy.init()
    node = Checker(topic)
    print(f'等待 {topic} 的消息（最多 15 秒）...')
    t0 = pytime.time()
    while rclpy.ok() and not node.done and pytime.time() - t0 < 15.0:
        rclpy.spin_once(node, timeout_sec=0.2)
    if not node.done:
        print(f'[FAIL] 15 秒内没收到 {topic} 的任何消息：')
        print('       检查 ros2 topic list / ros2 topic hz，驱动是否在跑、话题名是否一致。')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
