#!/usr/bin/env python3
"""时间戳规范化节点：用主机时钟给点云重打戳，消除雷达内部时钟跳变

【为什么需要】
VLP-16 没接 GPS/PPS，官方 ROS 驱动的 header.stamp 是用
    (主机时间 + 雷达内部自由运行的微秒计数)/2
重建出来的。雷达内部时钟一旦跳变/漂移，时间戳就**倒退** —— 实测 100 秒内倒退 6 次，
最大 -1748 ms。后果：
  1) FAST-LIO 打 "lidar loop back, clear buffer"（丢帧）
  2) 雷达与 IMU 的时间关系不再是固定偏移，周期性错乱 → 去畸变/时间对齐变差 → 地图精度受损

【原理（无状态，三步）】
  点云里的逐点 `time` 字段（相对扫描起始，秒）本身是准的（已验证）
  1) span = 该帧所有点 time 的最大值  （≈ 一圈时长 0.1s）
  2) 新 header.stamp = 收到该帧的主机时刻 − span
     —— FAST-LIO 要求 header.stamp = 扫描起始时刻，而点云是转完一圈才发出来的
  3) `time` 字段原样保留 → 去畸变照旧准确

由于 hipnuc IMU 驱动本来就用主机时钟打戳，重打戳后**雷达与 IMU 天然落在同一时基**，
只剩固定的 ~1~2ms 延迟（相比 1.4 秒的跳变小了三个数量级）。

【用法】
  实时（跟随 bringup launch 自动启动，见 velodyne_hi12_bringup.launch.py）
  手动：
    python3 tools/timestamp_normalizer.py                       # /velodyne_points -> /velodyne_points_norm
    python3 tools/timestamp_normalizer.py --in /a --out /b
  回放 bag 时（必须和 FAST-LIO 用同一个时间源）：
    python3 tools/timestamp_normalizer.py --ros-args -p use_sim_time:=true

【注意】
  启用后要把 config/velodyne.yaml 的 lid_topic 改成 /velodyne_points_norm（否则等于没接）。
"""
import argparse
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2


class TimestampNormalizer(Node):
    def __init__(self, in_topic, out_topic, imu_in, imu_out, fix_imu, report_period):
        super().__init__('timestamp_normalizer')
        self.in_topic, self.out_topic = in_topic, out_topic
        self.report_period = report_period

        self.pub = self.create_publisher(PointCloud2, out_topic, qos_profile_sensor_data)
        self.sub = self.create_subscription(PointCloud2, in_topic, self.on_cloud, qos_profile_sensor_data)

        self.n = 0
        self.jumps = 0             # 原始时间戳倒退次数
        self.worst_jump = 0.0      # 最大倒退量（秒，负值）
        self.last_in = None
        self.span_sum = 0.0
        self.last_out = None       # 上一帧输出时间戳（用于强制单调）
        self.forced = 0            # 被强制 +1ms 的帧数
        self.last_report = time.time()
        self.t_start = time.time()

        if fix_imu:
            # 注意：FAST-LIO 订阅 IMU 用的是默认 QoS（RELIABLE），这里必须用可靠发布，否则它收不到！
            self.imu_pub = self.create_publisher(Imu, imu_out, 100)
            self.imu_sub = self.create_subscription(Imu, imu_in, self.on_imu, qos_profile_sensor_data)
            self.imu_n = 0
            self.get_logger().info(f'IMU 也一并重打戳（墙钟）: {imu_in} -> {imu_out}（RELIABLE 发布）')
        else:
            self.imu_pub = self.imu_sub = None

        self.get_logger().info(
            f'时间戳规范化启动: {in_topic} -> {out_topic}'
            f'（stamp = 到达时刻 − 帧内最大 time）')

    # ---------------- 工具：读一帧里 time 字段的最大值（向量化，17k 点约 1ms） ----------------
    @staticmethod
    def _time_span(msg: PointCloud2):
        off = next((f.offset for f in msg.fields if f.name == 'time'), None)
        n = msg.width * msg.height
        if off is None or n == 0 or msg.point_step <= 0:
            return None
        need = n * msg.point_step
        if len(msg.data) < need:
            return None
        a = np.frombuffer(bytes(msg.data[:need]), dtype=np.uint8).reshape(n, msg.point_step)
        t = a[:, off:off + 4].copy().view(np.float32).reshape(-1)
        t = t[np.isfinite(t)]
        if t.size == 0:
            return None
        return float(t.max())

    # ------------------------------- 点云回调 -------------------------------
    def on_cloud(self, msg: PointCloud2):
        now = self.get_clock().now().nanoseconds * 1e-9

        # —— 诊断：统计原始时间戳的倒退（只为报告，不参与换算）——
        in_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_in is not None and in_stamp < self.last_in:
            self.jumps += 1
            d = in_stamp - self.last_in
            if d < self.worst_jump:
                self.worst_jump = d
        self.last_in = in_stamp

        # —— 核心换算 ——
        span = self._time_span(msg)
        if span is None or span <= 0.0 or span > 1.0:
            span = 0.1          # 兜底：读不到 time 字段时按 10Hz 一圈估算
        self.span_sum += span

        stamp = now - span
        if self.last_out is not None and stamp <= self.last_out:
            stamp = self.last_out + 0.001     # 强制单调：两帧到达太近时可能出现微小倒退，会让 FAST-LIO 清空缓冲区
            self.forced += 1
        self.last_out = stamp
        sec = int(stamp)
        msg.header.stamp.sec = sec
        msg.header.stamp.nanosec = int((stamp - sec) * 1e9)
        self.pub.publish(msg)
        self.n += 1

        if time.time() - self.last_report > self.report_period:
            dt = time.time() - self.t_start
            self.get_logger().info(
                f'已规范化 {self.n} 帧（{self.n/dt:.1f} Hz）｜原始戳倒退 {self.jumps} 次'
                f'（最大 {self.worst_jump*1000:.0f} ms）｜平均 span {self.span_sum/self.n:.4f}s'
                + ('｜已修正原始戳为单调' if self.jumps else ''))
            self.last_report = time.time()

    # ------------------------------- IMU 回调（可选） -------------------------------
    def on_imu(self, msg: Imu):
        """IMU 也用墙钟重打戳 —— 保证与点云同一时基（且顺带修掉驱动里 8ms 级的偶发倒序）"""
        now = self.get_clock().now().nanoseconds * 1e-9
        sec = int(now)
        msg.header.stamp.sec = sec
        msg.header.stamp.nanosec = int((now - sec) * 1e9)
        self.imu_pub.publish(msg)
        self.imu_n += 1


def main():
    ap = argparse.ArgumentParser(description='点云时间戳规范化（主机时钟重打戳）')
    ap.add_argument('--in', dest='in_topic', default='/velodyne_points', help='输入点云话题')
    ap.add_argument('--out', dest='out_topic', default='/velodyne_points_norm', help='输出点云话题')
    ap.add_argument('--imu-in', default='/imu/data')
    ap.add_argument('--imu-out', default='/imu/data_norm')
    ap.add_argument('--no-fix-imu', dest='fix_imu', action='store_false',
                    help='不给 IMU 重打戳（默认会重打；点云与 IMU 必须同基，一般不要关闭）')
    ap.set_defaults(fix_imu=True)
    ap.add_argument('--report-period', type=float, default=10.0, help='诊断打印间隔（秒）')
    args, _ = ap.parse_known_args()

    rclpy.init()
    node = TimestampNormalizer(args.in_topic, args.out_topic, args.imu_in, args.imu_out,
                               args.fix_imu, args.report_period)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
