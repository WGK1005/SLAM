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
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2


class TimestampNormalizer(Node):
    def __init__(self, in_topic, out_topic, imu_in, imu_out, fix_imu, report_period, mode='auto'):
        super().__init__('timestamp_normalizer')
        self.in_topic, self.out_topic = in_topic, out_topic
        self.report_period = report_period
        self.mode = mode

        deep_sensor_qos = QoSProfile(depth=500, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub = self.create_publisher(PointCloud2, out_topic, qos_profile_sensor_data)
        self.sub = self.create_subscription(PointCloud2, in_topic, self.on_cloud, deep_sensor_qos)

        self.n = 0
        self.jumps = 0             # 原始时间戳倒退次数
        self.reanchors = 0         # 时钟偏置重锚次数
        self.worst_jump = 0.0      # 最大倒退量（秒，负值）
        self.last_in = None
        self.span_sum = 0.0
        self.last_out = None       # 上一帧输出时间戳（用于强制单调）
        self.forced = 0            # 被强制 +1ms 的帧数
        self.last_report = time.time()
        self.t_start = time.time()

        self.shared_offset = None
        self.lidar_offset = None
        self.last_imu_in = None
        self.last_imu_out = None
        self.last_imu_mono = None
        self.last_lidar_mono = None

        if fix_imu:
            # 注意：FAST-LIO 订阅 IMU 用的是默认 QoS（RELIABLE），这里必须用可靠发布，否则它收不到！
            # 订阅侧使用 depth=500 避免在 4 核高负载下被 QoS depth=5 丢弃 200Hz IMU 数据包
            self.imu_pub = self.create_publisher(Imu, imu_out, 500)
            self.imu_sub = self.create_subscription(Imu, imu_in, self.on_imu, deep_sensor_qos)
            self.imu_n = 0
            self.get_logger().info(
                f'IMU 与雷达同源重打戳(mode={self.mode}, queue=500): {imu_in} -> {imu_out}')
        else:
            self.imu_pub = self.imu_sub = None

        self.get_logger().info(
            f'时间戳规范化启动 (mode={self.mode}): {in_topic} -> {out_topic}')

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
        mono_now = time.monotonic()

        # —— 诊断：统计原始时间戳的倒退 ——
        in_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        jumped_backward = False
        jumped_forward = False
        if self.last_in is not None:
            dt_in = in_stamp - self.last_in
            if dt_in <= 0.0:
                jumped_backward = True
                self.jumps += 1
                if dt_in < self.worst_jump:
                    self.worst_jump = dt_in
            elif dt_in > 2.0:
                jumped_forward = True
        self.last_in = in_stamp

        # —— 核心换算 ——
        span = self._time_span(msg)
        if span is None or span <= 0.0 or span > 1.0:
            span = 0.1          # 兜底：读不到 time 字段时按 10Hz 一圈估算
        self.span_sum += span

        if self.mode == 'wall':
            stamp = now - span
        else:
            if self.lidar_offset is None:
                if (self.shared_offset is not None and self.last_imu_in is not None and
                        abs((self.last_imu_in - span) - in_stamp) <= 0.50):
                    # IMU 与雷达处于同源主机时基（例如录制数据包），共享完全一致的固定偏置，消除 CPU 调度抖动
                    self.lidar_offset = self.shared_offset
                else:
                    self.lidar_offset = (now - span) - in_stamp
                self.reanchors += 1

            wall_pause = (
                self.last_lidar_mono is not None and (mono_now - self.last_lidar_mono) > 2.5 and
                not jumped_backward
            )
            cand = in_stamp + self.lidar_offset
            imu_skew = False
            if (self.last_imu_out is not None and self.last_imu_mono is not None and
                    (mono_now - self.last_imu_mono) < 0.25):
                # 若硬件 VLP-16 时钟相对主机 IMU 发生明显偏移（>0.30s），重新锚定至无跳变主机时基
                if abs((self.last_imu_out - span) - cand) > 0.30:
                    imu_skew = True

            if jumped_backward or jumped_forward or wall_pause or imu_skew:
                if (self.last_imu_out is not None and self.last_imu_mono is not None and
                        (mono_now - self.last_imu_mono) < 0.25 and not wall_pause):
                    target = self.last_imu_out - span
                else:
                    target = now - span
                self.lidar_offset = target - in_stamp
                stamp = target
                self.reanchors += 1
            else:
                stamp = cand

        if self.last_out is not None and stamp <= self.last_out:
            stamp = self.last_out + 0.001     # 强制单调：避免 FAST-LIO 清空缓冲区
            self.forced += 1
        self.last_out = stamp
        self.last_lidar_mono = mono_now
        sec = int(stamp)
        msg.header.stamp.sec = sec
        msg.header.stamp.nanosec = int((stamp - sec) * 1e9)
        self.pub.publish(msg)
        self.n += 1

        if time.time() - self.last_report > self.report_period:
            dt = max(1e-3, time.time() - self.t_start)
            self.get_logger().info(
                f'已规范化 {self.n} 帧（{self.n/dt:.1f} Hz, mode={self.mode}）｜原始戳倒退 {self.jumps} 次'
                f'（最大 {self.worst_jump*1000:.0f} ms）｜重锚 {self.reanchors} 次｜平均 span {self.span_sum/self.n:.4f}s')
            self.last_report = time.time()

    # ------------------------------- IMU 回调 -------------------------------
    def on_imu(self, msg: Imu):
        """保持传感器精确采样间隔（或 wall 模式下壁钟打戳），避免 Python 调度抖动破坏 IMU/LiDAR 同步"""
        now = self.get_clock().now().nanoseconds * 1e-9
        mono_now = time.monotonic()
        in_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self.mode == 'wall':
            stamp = now
        else:
            if self.shared_offset is None:
                self.shared_offset = now - in_stamp
            elif self.last_imu_in is not None:
                dt_in = in_stamp - self.last_imu_in
                wall_pause = (
                    self.last_imu_mono is not None and (mono_now - self.last_imu_mono) > 2.5
                )
                if dt_in > 1.0 or wall_pause:
                    self.shared_offset = now - in_stamp
            if self.last_imu_in is not None and in_stamp <= self.last_imu_in:
                stamp = self.last_imu_out + 0.0005
            else:
                stamp = in_stamp + self.shared_offset

        if self.last_imu_out is not None and stamp <= self.last_imu_out:
            stamp = self.last_imu_out + 0.0005

        self.last_imu_in = in_stamp
        self.last_imu_out = stamp
        self.last_imu_mono = mono_now
        sec = int(stamp)
        msg.header.stamp.sec = sec
        msg.header.stamp.nanosec = int((stamp - sec) * 1e9)
        self.imu_pub.publish(msg)
        self.imu_n += 1


def main():
    ap = argparse.ArgumentParser(description='点云/IMU 时间戳规范化节点')
    ap.add_argument('--in', dest='in_topic', default='/velodyne_points', help='输入点云话题')
    ap.add_argument('--out', dest='out_topic', default='/velodyne_points_norm', help='输出点云话题')
    ap.add_argument('--imu-in', default='/imu/data')
    ap.add_argument('--imu-out', default='/imu/data_norm')
    ap.add_argument('--mode', choices=['auto', 'wall'], default='auto',
                    help='auto=首帧时钟偏置锁定+硬件跳变重锚(保留原始采样间隔); wall=逐帧主机壁钟重打戳')
    ap.add_argument('--no-fix-imu', dest='fix_imu', action='store_false',
                    help='不给 IMU 重打戳（默认会重打；点云与 IMU 必须同基，一般不要关闭）')
    ap.set_defaults(fix_imu=True)
    ap.add_argument('--report-period', type=float, default=10.0, help='诊断打印间隔（秒）')
    args, _ = ap.parse_known_args()

    rclpy.init()
    node = TimestampNormalizer(args.in_topic, args.out_topic, args.imu_in, args.imu_out,
                               args.fix_imu, args.report_period, mode=args.mode)
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
