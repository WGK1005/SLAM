#!/usr/bin/env python3
"""无硬件联调：合成 VLP-16 点云 + HI12 风格 IMU 数据，喂给 FAST-LIO

用途：在没有雷达 / IMU 的情况下验证整条数据链路
  1) PointCloud2 字段是否按 FAST-LIO 期望的 x,y,z,intensity,ring,time 给出
  2) time 字段单位是"秒"、且是正偏移（对应 timestamp_unit: 0）
  3) IMU 与雷达时间戳同钟、能触发 FAST-LIO 初始化并输出 /Odometry
  4) 用不同点数/帧率压测 FAST-LIO 的 CPU 占用，估算目标板（如香橙派）能否实时

实现要点：点云在启动时**预生成并缓存**（静态房间场景），每帧只换 header.stamp 重发，
避免 Python 逐点射线求交成为瓶颈（否则测出来的是发布器的性能，不是 FAST-LIO 的）。

用法：
  python3 synthetic_sensor_pub.py                                # 16x225 点 @10Hz
  python3 synthetic_sensor_pub.py --az-steps 912  --scan-hz 20   # 14592 点/帧 @20Hz
  python3 synthetic_sensor_pub.py --az-steps 1824 --scan-hz 20   # 29184 点/帧 @20Hz（对齐真实 VLP-16@1200rpm）
"""
import argparse
import math
import struct
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, PointCloud2, PointField

IMU_HZ = 200.0
ROOM = dict(x=4.0, y=4.0, z_min=-1.0, z_max=3.0)   # 相对传感器：地面 -1m，天花板 +3m
POINT_STEP = 24                                     # x,y,z,intensity(16) + ring(2) + pad(2) + time(4)


def ray_box(origin, direction, room):
    """返回射线与房间内壁的交点距离（无交点返回 None）"""
    ox, oy, oz = origin
    dx, dy, dz = direction
    tmin, tmax = 0.0, 1e9
    for o, d, lo, hi in ((ox, dx, -room['x'], room['x']),
                         (oy, dy, -room['y'], room['y']),
                         (oz, dz, room['z_min'], room['z_max'])):
        if abs(d) < 1e-9:
            if o < lo or o > hi:
                return None
            continue
        t1, t2 = (lo - o) / d, (hi - o) / d
        if t1 > t2:
            t1, t2 = t2, t1
        tmin, tmax = max(tmin, t1), min(tmax, t2)
        if tmin > tmax:
            return None
    return tmin if tmin > 0 else (tmax if tmax > 0 else None)


class SyntheticSensor(Node):
    def __init__(self, rings, az_steps, scan_hz):
        super().__init__('synthetic_velodyne_hi12')
        self.rings = rings
        self.az_steps = az_steps
        self.scan_hz = scan_hz
        self.pc_pub = self.create_publisher(PointCloud2, '/velodyne_points', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 50)
        self.cloud_data, self.cloud_points = self._build_cloud()
        self.create_timer(1.0 / IMU_HZ, self.publish_imu)
        self.create_timer(1.0 / scan_hz, self.publish_scan)
        self.scan_i = 0
        self.get_logger().info(
            f'合成传感器启动：{rings} 线 x {az_steps} 步 = {self.cloud_points} 点/帧 @ {scan_hz} Hz '
            f'（约 {self.cloud_points * scan_hz / 1000:.0f} k 点/秒），点云已预生成缓存')

    def _build_cloud(self):
        """预生成一帧静态房间的点云（只做一次）"""
        period = 1.0 / self.scan_hz          # 一圈的时间跨度（秒）
        pts = []
        for ring in range(self.rings):
            elev = math.radians(-15.0 + ring * 2.0)          # VLP-16: -15°..+15°
            for k in range(self.az_steps):
                az = 2.0 * math.pi * k / self.az_steps
                d = (math.cos(elev) * math.cos(az),
                     math.cos(elev) * math.sin(az),
                     math.sin(elev))
                # 逐点时间：位于 [0, 1/scan_hz) 秒，单位＝秒（关键：正偏移）
                t = period * (k / float(self.az_steps))
                r = ray_box((0.0, 0.0, 0.0), d, ROOM)
                if r is None or r > 130.0:
                    continue
                pts.append((r * d[0], r * d[1], r * d[2], 100.0, ring, t))

        buf = bytearray(POINT_STEP * len(pts))
        for i, (x, y, z, inten, ring, t) in enumerate(pts):
            struct.pack_into('<ffffHxxf', buf, POINT_STEP * i, x, y, z, inten, ring, t)
        return bytes(buf), len(pts)

    # ------------------------------ IMU ------------------------------
    def publish_imu(self):
        m = Imu()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'imu_link'
        # 静止：只受重力（比力指向 +z，FAST-LIO 期望 m/s^2、未去重力）
        m.linear_acceleration.x = 0.0
        m.linear_acceleration.y = 0.0
        m.linear_acceleration.z = 9.80665
        m.angular_velocity.x = 0.0
        m.angular_velocity.y = 0.0
        m.angular_velocity.z = 0.0
        m.orientation.w = 1.0
        m.orientation_covariance[0] = -1.0
        m.angular_velocity_covariance[0] = 1e-4
        m.linear_acceleration_covariance[0] = 1e-4
        self.imu_pub.publish(m)

    # ---------------------------- LiDAR ------------------------------
    def publish_scan(self):
        self.scan_i += 1
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'velodyne'
        msg.height = 1
        msg.width = self.cloud_points
        msg.is_bigendian = False
        msg.point_step = POINT_STEP
        msg.row_step = POINT_STEP * self.cloud_points
        msg.is_dense = False
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='ring', offset=16, datatype=PointField.UINT16, count=1),
            PointField(name='time', offset=20, datatype=PointField.FLOAT32, count=1),
        ]
        msg.data = self.cloud_data
        self.pc_pub.publish(msg)
        if self.scan_i % 40 == 1:
            self.get_logger().info(
                f'第 {self.scan_i} 帧：{self.cloud_points} 点（字段 x,y,z,intensity,ring,time，秒制正偏移）')


def main():
    ap = argparse.ArgumentParser(description='合成 VLP-16 + IMU 数据发布器')
    ap.add_argument('--rings', type=int, default=16, help='线数（默认 16）')
    ap.add_argument('--az-steps', type=int, default=225, help='每圈每线采样数（默认 225 → 3600 点/帧）')
    ap.add_argument('--scan-hz', type=float, default=10.0, help='扫描频率（默认 10Hz；VLP-16 1200rpm = 20Hz）')
    args = ap.parse_args()

    rclpy.init()
    node = SyntheticSensor(args.rings, args.az_steps, args.scan_hz)
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
