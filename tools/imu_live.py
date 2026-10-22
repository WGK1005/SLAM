#!/usr/bin/env python3
"""实时打印 IMU 的加速度/角速度，用于判定 LiDAR-IMU 外参的旋转（extrinsic_R）

用法：
  python3 tools/imu_live.py          # 打印 20 秒
  python3 tools/imu_live.py 60       # 打印 60 秒

判定方法（一边看这个输出，一边手动摆弄传感器）：
  1) 静止：az 应 ≈ +9.8（z 轴朝上）
  2) 把传感器"低头"（让 LiDAR 的 x 方向朝下倾斜约 30°）：
       哪个分量明显变负，那个分量就是 IMU 的"前方"轴
       - x 变负  → IMU 的 x 朝前（与 LiDAR 一致）
       - y 变负  → IMU 的 y 朝前（需要绕 z 转 90°）
       - z 变正  → IMU 的 z 朝前（安装方向完全不同）
  3) 俯视把传感器逆时针旋转：
       wz 为正 → 右手系、z 朝上（常见）
       wz 为负 → y 轴反向
  4) 常见结论：
       LiDAR(前-左-上) ↔ IMU(前-右-上)  →  extrinsic_R = [1,0,0, 0,-1,0, 0,0,-1]
       两者完全对齐(前-左-上)           →  extrinsic_R = [1,0,0, 0,1,0, 0,0,1]
"""
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


class Live(Node):
    def __init__(self):
        super().__init__('imu_live')
        self.v = None
        self.create_subscription(Imu, '/imu/data', self.cb, qos_profile_sensor_data)

    def cb(self, m):
        self.v = (m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z,
                  m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z,
                  m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w)


def main():
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
    rclpy.init()
    n = Live()
    print('时间     ax      ay      az   |    wx      wy      wz   |  |a|')
    print('-' * 68)
    t0 = None
    k = 0
    while rclpy.ok():
        rclpy.spin_once(n, timeout_sec=0.02)
        if n.v is None:
            if k == 0:
                print('等待 /imu/data …')
            k += 1
            if k > 500:
                break
            continue
        if t0 is None:
            t0 = __import__('time').time()
        if k % 15 == 0:
            a = n.v
            mag = math.sqrt(a[0] ** 2 + a[1] ** 2 + a[2] ** 2)
            import time
            print(f'{time.time()-t0:5.1f}s {a[0]:+7.2f} {a[1]:+7.2f} {a[2]:+7.2f} | '
                  f'{a[3]:+7.3f} {a[4]:+7.3f} {a[5]:+7.3f} | {mag:6.2f}')
            if time.time() - t0 > dur:
                break
        k += 1
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
