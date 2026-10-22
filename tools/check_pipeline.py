#!/usr/bin/env python3
"""PC 跑通验收单：一次跑完所有关键指标，输出 PASS / WARN / FAIL

检查内容：
  1) /velodyne_points —— 字段是否含 time；time 是否秒制正偏移；每帧点数；帧率
  2) /imu/data        —— 帧率（FAST-LIO 建议 ≥200Hz）；加速度单位是否是 m/s^2（静止时 |a| ≈ 9.8）
  3) /Odometry        —— 帧率是否跟得上雷达（实时性）；静止时位置漂移包络
用法：
  python3 tools/check_pipeline.py                 # 默认测 20 秒
  python3 tools/check_pipeline.py --duration 30
  python3 tools/check_pipeline.py --lidar-topic /velodyne_points --imu-topic /imu/data
"""
import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2
from nav_msgs.msg import Odometry
from sensor_msgs_py import point_cloud2 as pc2

RESULT = []          # (等级, 项目, 说明)


def log(level, item, msg):
    RESULT.append((level, item, msg))
    icon = {'PASS': '[ OK ]', 'WARN': '[WARN]', 'FAIL': '[FAIL]', 'INFO': '[info]'}[level]
    print(f'{icon} {item:<22} {msg}')


class PipelineChecker(Node):
    def __init__(self, args):
        super().__init__('pipeline_checker')
        self.args = args
        self.lidar_t = []       # 收到点云的时刻
        self.imu_t = []
        self.odom_t = []
        self.lidar_pts = 0
        self.lidar_fields = None
        self.time_min = None
        self.time_max = None
        self.geom = None            # 点云几何统计（点数/方位角覆盖/线数/距离/空间范围）
        self.acc_mag = []
        self.acc_z = []
        self.pos = []
        self.vel = []
        self.create_subscription(PointCloud2, args.lidar_topic, self.lidar_cb, qos_profile_sensor_data)
        self.create_subscription(Imu, args.imu_topic, self.imu_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, args.odom_topic, self.odom_cb, qos_profile_sensor_data)

    def lidar_cb(self, msg):
        self.lidar_t.append(time.time())
        if self.lidar_fields is None:
            self.lidar_fields = [f.name for f in msg.fields]
            self.lidar_pts = msg.width * msg.height
            need = tuple(n for n in ('x', 'y', 'z', 'ring', 'time') if n in self.lidar_fields)
            if 'time' in self.lidar_fields and len(need) >= 2:
                pts = list(pc2.read_points(msg, field_names=need, skip_nans=True))
                if pts:
                    idx = {n: i for i, n in enumerate(need)}
                    ts = [p[idx['time']] for p in pts]
                    self.time_min, self.time_max = min(ts), max(ts)
                    self.geom = self._geom(pts, idx)

    @staticmethod
    def _geom(pts, idx):
        """统计点云几何，用于判断"点云本身是否健康"（相机会导致 FAST-LIO 匹配失败）"""
        xs = [p[idx['x']] for p in pts]
        ys = [p[idx['y']] for p in pts]
        zs = [p[idx['z']] for p in pts]
        rr = sorted(math.sqrt(x * x + y * y + z * z) for x, y, z in zip(xs, ys, zs))
        rings = sorted({int(p[idx['ring']]) for p in pts}) if 'ring' in idx else []
        bt = sorted(pts, key=lambda p: p[idx['time']])
        az = [math.degrees(math.atan2(p[idx['y']], p[idx['x']])) % 360 for p in bt]
        un = [az[0]]
        for a in az[1:]:
            if a < un[-1] - 180:
                a += 360
            elif a > un[-1] + 180:
                a -= 360
            un.append(a)
        return dict(n=len(pts), x=(xs and min(xs), max(xs)), y=(min(ys), max(ys)),
                    z=(min(zs), max(zs)), med=rr[len(rr) // 2], mx=rr[-1],
                    rings=len(rings), azcov=abs(un[-1] - un[0]))

    def imu_cb(self, msg):
        self.imu_t.append(time.time())
        a = msg.linear_acceleration
        self.acc_mag.append(math.sqrt(a.x * a.x + a.y * a.y + a.z * a.z))
        self.acc_z.append(a.z)

    def odom_cb(self, msg):
        self.odom_t.append(time.time())
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        self.pos.append(math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z))
        self.vel.append(math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z))


def rate(ts):
    if len(ts) < 3:
        return 0.0
    span = ts[-1] - ts[0]
    return (len(ts) - 1) / span if span > 0 else 0.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--duration', type=float, default=20.0, help='采样时长（秒）')
    ap.add_argument('--lidar-topic', default='/velodyne_points')
    ap.add_argument('--imu-topic', default='/imu/data')
    ap.add_argument('--odom-topic', default='/Odometry')
    args = ap.parse_args()

    rclpy.init()
    node = PipelineChecker(args)
    print(f'采样 {args.duration:.0f} 秒 …\n')
    t0 = time.time()
    while rclpy.ok() and time.time() - t0 < args.duration:
        rclpy.spin_once(node, timeout_sec=0.1)

    lidar_rate = rate(node.lidar_t)
    imu_rate = rate(node.imu_t)
    odom_rate = rate(node.odom_t)

    # ---------------- 1) 点云 ----------------
    if not node.lidar_t:
        log('FAIL', '点云', f'{args.lidar_topic} 没收到任何消息')
    else:
        log('INFO', '点云帧率', f'{lidar_rate:.2f} Hz  每帧 {node.lidar_pts} 点')
        log('INFO', '点云字段', str(node.lidar_fields))
        if node.geom:
            g = node.geom
            log('INFO', '点云几何', f'点数 {g["n"]}  距离中位 {g["med"]:.2f} m / 最大 {g["mx"]:.2f} m  '
                                   f'线数 {g["rings"]}  方位角覆盖 {g["azcov"]:.0f}°')
            log('INFO', '点云范围', f'x∈[{g["x"][0]:.1f},{g["x"][1]:.1f}]  y∈[{g["y"][0]:.1f},{g["y"][1]:.1f}]  '
                                   f'z∈[{g["z"][0]:.1f},{g["z"][1]:.1f}]  (单位 m)')
            if g['rings'] < 10:
                log('FAIL', '点云线数', f'只有 {g["rings"]} 线（VLP-16 应为 16）→ 检查 returns 是否 Strongest / scan_line 是否 16')
            elif g['rings'] < 16:
                log('INFO', '点云线数', f'{g["rings"]} 线（缺 {16 - g["rings"]} 条）：若缺的是最上面 ring 0/1/2，'
                                       '说明朝上打空（天花板高/室外）属正常；若缺的是中间线才需要查驱动')
            if g['azcov'] < 300:
                log('FAIL', '方位角覆盖', f'只有 {g["azcov"]:.0f}°（一整圈应≈360°）'
                                          ' → 雷达视野被遮挡（大部分方向没有回波），FAST-LIO 必然匹配失败')
            if g['med'] < 0.8:
                log('WARN', '点云距离', f'中位距离仅 {g["med"]:.2f} m → 雷达贴着物体/被挡在里面，'
                                        '建议移到开阔处（多数方向 2~5m）再测')
        if node.lidar_fields and 'time' in node.lidar_fields:
            log('PASS', 'time 字段', f'存在；range [{node.time_min:.6f}, {node.time_max:.6f}] s')
            if node.time_min is not None and node.time_min < -1e-9:
                log('FAIL', 'time 符号', '存在负值 → 驱动 timestamp_first_packet 必须设为 true')
            elif node.time_max is not None and node.time_max > 1.5:
                log('FAIL', 'time 单位', f'最大值 {node.time_max:.3f} 远超一个扫描周期 → 单位不是秒')
            else:
                log('PASS', 'time 语义', '秒制正偏移，符合 FAST-LIO 契约（timestamp_unit: 0）')
        else:
            log('FAIL', 'time 字段', '缺失 → FAST-LIO 只能按方位角推算时间')

    # ---------------- 2) IMU ----------------
    if not node.imu_t:
        log('FAIL', 'IMU', f'{args.imu_topic} 没收到任何消息')
    else:
        mag = sum(node.acc_mag) / len(node.acc_mag)
        z = sum(node.acc_z) / len(node.acc_z)
        log('INFO', 'IMU 帧率', f'{imu_rate:.1f} Hz ; |a| 均值 {mag:.2f} m/s^2 ; a.z 均值 {z:.2f}')
        if imu_rate < 50:
            log('FAIL', 'IMU 帧率', f'{imu_rate:.0f} Hz 太低，FAST-LIO 需要远高于雷达帧率（建议 ≥200Hz）')
        elif imu_rate < 150:
            log('WARN', 'IMU 帧率', f'{imu_rate:.0f} Hz 偏低，建议在 CHCenter 里提高输出率/波特率（目标 ≥200Hz）')
        else:
            log('PASS', 'IMU 帧率', f'{imu_rate:.0f} Hz 足够')
        if 0.5 < mag < 2.0:
            log('FAIL', 'IMU 加速度单位', f'|a|≈{mag:.2f}，看起来是 g 制；FAST-LIO 需要 m/s^2')
        elif 8.0 < mag < 11.5:
            log('PASS', 'IMU 加速度单位', f'|a|≈{mag:.2f} m/s^2（静止时接近重力，单位正确）')
        else:
            log('WARN', 'IMU 加速度单位', f'|a|≈{mag:.2f} 不在预期范围，检查静止状态或单位')

    # ---------------- 3) 里程计 / 实时性 ----------------
    if not node.odom_t:
        log('FAIL', '里程计', f'{args.odom_topic} 没收到任何消息（FAST-LIO 是否在跑/是否初始化完成）')
    else:
        log('INFO', '里程计帧率', f'{odom_rate:.2f} Hz ; 位置包络 {max(node.pos):.3f} m')
        if lidar_rate > 0:
            ratio = odom_rate / lidar_rate
            if ratio >= 0.9:
                log('PASS', '实时性', f'/Odometry {odom_rate:.1f}Hz ≈ 雷达 {lidar_rate:.1f}Hz，跟得上')
            else:
                log('FAIL', '实时性', f'/Odometry {odom_rate:.1f}Hz 只有雷达 {lidar_rate:.1f}Hz 的 {ratio*100:.0f}%'
                                      ' → 提高 point_filter_num 或降低雷达转速')
        if max(node.pos) < 0.3:
            log('PASS', '静止漂移', f'位置包络 {max(node.pos):.3f} m（传感器静止时应很小）')
        else:
            log('WARN', '静止漂移', f'位置包络 {max(node.pos):.3f} m；若传感器确实静止，检查外参/时间偏移')

    fails = sum(1 for r in RESULT if r[0] == 'FAIL')
    warns = sum(1 for r in RESULT if r[0] == 'WARN')
    print()
    if fails:
        print(f'===== 验收不通过：{fails} 项 FAIL，{warns} 项 WARN =====')
    elif warns:
        print(f'===== 基本通过：{warns} 项 WARN（可接受但建议处理）=====')
    else:
        print('===== 全部通过 ✅ 这条链路在 PC 上已经跑通 =====')

    node.destroy_node()
    rclpy.shutdown()
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
