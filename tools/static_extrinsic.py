#!/usr/bin/env python3
"""静态外参标定（桌面静置法）

原理
    传感器组合体静止放在水平桌面上时，两个传感器各自独立给出"竖直向上"的方向：
      · IMU   : 静止时加速度计读的是支持力，f_imu / |f_imu| 就是"上"在 IMU 系里的方向
      · LiDAR : 桌面/地面是水平面，拟合出的平面法向 n_lidar 就是"上"在 LiDAR 系里的方向
    FAST-LIO 需要 extrinsic_R 满足    v_imu = R · v_lidar
    所以正确的外参就是把 n_lidar 转到 f_imu / |f_imu| 的那个最小旋转：
      R = 旋转(n_lidar → up_imu)

    这个测量不经过 FAST-LIO，因此能干净地区分：
      · R 明显偏离单位阵 → 硬件（支架装歪），改 extrinsic_R
      · R ≈ 单位阵    → 外参没问题，地图歪就是软件/配置的问题

注意
    1) 单次静态只能确定 2 个自由度（俯仰+横滚）；绕竖直轴的偏航无法由单一向量确定，
       这里取最小旋转（即偏航沿用现有配置的假设）。
    2) 录包时只起驱动，不要起 FAST-LIO（避免占 CPU 影响时序）。
    3) 录 10~20 秒即可，全程不能碰。

用法
    python3 tools/static_extrinsic.py --bag ~/bags/static_1017_1520
    python3 tools/static_extrinsic.py --bag ... --frames 8 --write     # 直接写回 yaml
    python3 tools/static_extrinsic.py --bag ... --lidar-topic /velodyne_points --imu-topic /imu/data
"""
import argparse
import os
import sys
from pathlib import Path

import numpy as np

WORKSPACE = Path(__file__).resolve().parent.parent
CONFIG = WORKSPACE / 'src' / 'FAST_LIO' / 'config' / 'velodyne.yaml'


# ----------------------------------------------------------------- 读包
def read_bag(bag, lidar_topic, imu_topic, max_frames, max_imu):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Imu, PointCloud2
    from sensor_msgs_py import point_cloud2

    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'),
           rosbag2_py.ConverterOptions('cdr', 'cdr'))

    clouds, accs, gyrs = [], [], []
    while r.has_next() and (len(clouds) < max_frames or len(accs) < max_imu):
        topic, data, stamp = r.read_next()
        if topic == lidar_topic and len(clouds) < max_frames:
            msg = deserialize_message(data, PointCloud2)
            pts = np.array([(p[0], p[1], p[2]) for p in
                            point_cloud2.read_points(msg, field_names=('x', 'y', 'z'),
                                                     skip_nans=True)], dtype=np.float64)
            if len(pts) > 100:
                clouds.append(pts)
        elif topic == imu_topic and len(accs) < max_imu:
            msg = deserialize_message(data, Imu)
            a = msg.linear_acceleration
            w = msg.angular_velocity
            accs.append((a.x, a.y, a.z))
            gyrs.append((w.x, w.y, w.z))
    return clouds, np.array(accs), np.array(gyrs)


# ----------------------------------------------------------------- 平面拟合
def fit_floor_normal(P, cell=2.0, min_pts=20, pct=5.0):
    """每 cell×cell 的 XY 格子里取最低 pct% 的点当"地面"，拟合平面 → 法向"""
    x, y, z = P[:, 0], P[:, 1], P[:, 2]
    ix = np.floor(x / cell).astype(np.int64)
    iy = np.floor(y / cell).astype(np.int64)
    key = ix * 1000000 + iy
    order = np.argsort(key, kind='stable')
    sk = key[order]
    uniq, starts = np.unique(sk, return_index=True)
    ends = np.append(starts[1:], len(sk))
    rows = []
    for i, k in enumerate(uniq):
        idx = order[starts[i]:ends[i]]
        if len(idx) < min_pts:
            continue
        rows.append(((k // 1000000) * cell + cell / 2.0,
                     (k % 1000000) * cell + cell / 2.0,
                     np.percentile(z[idx], pct)))
    A = np.array(rows)
    if len(A) < 10:
        return None, None, 0
    M = np.c_[A[:, 0], A[:, 1], np.ones(len(A))]
    coef, *_ = np.linalg.lstsq(M, A[:, 2], rcond=None)
    # z = a*x + b*y + c  →  平面 a*x + b*y - z + c = 0  →  法向 (a, b, -1)
    n = np.array([coef[0], coef[1], -1.0])
    n = n / np.linalg.norm(n)
    if n[2] < 0:            # 统一指"上"
        n = -n
    return n, coef, len(A)


def rotation_between(a, b):
    """最小旋转 R，使 R @ a = b（a、b 均为单位向量）"""
    a = a / np.linalg.norm(a)
    b = b / np.linalg.norm(b)
    v = np.cross(a, b)
    c = float(np.clip(a @ b, -1.0, 1.0))
    s = np.linalg.norm(v)
    if s < 1e-12:
        if c > 0:
            return np.eye(3), 0.0
        # 反向：绕任一垂直轴转 180°
        axis = np.cross(a, np.array([1.0, 0, 0]))
        if np.linalg.norm(axis) < 1e-9:
            axis = np.cross(a, np.array([0, 1.0, 0]))
        axis = axis / np.linalg.norm(axis)
        return 2 * np.outer(axis, axis) - np.eye(3), 180.0
    K = np.array([[0, -v[2], v[1]],
                  [v[2], 0, -v[0]],
                  [-v[1], v[0], 0]]) / s   # 归一化后的反对称矩阵
    ang = np.arctan2(s, c)
    R = np.eye(3) + np.sin(ang) * K + (1 - np.cos(ang)) * (K @ K)
    return R, np.degrees(ang)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--bag', required=True)
    ap.add_argument('--lidar-topic', default='/velodyne_points')
    ap.add_argument('--imu-topic', default='/imu/data')
    ap.add_argument('--frames', type=int, default=6, help='用多少帧点云（默认 6）')
    ap.add_argument('--max-imu', type=int, default=1200, help='最多读多少条 IMU（默认 1200）')
    ap.add_argument('--cell', type=float, default=2.0)
    ap.add_argument('--pct', type=float, default=5.0)
    ap.add_argument('--write', action='store_true', help='把结果写回 velodyne.yaml')
    args = ap.parse_args()

    bag = os.path.expanduser(args.bag)
    if not os.path.isdir(bag):
        print('❌ 找不到 bag: %s' % bag)
        return 1

    clouds, acc, gyr = read_bag(bag, args.lidar_topic, args.imu_topic,
                                args.frames, args.max_imu)
    if not clouds:
        print('❌ bag 里没有 %s 的点云' % args.lidar_topic)
        return 1
    if len(acc) < 50:
        print('❌ bag 里 %s 的 IMU 数据太少 (%d 条)' % (args.imu_topic, len(acc)))
        return 1

    print('=== 1) 静止性检查（必须真的没动） ===')
    a_norm = np.linalg.norm(acc, axis=1)
    w_norm = np.linalg.norm(gyr, axis=1)
    w_std = gyr.std(axis=0)
    print('  IMU 样本 %d   |a| 均值 %.3f m/s² (应 ≈9.8)' % (len(acc), a_norm.mean()))
    print('  |ω| 均值 %.5f  P95 %.5f  最大 %.5f rad/s' %
          (w_norm.mean(), np.percentile(w_norm, 95), w_norm.max()))
    print('  陀螺各轴 std = %s rad/s' % np.round(w_std, 5))
    if w_norm.mean() > 0.05 or w_std.max() > 0.02:
        print('  ⚠️ 看起来动过了！静止标定要求全程不动，建议重录')
    else:
        print('  ✅ 静止')

    print('\n=== 2) 两个传感器各自测的"上"方向 ===')
    up_imu = acc.mean(axis=0)
    up_imu = up_imu / np.linalg.norm(up_imu)
    tilt_imu = np.degrees(np.arccos(np.clip(up_imu[2], -1, 1)))
    print('  IMU   上方向 (IMU 系)  = [%+.5f %+.5f %+.5f]' % tuple(up_imu))
    print('        相对自身 z 轴偏 %.3f 度' % tilt_imu)

    P = np.vstack(clouds) if len(clouds) == 1 else np.vstack(clouds)
    print('  点云 %d 帧，共 %d 点' % (len(clouds), len(P)))
    n_lidar, coef, ncells = fit_floor_normal(P, cell=args.cell, pct=args.pct)
    if n_lidar is None:
        print('  ❌ 地面平面拟合失败（点云里没有足够大的平面？）')
        return 1
    tilt_lidar = np.degrees(np.arccos(np.clip(n_lidar[2], -1, 1)))
    print('  LiDAR 地面法向(LiDAR 系) = [%+.5f %+.5f %+.5f]   (%d 个格)' %
          (n_lidar[0], n_lidar[1], n_lidar[2], ncells))
    print('        相对自身 z 轴偏 %.3f 度' % tilt_lidar)
    print('        平面 z = %+.4f*x %+.4f*y %+.3f' % (coef[0], coef[1], coef[2]))

    print('\n=== 3) 结论：是硬件还是软件 ===')
    if tilt_imu > 2.0 and tilt_lidar < 2.0:
        print('  → IMU 自身倾斜 %.2f 度，LiDAR 水平 → IMU 安装歪 / 桌面不水平' % tilt_imu)
    elif tilt_lidar > 2.0 and tilt_imu < 2.0:
        print('  → LiDAR 自身倾斜 %.2f 度，IMU 水平 → LiDAR 安装歪（硬件）' % tilt_lidar)
    elif tilt_lidar > 2.0 and tilt_imu > 2.0:
        print('  → 两者都倾斜：桌面可能不水平，或者整机一起歪（整机歪不影响外参）')
    else:
        print('  → 两个传感器各自都指向竖直 → 硬件没问题')

    print('\n=== 4) 应填的 extrinsic_R（把 LiDAR 的"上"转到 IMU 的"上"） ===')
    R, ang = rotation_between(n_lidar, up_imu)
    print('  需要转动 %.3f 度' % ang)
    print('  按行展开 9 个数：')
    print('    %s' % ', '.join('%.6f' % v for v in R.flatten()))
    print('  矩阵：')
    for row in R:
        print('    [%9.6f %9.6f %9.6f]' % tuple(row))
    print('  自检: det = %.9f   max|RRᵀ-I| = %.2e' %
          (np.linalg.det(R), np.abs(R @ R.T - np.eye(3)).max()))
    print('        R @ n_lidar = [%+.6f %+.6f %+.6f]  (应等于 IMU 上方向)' % tuple(R @ n_lidar))

    if ang < 1.0:
        print('\n  ⚠️ 转动量 < 1 度 → 外参基本就是单位阵。')
        print('     如果地图仍然明显倾斜，说明倾斜不是外参造成的，是软件/配置问题。')

    if args.write:
        import re
        s = CONFIG.read_text(encoding='utf-8')
        new = ('extrinsic_R: [ %9.6f, %9.6f, %9.6f,\n'
               '                           %9.6f, %9.6f, %9.6f,\n'
               '                           %9.6f, %9.6f, %9.6f]') % tuple(R.flatten())
        s2, n = re.subn(r'extrinsic_R:\s*\[[^\]]*\]', new, s)
        if n != 1:
            print('\n❌ 写回失败：extrinsic_R 匹配 %d 处' % n)
            return 1
        CONFIG.write_text(s2, encoding='utf-8')
        print('\n✅ 已写入 %s（重启 launch 生效）' % CONFIG)
    else:
        print('\n  （加 --write 可直接写回配置）')
    return 0


if __name__ == '__main__':
    sys.exit(main())
