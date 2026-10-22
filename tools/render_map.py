#!/usr/bin/env python3
"""把点云地图（PCD 或 ROS 话题）渲染成图片

支持两种输入：
  1) PCD 文件（FAST-LIO 的 /map_save 输出，binary 格式）
        python3 tools/render_map.py --pcd ~/maps/pc_base2.pcd --out ~/maps/pc_base2.png
  2) ROS 话题（实时/回放时直接出图）
        python3 tools/render_map.py --topic /cloud_registered --duration 20 --out /tmp/map.png

输出：一张 1x3 的图
  左  ：占据密度俯视（log 灰度）—— 看结构、墙、通道，**最清楚**
  中  ：高度着色的俯视散点（色标按 2~98 分位裁剪）—— 看高度分布
  右  ：3/4 视角（抽样）—— 看立体结构

★ 为什么需要 z 带过滤：
  FAST-LIO 的累积地图里绝大多数点是地面（实测某些场景 71% 的点挤在 ±0.5m 内），
  不过滤的话俯视就是一片糊。--z-band auto 会自动找地面高度并只保留其上方的点。

用法示例
    # 自动找地面、自动裁剪离群点（推荐）
    python3 tools/render_map.py --pcd map.pcd --out map.png

    # 指定高度带（米，相对地图坐标系原点）
    python3 tools/render_map.py --pcd map.pcd --out map.png --z-min 0.3 --z-max 2.5

    # 想看含地面的原始效果
    python3 tools/render_map.py --pcd map.pcd --out map.png --z-band raw
"""
import argparse
import sys

import matplotlib
matplotlib.use('Agg')          # 无界面环境
import matplotlib.pyplot as plt
import numpy as np


# ------------------------------ PCD（binary）读取 ------------------------------
def load_pcd_binary(path):
    with open(path, 'rb') as f:
        fields, sizes, types, points, data_fmt = [], [], [], 0, None
        while True:
            line = f.readline()
            if not line:
                raise RuntimeError('PCD 头解析失败')
            if line.startswith(b'#') or not line.strip():
                continue
            key = line.split()[0].decode(errors='ignore').upper()
            if key == 'FIELDS':
                fields = [x.decode() for x in line.split()[1:]]
            elif key == 'SIZE':
                sizes = [int(x) for x in line.split()[1:]]
            elif key == 'TYPE':
                types = [x.decode() for x in line.split()[1:]]
            elif key == 'POINTS':
                points = int(line.split()[1])
            elif key == 'DATA':
                data_fmt = line.split()[1].decode().lower()
                break
        if data_fmt != 'binary':
            raise RuntimeError('只支持 binary PCD，当前是 %s' % data_fmt)
        tmap = {'F': 'f', 'U': 'u', 'I': 'i'}
        dtype = [(n, '<%s%d' % (tmap[t], s)) for n, s, t in zip(fields, sizes, types)]
        arr = np.frombuffer(f.read(points * np.dtype(dtype).itemsize), dtype=np.dtype(dtype))
    return arr


def load_pcd_any(path):
    head = open(path, 'rb').read(4096)
    if b'DATA binary' in head:
        return load_pcd_binary(path)
    raise RuntimeError('ASCII PCD 暂不支持（用 CloudCompare 另存为 binary 即可）')


# ------------------------------ ROS 话题读取 ------------------------------
def load_topic(topic, duration):
    import time

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2
    import sensor_msgs_py.point_cloud2 as pc2

    class Grab(Node):
        def __init__(s):
            super().__init__('map_grab')
            s.best = None
            s.create_subscription(PointCloud2, topic, s.cb, qos_profile_sensor_data)

        def cb(s, m):
            if s.best is None or m.width * m.height > s.best[0]:
                pts = list(pc2.read_points(m, field_names=('x', 'y', 'z'), skip_nans=True))
                s.best = (len(pts), np.array(pts, dtype=np.float32))

    rclpy.init()
    n = Grab()
    t0 = time.time()
    while rclpy.ok() and time.time() - t0 < duration:
        rclpy.spin_once(n, timeout_sec=0.2)
    n.destroy_node()
    rclpy.shutdown()
    if n.best is None:
        raise RuntimeError('%s 没收到点云' % topic)
    a = n.best[1]
    return a.view([('x', '<f4'), ('y', '<f4'), ('z', '<f4')]).reshape(-1)


# ---------------------------------- 工具 ----------------------------------
def detect_floor(z):
    """用 z 直方图峰值估计地面高度（大多数点所在的那一层）"""
    if len(z) < 100:
        return float(np.median(z))
    hist, edges = np.histogram(z, bins=120)
    k = int(np.argmax(hist))
    return float(0.5 * (edges[k] + edges[k + 1]))


# ---------------------------------- 渲染 ----------------------------------
def render(xyz, out_path, title, max_range=None, z_band='auto',
           z_min=None, z_max=None, cell=0.05, max_plot=400000):
    x = np.asarray(xyz['x'], dtype=np.float32)
    y = np.asarray(xyz['y'], dtype=np.float32)
    z = np.asarray(xyz['z'], dtype=np.float32)
    ok = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
    x, y, z = x[ok], y[ok], z[ok]
    n_all = len(x)

    r = np.sqrt(x * x + y * y + z * z)
    if max_range is None:
        max_range = float(np.percentile(r, 99.5))
    keep = r <= max_range
    x, y, z = x[keep], y[keep], z[keep]
    print('  总点数 %d | 裁剪 |r|<=%.1fm 后 %d 点 (%.1f%%)'
          % (n_all, max_range, len(x), 100.0 * len(x) / max(n_all, 1)))
    if len(x) == 0:
        raise RuntimeError('裁剪后没有点，检查 --max-range')

    floor = detect_floor(z)
    print('  检测到地面高度 z ≈ %.2f m' % floor)

    # ---- 高度带过滤 ----
    zz = z.copy()
    if z_min is not None or z_max is not None:
        lo = z_min if z_min is not None else float(z.min())
        hi = z_max if z_max is not None else float(z.max())
    elif z_band == 'raw':
        lo, hi = float(z.min()), float(z.max())
    else:                                   # auto：地面往上 0.15~3.0 m
        lo, hi = floor + 0.15, floor + 3.0
    m = (zz >= lo) & (zz <= hi)
    print('  高度带 [%.2f, %.2f] m -> 保留 %d 点 (%.1f%%)'
          % (lo, hi, int(m.sum()), 100.0 * m.sum() / len(x)))
    if m.sum() < 50:
        print('  [warn] 高度带内点太少，改为不做高度过滤')
        m = np.ones(len(x), dtype=bool)
        lo, hi = float(z.min()), float(z.max())
    xs, ys, zs = x[m], y[m], z[m]

    fig = plt.figure(figsize=(21, 7))
    fig.suptitle(title, fontsize=13)

    # ---------- 左：占据密度俯视 ----------
    ax1 = fig.add_subplot(1, 3, 1)
    xe = np.arange(xs.min(), xs.max() + cell, cell)
    ye = np.arange(ys.min(), ys.max() + cell, cell)
    H, _, _ = np.histogram2d(xs, ys, bins=[xe, ye])
    occ = (H > 0).astype(float)
    # 用 log 密度着色 + 二值轮廓：墙会显示为深色实线
    Hlog = np.log1p(H.T)
    ax1.imshow(Hlog, origin='lower', cmap='gray_r', aspect='equal',
               extent=[xe[0], xe[-1], ye[0], ye[-1]], interpolation='nearest')
    ax1.plot(0, 0, 'r*', markersize=14, label='origin (start)')
    ax1.set_xlabel('X (m)'); ax1.set_ylabel('Y (m)')
    ax1.set_title('Occupancy (log density) - structure view')
    ax1.grid(alpha=0.15, color='c'); ax1.legend(loc='upper right', fontsize=9)

    # ---------- 中：高度着色俯视 ----------
    ax2 = fig.add_subplot(1, 3, 2)
    idx = np.arange(len(xs))
    if len(xs) > max_plot:
        idx = np.random.default_rng(0).choice(len(xs), max_plot, replace=False)
    vmin, vmax = np.percentile(zs, [2, 98])
    if vmax - vmin < 1e-6:
        vmin, vmax = zs.min(), zs.max() + 1e-6
    sc = ax2.scatter(xs[idx], ys[idx], c=zs[idx], s=0.6, cmap='turbo',
                     vmin=vmin, vmax=vmax, linewidths=0)
    ax2.plot(0, 0, 'r*', markersize=14, label='origin (start)')
    ax2.set_xlabel('X (m)'); ax2.set_ylabel('Y (m)')
    ax2.set_title('Top view (color = Z, clipped %.1f~%.1f m)' % (vmin, vmax))
    ax2.set_aspect('equal'); ax2.grid(alpha=0.25); ax2.legend(loc='upper right', fontsize=9)
    plt.colorbar(sc, ax=ax2, label='Z (m)', shrink=0.85)

    # ---------- 右：3/4 视角 ----------
    ax3 = fig.add_subplot(1, 3, 3, projection='3d')
    n2 = min(len(xs), 120000)
    i2 = np.random.default_rng(1).choice(len(xs), n2, replace=False) if len(xs) > n2 else np.arange(len(xs))
    ax3.scatter(xs[i2], ys[i2], zs[i2], c=zs[i2], s=0.5, cmap='turbo',
                vmin=vmin, vmax=vmax, linewidths=0)
    ax3.set_xlabel('X (m)'); ax3.set_ylabel('Y (m)'); ax3.set_zlabel('Z (m)')
    ax3.set_title('3/4 view (%d pts)' % n2)
    try:
        ax3.set_box_aspect((1, 1, 0.4))
    except Exception:
        pass
    ax3.view_init(elev=30, azim=-60)

    plt.tight_layout()
    plt.savefig(out_path, dpi=120)
    print('  ✅ 图片已保存: %s' % out_path)
    print('     范围 x[%.1f,%.1f] y[%.1f,%.1f]  占用格 %d 个 (%.0f m2)'
          % (xs.min(), xs.max(), ys.min(), ys.max(), int(occ.sum()), occ.sum() * cell * cell))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--pcd', help='PCD 文件路径（binary）')
    ap.add_argument('--topic', help='ROS 点云话题（与 --pcd 二选一）')
    ap.add_argument('--duration', type=float, default=20.0, help='从话题取数时长（秒）')
    ap.add_argument('--out', required=True, help='输出 PNG 路径')
    ap.add_argument('--title', default='FAST-LIO map')
    ap.add_argument('--max-range', type=float, default=None,
                    help='裁剪半径(m)；默认取 |r| 的 99.5%% 分位，剔除极端离群点')
    ap.add_argument('--z-band', choices=['auto', 'raw'], default='auto',
                    help='auto=只画地面以上的点（默认）；raw=全部')
    ap.add_argument('--z-min', type=float, default=None, help='高度带下限(m)，覆盖 --z-band')
    ap.add_argument('--z-max', type=float, default=None, help='高度带上限(m)，覆盖 --z-band')
    ap.add_argument('--cell', type=float, default=0.05, help='占据图分辨率(m)，默认 0.05')
    args = ap.parse_args()

    if args.pcd:
        print('读取 %s …' % args.pcd)
        xyz = load_pcd_any(args.pcd)
    elif args.topic:
        print('订阅 %s 取 %.0f 秒 …' % (args.topic, args.duration))
        xyz = load_topic(args.topic, args.duration)
    else:
        print('需要 --pcd 或 --topic', file=sys.stderr)
        return 1
    render(xyz, args.out, args.title, max_range=args.max_range,
           z_band=args.z_band, z_min=args.z_min, z_max=args.z_max, cell=args.cell)
    return 0


if __name__ == '__main__':
    sys.exit(main())
