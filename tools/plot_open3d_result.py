#!/usr/bin/env python3
import sys, os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

plt.rcParams['font.sans-serif'] = ['WenQuanYi Micro Hei', 'WenQuanYi Zen Hei', 'Droid Sans Fallback']
plt.rcParams['axes.unicode_minus'] = False

def load_pcd(p):
    fields, sizes, types, points = [], [], [], 0
    with open(p, 'rb') as f:
        while True:
            line = f.readline()
            if not line: break
            k = line.split()[0].decode(errors='ignore').upper()
            if k == 'POINTS': points = int(line.split()[1])
            elif k == 'FIELDS': fields = [x.decode() for x in line.split()[1:]]
            elif k == 'SIZE': sizes = [int(x) for x in line.split()[1:]]
            elif k == 'TYPE': types = [x.decode() for x in line.split()[1:]]
            elif k == 'DATA': break
        tmap = {'F': 'f', 'U': 'u', 'I': 'i'}
        dt = np.dtype([(n, f'<{tmap[t]}{s}') for n, s, t in zip(fields, sizes, types)])
        a = np.frombuffer(f.read(points * dt.itemsize), dtype=dt)
    P = np.stack([np.asarray(a['x'], float), np.asarray(a['y'], float), np.asarray(a['z'], float)], 1)
    return P[np.isfinite(P).all(1)]

def main():
    out_dir = sys.argv[1] if len(sys.argv) > 1 else '/tmp/o3d_eval'
    map_pcd = sys.argv[2] if len(sys.argv) > 2 else '/home/wgk/maps/pc_base2.pcd'
    save_png = sys.argv[3] if len(sys.argv) > 3 else '/home/wgk/maps/lab_open3d_result.png'

    # 读取测试指标
    with open(os.path.join(out_dir, 'result.txt')) as f:
        lines = [x.strip() for x in f if x.strip()]
        cost_ms = float(lines[0])
        init_fit = float(lines[1])
        final_fit = float(lines[2])
        rmse = float(lines[3])
        off_x, off_y, off_yaw = float(lines[4]), float(lines[5]), float(lines[6])
        est_x, est_y, est_yaw = float(lines[7]), float(lines[8]), float(lines[9])

    map_pts = load_pcd(map_pcd)
    init_pts = load_pcd(os.path.join(out_dir, 'scan_initial.pcd'))
    align_pts = load_pcd(os.path.join(out_dir, 'scan_aligned.pcd'))

    # 滤掉地图中过高过低的点，聚焦在房间结构
    z_mask = (map_pts[:, 2] > -0.5) & (map_pts[:, 2] < 2.5)
    map_crop = map_pts[z_mask]

    # 计算 2D 占据栅格直方图 (与 render_map.py 相同算法)
    res = 0.05
    all_x = np.concatenate([init_pts[:, 0], align_pts[:, 0]])
    all_y = np.concatenate([init_pts[:, 1], align_pts[:, 1]])
    margin = 2.5
    x_min = float(np.floor(all_x.min() - margin))
    x_max = float(np.ceil(all_x.max() + margin))
    y_min = float(np.floor(all_y.min() - margin))
    y_max = float(np.ceil(all_y.max() + margin))

    xbins = int((x_max - x_min) / res)
    ybins = int((y_max - y_min) / res)

    H, xedges, yedges = np.histogram2d(
        map_crop[:, 0], map_crop[:, 1],
        bins=[xbins, ybins], range=[[x_min, x_max], [y_min, y_max]]
    )
    H_log = np.log1p(H.T)

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(18, 9), dpi=150)

    # ── 左图：配准前（带偏差的粗估计） ──
    ax1.imshow(H_log, origin='lower', extent=[x_min, x_max, y_min, y_max],
               cmap='gray_r', alpha=0.75, vmin=0, vmax=np.percentile(H_log[H_log > 0], 95) if np.any(H_log>0) else 1)
    ax1.scatter(init_pts[:, 0], init_pts[:, 1], s=4, c='red', alpha=0.85, label='初始粗坐标扫描 (偏差+0.5m, +15°)')
    ax1.plot([0, off_x], [0, off_y], 'r--', lw=2)
    ax1.plot(0, 0, 'y*', markersize=14, label='地图起点原点 (0,0)')
    ax1.plot(off_x, off_y, 'ro', markersize=10, label='粗坐标位置')
    ax1.set_title(f"【配准前】粗坐标位置 (dx={off_x:+.2f}m, dy={off_y:+.2f}m, dyaw={off_yaw:+.1f}°)\n重合度 (Fitness): {init_fit*100:.1f} %  (明显错位)",
                  fontsize=13, fontweight='bold', color='darkred')
    ax1.set_xlabel("X (m)", fontsize=11)
    ax1.set_ylabel("Y (m)", fontsize=11)
    ax1.set_xlim(x_min, x_max)
    ax1.set_ylim(y_min, y_max)
    ax1.grid(True, linestyle=':', alpha=0.5)
    ax1.legend(loc='lower left', framealpha=0.9)

    # ── 右图：配准后（Open3D ICP 纠正对齐） ──
    ax2.imshow(H_log, origin='lower', extent=[x_min, x_max, y_min, y_max],
               cmap='gray_r', alpha=0.75, vmin=0, vmax=np.percentile(H_log[H_log > 0], 95) if np.any(H_log>0) else 1)
    ax2.scatter(align_pts[:, 0], align_pts[:, 1], s=4, c='#00cc00', alpha=0.85, label='Open3D ICP 收敛对齐点云')
    ax2.plot(0, 0, 'y*', markersize=14, label='地图起点原点 (0,0)')
    ax2.plot(est_x, est_y, 'g^', markersize=10, label=f'Open3D 定位位姿 ({est_x:.3f}, {est_y:.3f})')
    ax2.set_title(f"【配准后】Open3D ICP 精确对齐 (耗时 {cost_ms:.1f} ms)\n重合度 (Fitness): {final_fit*100:.1f} % | 点均误差: {rmse*100:.2f} cm (严丝合缝贴合)",
                  fontsize=13, fontweight='bold', color='darkgreen')
    ax2.set_xlabel("X (m)", fontsize=11)
    ax2.set_ylabel("Y (m)", fontsize=11)
    ax2.set_xlim(x_min, x_max)
    ax2.set_ylim(y_min, y_max)
    ax2.grid(True, linestyle=':', alpha=0.5)
    ax2.legend(loc='lower left', framealpha=0.9)

    plt.suptitle("Open3D 在实验室灰度结构图上的配准定位效果对比", fontsize=16, fontweight='bold', y=0.98)
    plt.tight_layout()
    plt.savefig(save_png, bbox_inches='tight')
    print(f"✅ 可视化结果图片已保存至: {save_png}")

if __name__ == '__main__':
    main()
