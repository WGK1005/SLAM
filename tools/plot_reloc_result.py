#!/usr/bin/env python3
import sys, os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

plt.rcParams['font.sans-serif'] = ['WenQuanYi Micro Hei', 'WenQuanYi Zen Hei', 'Droid Sans Fallback', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

def load_pcd(p):
    if not os.path.exists(p): return np.zeros((0, 3))
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
    map_pcd_path = sys.argv[1] if len(sys.argv) > 1 else '/home/wgk/maps/map_1021_1050.pcd'
    coarse_pcd_path = sys.argv[2] if len(sys.argv) > 2 else '/tmp/reloc_eval/scan_coarse.pcd'
    fine_pcd_path = sys.argv[3] if len(sys.argv) > 3 else '/tmp/reloc_eval/scan_fine.pcd'
    info_path = sys.argv[4] if len(sys.argv) > 4 else '/tmp/reloc_eval/info.txt'
    save_png = sys.argv[5] if len(sys.argv) > 5 else '/home/wgk/maps/reloc_process_result.png'

    # 读取指标信息
    info = {}
    if os.path.exists(info_path):
        with open(info_path) as f:
            for line in f:
                if '=' in line:
                    k, v = line.strip().split('=', 1)
                    info[k] = float(v)

    map_pts = load_pcd(map_pcd_path)
    coarse_pts = load_pcd(coarse_pcd_path)
    fine_pts = load_pcd(fine_pcd_path)

    # 截取房间和走廊高度截面 [-0.5m, 1.5m]
    z_mask = (map_pts[:, 2] > -0.5) & (map_pts[:, 2] < 1.5)
    map_crop = map_pts[z_mask]

    # 确定显示范围 (聚焦在有效建筑范围)
    x_min, x_max = -8.0, 4.0
    y_min, y_max = -3.0, 12.0
    res = 0.05
    xbins = int((x_max - x_min) / res)
    ybins = int((y_max - y_min) / res)

    H, _, _ = np.histogram2d(
        map_crop[:, 0], map_crop[:, 1],
        bins=[xbins, ybins], range=[[x_min, x_max], [y_min, y_max]]
    )
    H_log = np.log1p(H.T)

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(18, 10), dpi=150)

    gt_x = info.get('gt_x', 0.0)
    gt_y = info.get('gt_y', 0.0)
    gt_yaw = info.get('gt_yaw', 0.0)
    coarse_x = info.get('coarse_x', 0.0)
    coarse_y = info.get('coarse_y', 0.0)
    coarse_yaw = info.get('coarse_yaw', 0.0)
    fine_x = info.get('fine_x', 0.0)
    fine_y = info.get('fine_y', 0.0)
    fine_yaw = info.get('fine_yaw', 0.0)
    fitness = info.get('fitness', 100.0)
    rmse = info.get('rmse', 0.037)
    coarse_ms = info.get('coarse_ms', 50.0)
    fine_ms = info.get('fine_ms', 40.0)

    # ── 左图：粗匹配阶段 ──
    vmax = np.percentile(H_log[H_log > 0], 95) if np.any(H_log > 0) else 1
    ax1.imshow(H_log, origin='lower', extent=[x_min, x_max, y_min, y_max],
               cmap='gray_r', alpha=0.75, vmin=0, vmax=vmax)
    if len(coarse_pts) > 0:
        ax1.scatter(coarse_pts[:, 0], coarse_pts[:, 1], s=4, c='#ff9900', alpha=0.85, label='粗匹配候选扫描 (360°似然搜索)')
    
    # 画真值和粗候选箭头
    rad_c = np.radians(coarse_yaw)
    ax1.arrow(coarse_x, coarse_y, 0.8 * np.cos(rad_c), 0.8 * np.sin(rad_c),
              head_width=0.25, head_length=0.35, fc='gold', ec='darkorange', lw=2, label=f'粗候选位姿 (Yaw={coarse_yaw:.1f}°)')
    ax1.plot(gt_x, gt_y, 'ro', markersize=9, label=f'开机真值坐标 ({gt_x:.2f}, {gt_y:.2f})')

    ax1.set_title(f"【阶段一：360° 全局几何粗匹配】 (耗时 {coarse_ms:.1f} ms)\n粗候选位置: X={coarse_x:.2f}m, Y={coarse_y:.2f}m, Yaw={coarse_yaw:.1f}° (初步锁定房间)",
                  fontsize=13, fontweight='bold', color='darkorange')
    ax1.set_xlabel("X (m)", fontsize=11)
    ax1.set_ylabel("Y (m)", fontsize=11)
    ax1.set_xlim(x_min, x_max)
    ax1.set_ylim(y_min, y_max)
    ax1.grid(True, linestyle=':', alpha=0.5)
    ax1.legend(loc='lower left', framealpha=0.9)

    # ── 右图：精微调阶段 ──
    ax2.imshow(H_log, origin='lower', extent=[x_min, x_max, y_min, y_max],
               cmap='gray_r', alpha=0.75, vmin=0, vmax=vmax)
    if len(fine_pts) > 0:
        ax2.scatter(fine_pts[:, 0], fine_pts[:, 1], s=4, c='#00dd00', alpha=0.85, label='点到面 ICP 亚厘米精修咬合点云')

    rad_f = np.radians(fine_yaw)
    ax2.arrow(fine_x, fine_y, 0.8 * np.cos(rad_f), 0.8 * np.sin(rad_f),
              head_width=0.25, head_length=0.35, fc='lime', ec='darkgreen', lw=2, label=f'最终精准位姿 (Yaw={fine_yaw:.1f}°)')
    ax2.plot(gt_x, gt_y, 'ro', markersize=9, label=f'开机真值坐标 ({gt_x:.2f}, {gt_y:.2f})')

    err_dist_mm = np.hypot(fine_x - gt_x, fine_y - gt_y) * 1000.0
    err_yaw = abs(fine_yaw - gt_yaw)
    if err_yaw > 180: err_yaw = 360 - err_yaw

    ax2.set_title(f"【阶段二：两级点到面 ICP 精修】 (耗时 {fine_ms:.1f} ms)\n最终位姿: X={fine_x:.3f}m, Y={fine_y:.3f}m | 平移误差: {err_dist_mm:.1f} 毫米 | 咬合度: {fitness:.1f}%",
                  fontsize=13, fontweight='bold', color='darkgreen')
    ax2.set_xlabel("X (m)", fontsize=11)
    ax2.set_ylabel("Y (m)", fontsize=11)
    ax2.set_xlim(x_min, x_max)
    ax2.set_ylim(y_min, y_max)
    ax2.grid(True, linestyle=':', alpha=0.5)
    ax2.legend(loc='lower left', framealpha=0.9)

    plt.suptitle("开机无初值全局特征匹配过程全流程可视化 (粗匹配锁定 → ICP高精咬合)", fontsize=16, fontweight='bold', y=0.98)
    plt.tight_layout()
    os.makedirs(os.path.dirname(save_png), exist_ok=True)
    plt.savefig(save_png, bbox_inches='tight')
    print(f"✅ 匹配过程图像已生成并保存至: {save_png}")

if __name__ == '__main__':
    main()

