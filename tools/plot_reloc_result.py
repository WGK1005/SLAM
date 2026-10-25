#!/usr/bin/env python3
import sys, os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

plt.rcParams['font.sans-serif'] = ['WenQuanYi Micro Hei', 'WenQuanYi Zen Hei', 'Droid Sans Fallback', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

def load_pcd(p):
    if not os.path.exists(p):
        return np.zeros((0, 3))
    fields, sizes, types, points = [], [], [], 0
    with open(p, 'rb') as f:
        while True:
            line = f.readline()
            if not line:
                break
            k = line.split()[0].decode(errors='ignore').upper()
            if k == 'POINTS':
                points = int(line.split()[1])
            elif k == 'FIELDS':
                fields = [x.decode() for x in line.split()[1:]]
            elif k == 'SIZE':
                sizes = [int(x) for x in line.split()[1:]]
            elif k == 'TYPE':
                types = [x.decode() for x in line.split()[1:]]
            elif k == 'DATA':
                break
        tmap = {'F': 'f', 'U': 'u', 'I': 'i'}
        dt = np.dtype([(n, f'<{tmap[t]}{s}') for n, s, t in zip(fields, sizes, types)])
        a = np.frombuffer(f.read(points * dt.itemsize), dtype=dt)
    P = np.stack([np.asarray(a['x'], float), np.asarray(a['y'], float), np.asarray(a['z'], float)], 1)
    return P[np.isfinite(P).all(1)]

def filter_wall_slice(pts, z_floor, min_h=0.20, max_h=1.85):
    if len(pts) == 0:
        return pts
    h = pts[:, 2] - z_floor
    return pts[(h >= min_h) & (h <= max_h)]

def main():
    home = os.path.expanduser('~')
    map_pcd_path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(home, 'maps/map_1021_1050.pcd')
    coarse_pcd_path = sys.argv[2] if len(sys.argv) > 2 else '/tmp/reloc_eval/scan_coarse.pcd'
    fine_pcd_path = sys.argv[3] if len(sys.argv) > 3 else '/tmp/reloc_eval/scan_fine.pcd'
    info_path = sys.argv[4] if len(sys.argv) > 4 else '/tmp/reloc_eval/info.txt'
    save_png = sys.argv[5] if len(sys.argv) > 5 else os.path.join(home, 'maps/reloc_process_result.png')

    info = {}
    if os.path.exists(info_path):
        with open(info_path) as f:
            for line in f:
                if '=' in line:
                    k, v = line.strip().split('=', 1)
                    try:
                        info[k] = float(v)
                    except ValueError:
                        pass

    map_pts = load_pcd(map_pcd_path)
    coarse_pts = load_pcd(coarse_pcd_path)
    fine_pts = load_pcd(fine_pcd_path)

    z_floor = info.get('z_floor', -0.445)
    has_gt = int(info.get('has_gt', 0)) == 1
    gt_x = info.get('gt_x', 0.0)
    gt_y = info.get('gt_y', 0.0)
    gt_yaw = info.get('gt_yaw', 0.0)
    coarse_x = info.get('coarse_x', 0.0)
    coarse_y = info.get('coarse_y', 0.0)
    coarse_yaw = info.get('coarse_yaw', 0.0)
    fine_x = info.get('fine_x', 0.0)
    fine_y = info.get('fine_y', 0.0)
    fine_z = info.get('fine_z', 0.0)
    fine_yaw = info.get('fine_yaw', 0.0)
    fitness = info.get('fitness', 95.0)
    wall_fitness = info.get('wall_fitness', fitness)
    wall_rmse_cm = info.get('wall_rmse', 0.05) * 100.0
    coarse_ms = info.get('coarse_ms', 40.0)
    fine_ms = info.get('fine_ms', 80.0)

    # 仅保留纯垂直墙面切片（彻底剥离地板和天花板干扰，让墙壁轮廓清晰锐利）
    map_walls = filter_wall_slice(map_pts, z_floor)
    coarse_walls = filter_wall_slice(coarse_pts, z_floor)
    fine_walls = filter_wall_slice(fine_pts, z_floor)

    x_min, x_max = -7.5, 3.8
    y_min, y_max = -2.5, 11.5
    res = 0.04
    xbins = int((x_max - x_min) / res)
    ybins = int((y_max - y_min) / res)

    H, _, _ = np.histogram2d(
        map_walls[:, 0], map_walls[:, 1],
        bins=[xbins, ybins], range=[[x_min, x_max], [y_min, y_max]]
    )
    H_log = np.log1p(H.T)
    vmax = np.percentile(H_log[H_log > 0], 95) if np.any(H_log > 0) else 1

    # ==================== 图 1：双栏配准过程诊断图 (reloc_process_result.png) ====================
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(18, 10), dpi=150)

    # ── 左图：阶段一 360° 纯墙面正负距离场粗匹配 ──
    ax1.imshow(H_log, origin='lower', extent=[x_min, x_max, y_min, y_max],
               cmap='gray_r', alpha=0.80, vmin=0, vmax=vmax)
    if len(coarse_walls) > 0:
        ax1.scatter(coarse_walls[:, 0], coarse_walls[:, 1], s=5, c='#ff8800', alpha=0.85,
                    label='粗匹配墙面切片点云 (360°正负距离场)')

    rad_c = np.radians(coarse_yaw)
    ax1.plot(coarse_x, coarse_y, 'o', color='darkorange', markersize=10)
    ax1.arrow(coarse_x, coarse_y, 0.85 * np.cos(rad_c), 0.85 * np.sin(rad_c),
              head_width=0.25, head_length=0.35, fc='gold', ec='darkorange', lw=2.2,
              label=f'粗匹配最佳候选 ({coarse_x:.2f}m, {coarse_y:.2f}m, {coarse_yaw:.1f}°)')
    if has_gt:
        ax1.plot(gt_x, gt_y, 'ro', markersize=9, label=f'基准真值坐标 ({gt_x:.2f}m, {gt_y:.2f}m)')

    ax1.set_title(f"【阶段一：地面解耦 + 360° 纯墙面正负距离场粗匹配】 ({coarse_ms:.1f} ms)\n"
                  f"最佳粗候选: X={coarse_x:.2f}m, Y={coarse_y:.2f}m, Yaw={coarse_yaw:.1f}°",
                  fontsize=12.5, fontweight='bold', color='#c65100')
    ax1.set_xlabel("全局坐标系 X (m)", fontsize=11)
    ax1.set_ylabel("全局坐标系 Y (m)", fontsize=11)
    ax1.set_xlim(x_min, x_max)
    ax1.set_ylim(y_min, y_max)
    ax1.grid(True, linestyle=':', alpha=0.5)
    ax1.legend(loc='lower left', framealpha=0.92, fontsize=10)

    # ── 右图：阶段二 多假设纯墙面 ICP + 6-DoF 精修 ──
    ax2.imshow(H_log, origin='lower', extent=[x_min, x_max, y_min, y_max],
               cmap='gray_r', alpha=0.80, vmin=0, vmax=vmax)
    if len(fine_walls) > 0:
        ax2.scatter(fine_walls[:, 0], fine_walls[:, 1], s=5, c='#00c853', alpha=0.90,
                    label='精配准后雷达墙面点云 (与地图墙体严丝合缝)')

    rad_f = np.radians(fine_yaw)
    ax2.plot(fine_x, fine_y, 'ro', markersize=11, zorder=5)
    ax2.arrow(fine_x, fine_y, 0.90 * np.cos(rad_f), 0.90 * np.sin(rad_f),
              head_width=0.28, head_length=0.38, fc='red', ec='darkred', lw=2.5, zorder=6,
              label=f'最终锁定位姿 (X={fine_x:.3f}m, Y={fine_y:.3f}m, Yaw={fine_yaw:.1f}°)')

    if has_gt:
        ax2.plot(gt_x, gt_y, 'bo', markersize=8, alpha=0.7, label=f'基准真值 ({gt_x:.2f}m, {gt_y:.2f}m)')
        err_dist_cm = np.hypot(fine_x - gt_x, fine_y - gt_y) * 100.0
        title_right = (f"【阶段二：Top-10 多假设墙面 ICP 精修】 ({fine_ms:.1f} ms)\n"
                       f"锁定: ({fine_x:.3f}m, {fine_y:.3f}m, {fine_yaw:.1f}°) | 误差: {err_dist_cm:.1f}cm | 纯墙面咬合: {wall_fitness:.1f}%")
    else:
        title_right = (f"【阶段二：Top-10 多假设墙面 ICP 精修】 ({fine_ms:.1f} ms)\n"
                       f"锁定坐标: X={fine_x:.3f}m, Y={fine_y:.3f}m, Yaw={fine_yaw:.1f}° | 纯墙面咬合: {wall_fitness:.1f}% (残差 {wall_rmse_cm:.1f}cm)")

    ax2.set_title(title_right, fontsize=12.5, fontweight='bold', color='#1b5e20')
    ax2.set_xlabel("全局坐标系 X (m)", fontsize=11)
    ax2.set_ylabel("全局坐标系 Y (m)", fontsize=11)
    ax2.set_xlim(x_min, x_max)
    ax2.set_ylim(y_min, y_max)
    ax2.grid(True, linestyle=':', alpha=0.5)
    ax2.legend(loc='lower left', framealpha=0.92, fontsize=10)

    import time, shutil
    ts = time.strftime("%m%d_%H%M%S")
    hist_dir = os.path.join(os.path.dirname(save_png), 'history')
    os.makedirs(hist_dir, exist_ok=True)

    plt.suptitle(f"开机无先验全局重定位 V2 全流程诊断图 [{ts}] (地面剔除 · 纯墙面正负距离场 · 多假设ICP校验)",
                 fontsize=15, fontweight='bold', y=0.98)
    plt.tight_layout()
    os.makedirs(os.path.dirname(save_png), exist_ok=True)
    plt.savefig(save_png, bbox_inches='tight')
    plt.close(fig)
    shutil.copy2(save_png, os.path.join(hist_dir, f"reloc_process_{ts}.png"))
    print(f"✅ [1/2] 匹配过程双栏诊断图已更新: {save_png} (已存档 reloc_process_{ts}.png)")

    # ==================== 图 2：单栏高清实机位置平面图 (real_robot_reloc_location.png) ====================
    loc_png = os.path.join(os.path.dirname(save_png), 'real_robot_reloc_location.png')
    fig2, ax = plt.subplots(figsize=(11, 10), dpi=150)
    ax.imshow(H_log, origin='lower', extent=[x_min, x_max, y_min, y_max],
              cmap='gray_r', alpha=0.78, vmin=0, vmax=vmax)
    if len(fine_walls) > 0:
        ax.scatter(fine_walls[:, 0], fine_walls[:, 1], s=4, c='#00c853', alpha=0.75,
                   label='当前雷达扫描墙面点云 (已对齐)')

    ax.plot(fine_x, fine_y, 'ro', markersize=15, zorder=5,
            label=f'当前实机锁定位置 (X={fine_x:.3f}m, Y={fine_y:.3f}m)')
    ax.arrow(fine_x, fine_y, 1.0 * np.cos(rad_f), 1.0 * np.sin(rad_f),
             head_width=0.35, head_length=0.45, fc='red', ec='darkred', lw=3.5, zorder=6,
             label=f'实机航向朝向 (Yaw={fine_yaw:.1f}°)')

    info_box = (
        f"[当前实机锁定状态 V2 | {ts}]\n"
        f"坐标 X : {fine_x:.3f} m\n"
        f"坐标 Y : {fine_y:.3f} m\n"
        f"高度 Z : {fine_z:.3f} m\n"
        f"航向 Yaw : {fine_yaw:.2f}°\n"
        f"纯墙面咬合 : {wall_fitness:.1f}%\n"
        f"墙面残差 : {wall_rmse_cm:.2f} cm\n"
        f"总计算耗时 : {coarse_ms + fine_ms:.1f} ms"
    )
    # 自动选择文本框偏移方向防止出界
    offset_x = 0.45 if fine_x < 0.0 else -3.2
    offset_y = -0.3 if fine_y > 2.0 else 1.2
    ax.text(fine_x + offset_x, fine_y + offset_y, info_box,
            fontsize=10.5, fontweight='bold', color='#b71c1c', zorder=7,
            bbox=dict(boxstyle='round,pad=0.6', facecolor='#fffde7', alpha=0.95, edgecolor='#d32f2f', lw=1.5))

    ax.set_title(f"实机开机无先验全局自定位结果 [{ts}] (V2 纯墙面几何锁定)\n"
                 f"X = {fine_x:.3f} m, Y = {fine_y:.3f} m, Yaw = {fine_yaw:.1f}° · 纯墙面咬合度 {wall_fitness:.1f}%",
                 fontsize=13.5, fontweight='bold', color='#1b5e20', pad=14)
    ax.set_xlabel("全局坐标系 X (米)", fontsize=12)
    ax.set_ylabel("全局坐标系 Y (米)", fontsize=12)
    ax.set_xlim(x_min, x_max)
    ax.set_ylim(y_min, y_max)
    ax.grid(True, linestyle='--', alpha=0.5)
    ax.legend(loc='upper right', fontsize=10.5, framealpha=0.92)
    plt.tight_layout()
    plt.savefig(loc_png, bbox_inches='tight')
    plt.close(fig2)
    shutil.copy2(loc_png, os.path.join(hist_dir, f"reloc_{ts}.png"))
    print(f"✅ [2/2] 实机平面位置红点图已更新: {loc_png} (已存档 reloc_{ts}.png)")

if __name__ == '__main__':
    main()
