#!/usr/bin/env python3
"""
将实机 3D 激光先验点云地图 (map_1021_1050.pcd) 精修规整为如 rmuc2025.png 一样纯净的黑白二值底图 (lab_map_1021.pgm)
并生成对标 ROS 2 Nav2 的三层膨胀区域图 (禁止区 / 挤压区 / 通行区)。
"""
import os
import sys
import numpy as np
from scipy import ndimage

def load_pcd(p):
    if not os.path.exists(p):
        raise FileNotFoundError(f"找不到点云文件: {p}")
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

def build_nav2_3zone_rgb(obs_mask, dist_m, forbid_r=0.22, squeeze_r=0.50):
    """生成对标 ROS 2 Nav2 RViz 风格的三层区域彩色图 (禁止区 / 挤压区 / 通行区)"""
    H, W = obs_mask.shape
    rgb = np.ones((H, W, 3), dtype=np.float32)  # 通行区: 纯白色 (1, 1, 1)

    # 1. 挤压区 (Squeeze / Inflation Zone): 0.22m < d <= 0.50m -> 青蓝色 + 外缘粉红线
    squeeze_mask = (dist_m > forbid_r) & (dist_m <= squeeze_r)
    rgb[squeeze_mask] = [0.58, 0.93, 0.98]
    squeeze_border = (dist_m > squeeze_r - 0.05) & (dist_m <= squeeze_r)
    rgb[squeeze_border] = [0.94, 0.65, 0.70]

    # 2. 禁止区 (Forbidden / Inscribed Zone): 0 < d <= 0.22m -> 深蓝色 + 内切紫品红边界
    forbid_mask = (dist_m > 0.0) & (dist_m <= forbid_r)
    rgb[forbid_mask] = [0.22, 0.58, 0.88]
    forbid_border = (dist_m > forbid_r - 0.05) & (dist_m <= forbid_r)
    rgb[forbid_border] = [0.88, 0.12, 0.62]

    # 3. 障碍物本体 (Lethal Obstacle): d == 0 -> 纯黑/深紫黑
    rgb[obs_mask] = [0.06, 0.03, 0.12]
    return rgb

def main():
    pcd_path = sys.argv[1] if len(sys.argv) > 1 else '/home/wgk/maps/map_1021_1050.pcd'
    out_dir = sys.argv[2] if len(sys.argv) > 2 else '/home/wgk/decision_planning/SCU-TDT-V1/maps'
    os.makedirs(out_dir, exist_ok=True)

    pts = load_pcd(pcd_path)
    z_floor = -0.470
    res = 0.05
    min_x, max_x = -7.50, 3.50   # 宽 11.0m -> 220 px
    min_y, max_y = -2.50, 11.50  # 高 14.0m -> 280 px
    W = int(round((max_x - min_x) / res))
    H = int(round((max_y - min_y) / res))

    gx = np.floor((pts[:, 0] - min_x) / res).astype(int)
    gy = np.floor((pts[:, 1] - min_y) / res).astype(int)
    valid = (gx >= 0) & (gx < W) & (gy >= 0) & (gy < H)
    pts, gx, gy = pts[valid], gx[valid], gy[valid]
    z = pts[:, 2]

    # 原始非地面障碍物投影 (用于对比展示原始点云的散点与走廊残影)
    obs_pts_mask = (z >= -0.33) & (z <= 1.55)
    obs_cnt = np.zeros((H, W), dtype=np.int32)
    np.add.at(obs_cnt, (gy[obs_pts_mask], gx[obs_pts_mask]), 1)
    raw_2d_obs = (obs_cnt >= 3)

    # ==================== 构建如 rmuc2025.png 一样干净规整的实验室黑白底图 ====================
    # 对照实机点云真实尺寸与建图行走通道，将外墙、靠墙实验台、上下排中央实验台岛规整化为严密闭合的纯黑几何块 (0)，
    # 将实际可通行走廊 (南侧起始通道 -> 左侧宽敞主走廊 -> 中段横廊 / 北侧横廊) 规整为 0 噪点纯白通道 (255)。
    yy, xx = np.mgrid[0:H, 0:W]
    wx = min_x + (xx + 0.5) * res
    wy = min_y + (yy + 0.5) * res

    # 默认外部未探索区域为黑色障碍 (0)，室内净空区域 [-5.35, 1.65] x [-1.20, 10.35] 初始化为可通行纯白 (255)
    grid = np.zeros((H, W), dtype=np.uint8)
    inside_room = (wx >= -5.35) & (wx <= 1.65) & (wy >= -1.20) & (wy <= 10.35)
    grid[inside_room] = 255

    # 1. 西侧靠墙实验台与立柱区 (X ∈ [-5.35, -3.85], Y ∈ [-1.20, 9.05])
    west_benches = (wx >= -5.35) & (wx <= -3.85) & (wy >= -1.20) & (wy <= 9.05)
    grid[west_benches] = 0
    # 西侧北角立柱凹槽
    west_north_pillar = (wx >= -5.35) & (wx <= -4.45) & (wy >= 9.05) & (wy <= 10.35)
    grid[west_north_pillar] = 0

    # 2. 南侧靠墙柜体区 (Y ∈ [-1.20, -0.38], X ∈ [-3.85, 1.65])
    south_cabinets = (wx >= -3.85) & (wx <= 1.65) & (wy >= -1.20) & (wy <= -0.38)
    grid[south_cabinets] = 0

    # 3. 东侧靠墙实验台与右侧封堵区 (X ∈ [0.65, 1.65], Y ∈ [-1.20, 10.35])
    east_benches = (wx >= 0.65) & (wx <= 1.65) & (wy >= -1.20) & (wy <= 10.35)
    grid[east_benches] = 0

    # 4. 下排中央实验台岛 + 右侧连接封堵障碍 (X ∈ [-2.15, 0.65], Y ∈ [0.45, 4.80])
    #    彻底封死起点 (0.03, 0.06) 正北方的障碍物区，机器人必须向西走南侧通道进入西侧主走廊!
    lower_island = (wx >= -2.15) & (wx <= 0.65) & (wy >= 0.45) & (wy <= 4.80)
    grid[lower_island] = 0

    # 5. 上排中央实验台岛 + 右侧封堵设备区 (X ∈ [-2.15, 0.65], Y ∈ [5.80, 9.00])
    #    保留中段死胡同凹槽 (Y ∈ [4.80, 5.80], X ∈ [-2.15, 0.15])，但右侧不贯通，北上必须走西侧宽阔主走廊!
    upper_island = (wx >= -2.15) & (wx <= 0.65) & (wy >= 5.80) & (wy <= 9.00)
    grid[upper_island] = 0
    mid_east_block = (wx >= 0.15) & (wx <= 0.65) & (wy >= 4.80) & (wy <= 5.80)
    grid[mid_east_block] = 0

    # 6. 北侧靠墙实验台局部凸起 (Y ∈ [9.90, 10.35], X ∈ [-3.20, 0.65])
    north_wall_bench = (wx >= -3.20) & (wx <= 0.65) & (wy >= 9.95) & (wy <= 10.35)
    grid[north_wall_bench] = 0

    obs_clean = (grid == 0)

    pgm_path = os.path.join(out_dir, 'lab_map_1021.pgm')
    with open(pgm_path, 'wb') as f:
        f.write(f"P5\n{W} {H}\n255\n".encode())
        f.write(grid.tobytes())

    png_clean_path = os.path.join(out_dir, 'lab_map_1021.png')
    yaml_path = os.path.join(out_dir, 'lab_map_1021.yaml')
    with open(yaml_path, 'w') as f:
        f.write(f"""image: lab_map_1021.pgm
resolution: {res}
origin: [{min_x}, {min_y}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
width: {W}
height: {H}
z_floor: {z_floor}
""")

    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        import matplotlib.image as mpimg
        import matplotlib.patches as mpatches

        mpimg.imsave(png_clean_path, grid, cmap='gray', vmin=0, vmax=255)
        plt.rcParams['font.sans-serif'] = ['WenQuanYi Micro Hei', 'Noto Sans CJK SC', 'SimHei', 'DejaVu Sans']
        plt.rcParams['axes.unicode_minus'] = False

        dist_m = ndimage.distance_transform_edt(~obs_clean) * res
        rgb_3zone = build_nav2_3zone_rgb(obs_clean, dist_m, forbid_r=0.22, squeeze_r=0.50)

        fig, axes = plt.subplots(1, 3, figsize=(18, 6.2), dpi=160)
        fig.suptitle("【实验室地图精修对比】原始散乱点云投影 vs rmuc2025 风格纯净黑白底图 vs Nav2 三层膨胀区域图",
                     fontsize=14.5, fontweight='bold', y=0.98)

        ax1 = axes[0]
        ax1.imshow(raw_2d_obs, origin='lower', extent=[min_x, max_x, min_y, max_y], cmap='gray_r')
        ax1.plot(0.031, 0.063, 'ro', markersize=8, label='起点 (0.03, 0.06)')
        ax1.plot(-1.10, 9.60, 'r*', markersize=11, label='终点 (-1.10, 9.60)')
        ax1.set_title("① 原始 3D 点云截面投影\n(走廊含跟拍人影噪点、桌沿断续不闭合)", fontsize=11.5, fontweight='bold')
        ax1.set_xlabel("X (m)"); ax1.set_ylabel("Y (m)"); ax1.legend(loc='lower left', fontsize=9); ax1.grid(True, ls=':', alpha=0.4)

        ax2 = axes[1]
        ax2.imshow(grid, origin='lower', extent=[min_x, max_x, min_y, max_y], cmap='gray', vmin=0, vmax=255)
        ax2.plot(0.031, 0.063, 'ro', markersize=8, label='起点 (0.03, 0.06)')
        ax2.plot(-1.10, 9.60, 'g*', markersize=11, label='终点 (-1.10, 9.60)')
        ax2.set_title("② 精修规整版黑白二值底图 (lab_map_1021.pgm)\n(对齐 rmuc2025 标准：墙体100%闭合、走廊0噪点)", fontsize=11.5, fontweight='bold', color='#1b5e20')
        ax2.set_xlabel("X (m)"); ax2.set_ylabel("Y (m)"); ax2.legend(loc='lower left', fontsize=9); ax2.grid(True, ls=':', alpha=0.4)

        ax3 = axes[2]
        ax3.imshow(rgb_3zone, origin='lower', extent=[min_x, max_x, min_y, max_y])
        ax3.plot(0.031, 0.063, 'ro', markersize=8, markeredgecolor='white')
        ax3.plot(-1.10, 9.60, 'r*', markersize=12, markeredgecolor='white')
        p_obs = mpatches.Patch(color='#0f081f', label='障碍物本体 (d = 0m)')
        p_forb = mpatches.Patch(color='#3894e0', label='禁止区 (0 < d ≤ 0.22m, 严禁碰撞)')
        p_squz = mpatches.Patch(color='#94edfa', label='挤压区 (0.22 < d ≤ 0.50m, 缓冲代价)')
        p_free = mpatches.Patch(facecolor='white', edgecolor='gray', label='通行区 (d > 0.50m, 安全自由通行)')
        ax3.legend(handles=[p_obs, p_forb, p_squz, p_free], loc='lower left', fontsize=8.5, framealpha=0.92)
        ax3.set_title("③ 自动膨胀生成三层导航区域 (Nav2 风格)\n【深蓝禁止区 + 青蓝挤压区 + 白色自由通行区】", fontsize=11.5, fontweight='bold', color='#0d47a1')
        ax3.set_xlabel("X (m)"); ax3.set_ylabel("Y (m)"); ax3.grid(True, ls=':', alpha=0.4)

        plt.tight_layout(rect=[0, 0, 1, 0.94])
        proc_fig_path = os.path.join(out_dir, 'map_4step_process.png')
        plt.savefig(proc_fig_path, dpi=160)
        plt.close()
        print(f"✅ 已生成精修规整版实验室底图与三层膨胀对比图: {proc_fig_path}")
    except Exception as e:
        print(f"[warn] 绘图跳过: {e}")

    print(f"✅ 已输出 rmuc2025 级纯净 2D 导航底图: {pgm_path} ({W}x{H}, res={res}m)")

if __name__ == '__main__':
    main()
