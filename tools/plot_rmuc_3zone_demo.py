#!/usr/bin/env -S python3 -s
"""
在 TDT-Nav-Kit 原版 rmuc2025.png (560x300, res=0.05m, 28m x 15m) 上：
1. 生成对标用户示例图的【禁止区(深蓝+洋红) + 挤压区(青色+粉红) + 通行区(纯白)】三层膨胀代价图
2. 叠加原版 YAstar 路径、DP 化简点、方形安全飞行走廊 (Square SFC) 与 Minimum Snap 轨迹
输出高清大图: /home/wgk/decision_planning/SCU-TDT-V1/maps/tdt_rmuc2025_3zone_result.png
"""
import os
import numpy as np
from scipy.ndimage import distance_transform_edt
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from PIL import Image

plt.rcParams['font.sans-serif'] = ['WenQuanYi Micro Hei', 'WenQuanYi Zen Hei', 'Droid Sans Fallback', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

def main():
    img_path = "/home/wgk/decision_planning/SCU-TDT-V1/images/rmuc2025.png"
    out_dir = "/home/wgk/decision_planning/SCU-TDT-V1/maps"
    os.makedirs(out_dir, exist_ok=True)

    im = Image.open(img_path).convert('L')
    # 注意 OpenCV / 图像坐标中第 0 行在顶部；为了与 C++ orig_demo_main.cpp 的世界坐标 (x=col*0.05, y=row*0.05) 对齐
    raw = np.array(im)
    H, W = raw.shape
    res = 0.05

    free_mask = (raw > 127)
    dist_m = distance_transform_edt(free_mask) * res

    # 构造对标 Nav2 RViz 的三层膨胀 RGB 图
    rgb = np.zeros((H, W, 3), dtype=np.float32)
    rgb[~free_mask] = [0.06, 0.05, 0.12]                      # 障碍物本体: 深黑紫
    forbidden = (dist_m > 0.0) & (dist_m <= 0.25)             # 禁止区 (0 < d <= 0.25m)
    rgb[forbidden] = [0.16, 0.48, 0.85]                       # 深蓝
    forb_edge = (dist_m > 0.21) & (dist_m <= 0.26)
    rgb[forb_edge] = [0.85, 0.12, 0.52]                       # 洋红边界线
    squeeze = (dist_m > 0.26) & (dist_m <= 0.55)              # 挤压区 (0.25m < d <= 0.55m)
    rgb[squeeze] = [0.55, 0.94, 0.98]                         # 青色
    sqz_edge = (dist_m > 0.51) & (dist_m <= 0.56)
    rgb[sqz_edge] = [0.94, 0.55, 0.62]                        # 粉红过渡线
    free_zone = dist_m > 0.56                                 # 通行区 (d > 0.55m)
    rgb[free_zone] = [0.99, 0.99, 1.00]                       # 纯白

    extent = [0, W * res, H * res, 0]  # 与原版 OpenCV 图像坐标系一致 (左上角为原点, X向右0..28m, Y向下0..15m)

    fig, axes = plt.subplots(1, 2, figsize=(20, 6.2), dpi=160)
    fig.suptitle("【TDT-Nav-Kit 原版 RMUC 2025 赛场地图】纯净黑白底图 + 三层膨胀区域 (禁止区/挤压区/通行区) + 方形走廊 Minimum Snap",
                 fontsize=14, fontweight='bold', y=0.98)

    # 左图：三层膨胀代价图 + YAstar 搜索 + DP 化简点
    ax1 = axes[0]
    ax1.imshow(rgb, extent=extent)
    if os.path.exists("/tmp/tdt_rmuc2025_eval/astar_raw.txt"):
        raw_pts = np.loadtxt("/tmp/tdt_rmuc2025_eval/astar_raw.txt")
        if raw_pts.ndim == 2 and len(raw_pts) > 1:
            ax1.plot(raw_pts[:, 0], raw_pts[:, 1], '--', color='#d62728', linewidth=1.8, label='前端 YAstar 人工势场搜索路径')
    if os.path.exists("/tmp/tdt_rmuc2025_eval/waypoints.txt"):
        wp_pts = np.loadtxt("/tmp/tdt_rmuc2025_eval/waypoints.txt")
        if wp_pts.ndim == 2 and len(wp_pts) > 1:
            ax1.plot(wp_pts[:, 0], wp_pts[:, 1], '-o', color='#2ca02c', linewidth=2.2, markersize=5, label='DP 拓扑化简关键路点 (绿线)')
            ax1.scatter([wp_pts[0, 0]], [wp_pts[0, 1]], c='red', s=110, zorder=8, label=f'起点 ({wp_pts[0,0]:.1f}, {wp_pts[0,1]:.1f})')
            ax1.scatter([wp_pts[-1, 0]], [wp_pts[-1, 1]], c='gold', edgecolors='black', marker='*', s=180, zorder=8,
                        label=f'终点 ({wp_pts[-1,0]:.1f}, {wp_pts[-1,1]:.1f})')

    p_obs = patches.Patch(color=[0.06, 0.05, 0.12], label='赛场墙体障碍物 (d = 0m)')
    p_forb = patches.Patch(color=[0.16, 0.48, 0.85], label='禁止区 (0 < d ≤ 0.25m, 严禁碰撞)')
    p_sqz = patches.Patch(color=[0.55, 0.94, 0.98], label='挤压区 (0.25 < d ≤ 0.55m, 缓冲代价)')
    p_free = patches.Patch(facecolor='white', edgecolor='gray', label='通行区 (d > 0.55m, 自由通行)')
    handles, _ = ax1.get_legend_handles_labels()
    ax1.legend(handles=[p_obs, p_forb, p_sqz, p_free] + handles, loc='lower left', fontsize=8.2, framealpha=0.92)
    ax1.set_title("① RMUC 2025 赛场三层膨胀区域图 (Nav2 RViz 配色) + 前端 YAstar 搜索", fontsize=11.5, fontweight='bold')
    ax1.set_xlabel("赛场 X (m)")
    ax1.set_ylabel("赛场 Y (m)")
    ax1.grid(True, linestyle=':', alpha=0.35)

    # 右图：原版黑白底图 + 蓝色方形安全走廊 (Square SFC) + 绿色化简折线 + 红色 Minimum Snap 平滑轨迹
    ax2 = axes[1]
    bg_rgb = np.zeros((H, W, 3), dtype=np.float32)
    bg_rgb[~free_mask] = [0.08, 0.09, 0.11]
    bg_rgb[free_mask] = [0.99, 0.99, 1.00]
    ax2.imshow(bg_rgb, extent=extent)

    if os.path.exists("/tmp/tdt_rmuc2025_eval/corridors.txt"):
        corrs = np.loadtxt("/tmp/tdt_rmuc2025_eval/corridors.txt")
        if corrs.ndim == 1 and corrs.size == 4:
            corrs = corrs.reshape(1, 4)
        for i, c in enumerate(corrs):
            x0, y0, x1, y1 = min(c[0], c[2]), min(c[1], c[3]), max(c[0], c[2]), max(c[1], c[3])
            rect = patches.Rectangle((x0, y0), x1 - x0, y1 - y0,
                                     linewidth=1.3, edgecolor='#1f77b4', facecolor='#6baed6', alpha=0.28,
                                     label='方形安全飞行走廊 (Square SFC)' if i == 0 else None)
            ax2.add_patch(rect)

    if os.path.exists("/tmp/tdt_rmuc2025_eval/waypoints.txt"):
        wp_pts = np.loadtxt("/tmp/tdt_rmuc2025_eval/waypoints.txt")
        if wp_pts.ndim == 2 and len(wp_pts) > 1:
            ax2.plot(wp_pts[:, 0], wp_pts[:, 1], '-o', color='#2ca02c', linewidth=1.8, markersize=4.5, alpha=0.85,
                     label='DP 化简折线 (绿线)')

    if os.path.exists("/tmp/tdt_rmuc2025_eval/smooth_traj.txt"):
        sm_pts = np.loadtxt("/tmp/tdt_rmuc2025_eval/smooth_traj.txt")
        if sm_pts.ndim == 2 and len(sm_pts) > 1:
            ax2.plot(sm_pts[:, 0], sm_pts[:, 1], '-', color='#e31a1c', linewidth=2.8, zorder=7,
                     label='Minimum Snap 五阶平滑轨迹 (红线)')
            ax2.scatter([sm_pts[0, 0]], [sm_pts[0, 1]], c='red', s=110, zorder=8, label='起点 Start')
            ax2.scatter([sm_pts[-1, 0]], [sm_pts[-1, 1]], c='gold', edgecolors='black', marker='*', s=180, zorder=8, label='终点 Goal')

    ax2.set_title("② TDT-Nav-Kit 原版演示复现：蓝色方形安全走廊 (SFC) + 红色 Minimum Snap 轨迹", fontsize=11.5, fontweight='bold')
    ax2.set_xlabel("赛场 X (m)")
    ax2.set_ylabel("赛场 Y (m)")
    ax2.legend(loc='lower left', fontsize=8.8, framealpha=0.92)
    ax2.grid(True, linestyle=':', alpha=0.35)

    plt.tight_layout(rect=[0, 0, 1, 0.94])
    out_file = os.path.join(out_dir, "tdt_rmuc2025_3zone_result.png")
    plt.savefig(out_file, dpi=160)
    plt.close(fig)
    print(f"✅ 已生成 RMUC 2025 原版三层膨胀+方形走廊轨迹对比图: {out_file}")

if __name__ == '__main__':
    main()
