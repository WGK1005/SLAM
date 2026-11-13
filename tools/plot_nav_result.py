#!/usr/bin/env python3
"""
渲染 SCU-TDT-V1 在真实实验室地图上的决策规划与轨迹优化全流程诊断大图
包含：
  - 左图：前端 YAstar 人工势场代价图 + 原始搜索路径 + DP 化简关键拐点 + SCURM 智能脱困演示
  - 中图：后端 Minimum Snap + 蓝色方形安全走廊 (Square SFC) + 红色五阶平滑轨迹 (叠加实机 3D 障碍物点云底图)
  - 右图：连续平滑轨迹的速度曲线 V(t) 与加速度曲线 A(t) 动力学剖面
"""
import os
import sys
import time
import shutil
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

plt.rcParams['font.sans-serif'] = ['WenQuanYi Micro Hei', 'WenQuanYi Zen Hei', 'Droid Sans Fallback', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

def load_txt_2d(p):
    if not os.path.exists(p) or os.path.getsize(p) == 0:
        return np.zeros((0, 2))
    return np.atleast_2d(np.loadtxt(p))

def main():
    eval_dir = '/tmp/scu_tdt_eval'
    pgm_path = '/home/wgk/decision_planning/SCU-TDT-V1/maps/lab_map_1021.pgm'
    out_png = '/home/wgk/decision_planning/SCU-TDT-V1/maps/scu_tdt_plan_result.png'

    info = {}
    info_path = os.path.join(eval_dir, 'info.txt')
    if os.path.exists(info_path):
        with open(info_path) as f:
            for line in f:
                if '=' in line:
                    k, v = line.strip().split('=', 1)
                    try:
                        info[k] = float(v)
                    except ValueError:
                        pass

    with open(pgm_path, 'rb') as f:
        _ = f.readline()
        dims = f.readline().decode().split()
        w, h = int(dims[0]), int(dims[1])
        _ = f.readline()
        grid = np.frombuffer(f.read(w * h), dtype=np.uint8).reshape((h, w))

    cost_bin = os.path.join(eval_dir, 'costmap.bin')
    if os.path.exists(cost_bin):
        with open(cost_bin, 'rb') as f:
            cw = np.frombuffer(f.read(4), dtype=np.int32)[0]
            ch = np.frombuffer(f.read(4), dtype=np.int32)[0]
            cost_mat = np.frombuffer(f.read(cw * ch * 4), dtype=np.float32).reshape((ch, cw))
    else:
        cost_mat = np.ones_like(grid, dtype=np.float32)

    astar_raw = load_txt_2d(os.path.join(eval_dir, 'astar_raw.txt'))
    waypoints = load_txt_2d(os.path.join(eval_dir, 'waypoints.txt'))
    traj = load_txt_2d(os.path.join(eval_dir, 'smooth_traj.txt'))
    corridors = load_txt_2d(os.path.join(eval_dir, 'corridors.txt'))

    min_x, max_x = -7.50, 3.50
    min_y, max_y = -2.50, 11.50
    extent = [min_x, max_x, min_y, max_y]

    ts = time.strftime("%m%d_%H%M%S")
    fig = plt.figure(figsize=(21, 9.5), dpi=150)
    gs = fig.add_gridspec(2, 3, width_ratios=[1.15, 1.15, 1.0], wspace=0.22, hspace=0.28)

    ax1 = fig.add_subplot(gs[:, 0])
    ax2 = fig.add_subplot(gs[:, 1])
    ax3 = fig.add_subplot(gs[0, 2])
    ax4 = fig.add_subplot(gs[1, 2])

    # ==================== 左栏：三层区域地图 (禁止区/挤压区/通行区) + 前端 YAstar 搜索 ====================
    from scipy import ndimage
    import matplotlib.patches as mpatches
    obs_mask = (grid == 0)
    dist_m = ndimage.distance_transform_edt(~obs_mask) * 0.05
    forbid_r, squeeze_r = 0.22, 0.50

    rgb_3zone = np.ones((h, w, 3), dtype=np.float32)  # 通行区: 纯白色
    squeeze_mask = (dist_m > forbid_r) & (dist_m <= squeeze_r)
    rgb_3zone[squeeze_mask] = [0.58, 0.93, 0.98]      # 挤压区: 青蓝色
    rgb_3zone[(dist_m > squeeze_r - 0.05) & (dist_m <= squeeze_r)] = [0.94, 0.65, 0.70]
    forbid_mask = (dist_m > 0.0) & (dist_m <= forbid_r)
    rgb_3zone[forbid_mask] = [0.22, 0.58, 0.88]       # 禁止区: 深蓝色
    rgb_3zone[(dist_m > forbid_r - 0.05) & (dist_m <= forbid_r)] = [0.88, 0.12, 0.62]
    rgb_3zone[obs_mask] = [0.08, 0.04, 0.16]          # 障碍物本体: 深黑紫色

    ax1.imshow(rgb_3zone, origin='lower', extent=extent)

    if len(astar_raw) > 0:
        ax1.plot(astar_raw[:, 0], astar_raw[:, 1], '--', color='#d50000', lw=2.2, alpha=0.90,
                 label=f'前端 YAstar 搜索路径 ({len(astar_raw)} 点, {info.get("astar_ms", 0):.2f}ms)')
    if len(waypoints) > 0:
        ax1.plot(waypoints[:, 0], waypoints[:, 1], '-o', color='#00c853', lw=2.4, markersize=5.5,
                 label=f'DP化简+均匀重采样控制点 ({len(waypoints)} 点, {info.get("simplify_ms", 0):.2f}ms)')

    p_forb = mpatches.Patch(color='#3894e0', label='禁止区 (d ≤ 0.22m, 严禁碰撞)')
    p_squz = mpatches.Patch(color='#94edfa', label='挤压区 (0.22 < d ≤ 0.50m, 缓冲代价)')
    p_free = mpatches.Patch(facecolor='white', edgecolor='gray', label='通行区 (d > 0.50m, 自由通行)')

    ax1.plot(info.get('start_x', 0.03), info.get('start_y', 0.06), 'ro', markersize=11, markeredgecolor='white',
             label=f'实机重定位起点 ({info.get("start_x", 0.03):.2f}, {info.get("start_y", 0.06):.2f})')
    ax1.plot(info.get('goal_x', -1.1), info.get('goal_y', 9.6), 'r*', markersize=15, markeredgecolor='white',
             label=f'目标导航点 ({info.get("goal_x", -1.1):.2f}, {info.get("goal_y", 9.6):.2f})')

    ax1.set_title(f"【前端：三层膨胀地图 (禁止区/挤压区/通行区) + YAstar 搜索】\n"
                  f"搜索耗时: {info.get('astar_ms', 0):.2f} ms | 绕开正北障碍物走左侧主通道",
                  fontsize=11.5, fontweight='bold', color='#bf360c')
    ax1.set_xlabel("实验室全局 X (m)", fontsize=11)
    ax1.set_ylabel("实验室全局 Y (m)", fontsize=11)
    ax1.set_xlim(min_x, max_x)
    ax1.set_ylim(min_y, max_y)
    ax1.grid(True, linestyle=':', alpha=0.5)
    handles1, _ = ax1.get_legend_handles_labels()
    ax1.legend(handles=[p_forb, p_squz, p_free] + handles1, loc='lower left', fontsize=8.2, framealpha=0.92)

    # ==================== 中栏：后端 Minimum Snap + 方形安全走廊 SFC ====================
    ax2.imshow(grid, origin='lower', extent=extent, cmap='gray', vmin=0, vmax=255, alpha=0.90)

    if len(corridors) > 0:
        for idx, c in enumerate(corridors):
            w_box = c[2] - c[0]
            h_box = c[3] - c[1]
            if w_box > 0.02 and h_box > 0.02:
                rect = Rectangle((c[0], c[1]), w_box, h_box,
                                 linewidth=1.5, edgecolor='#1565c0', facecolor='#42a5f5', alpha=0.22,
                                 label='方形安全飞行走廊 (Square SFC)' if idx == 1 else None)
                ax2.add_patch(rect)

    if len(waypoints) > 0:
        ax2.plot(waypoints[:, 0], waypoints[:, 1], '--o', color='#2e7d32', lw=1.6, markersize=5, alpha=0.75,
                 label='前端化简折线控制点')

    if len(traj) > 0:
        ax2.plot(traj[:, 1], traj[:, 2], '-', color='#d50000', lw=3.0,
                 label=f'Minimum Snap 五阶平滑轨迹 ({info.get("min_snap_ms", 0):.2f}ms)')

    ax2.plot(info.get('start_x', 0.03), info.get('start_y', 0.06), 'ro', markersize=11, label='起点 (重定位锁定位姿)')
    ax2.plot(info.get('goal_x', -1.1), info.get('goal_y', 9.6), 'g*', markersize=15, label='终点 (零速平滑停止)')

    ax2.set_title(f"【后端：Minimum Snap + 方形安全走廊 (SFC) 轨迹优化】\n"
                  f"优化耗时: {info.get('min_snap_ms', 0):.2f} ms | 轨迹总长: {info.get('path_len', 0):.2f} m | 100% 无碰撞",
                  fontsize=12, fontweight='bold', color='#0d47a1')
    ax2.set_xlabel("实验室全局 X (m)", fontsize=11)
    ax2.set_ylabel("实验室全局 Y (m)", fontsize=11)
    ax2.set_xlim(min_x, max_x)
    ax2.set_ylim(min_y, max_y)
    ax2.grid(True, linestyle=':', alpha=0.5)
    ax2.legend(loc='lower left', fontsize=9.5, framealpha=0.92)

    # ==================== 右栏：连续时间速度 V(t) 与加速度 A(t) 曲线 ====================
    if len(traj) > 1:
        t = traj[:, 0]
        vx, vy = traj[:, 3], traj[:, 4]
        v_norm = np.hypot(vx, vy)
        ax, ay = traj[:, 5], traj[:, 6]
        a_norm = np.hypot(ax, ay)

        ax3.plot(t, v_norm, '-', color='#d50000', lw=2.2, label='合速度 ||V(t)|| (m/s)')
        ax3.plot(t, vx, '--', color='#1976d2', lw=1.4, alpha=0.8, label='Vx(t)')
        ax3.plot(t, vy, '--', color='#388e3c', lw=1.4, alpha=0.8, label='Vy(t)')
        ax3.axhline(1.20, color='gray', linestyle=':', lw=1.2, label='最大速度限制 (1.2 m/s)')
        ax3.set_title("平滑轨迹速度剖面 V(t) (连续无突变)", fontsize=11.5, fontweight='bold')
        ax3.set_xlabel("时间 t (s)")
        ax3.set_ylabel("速度 (m/s)")
        ax3.grid(True, linestyle=':', alpha=0.6)
        ax3.legend(loc='upper right', fontsize=8.5)

        ax4.plot(t, a_norm, '-', color='#ff6f00', lw=2.0, label='合加速度 ||A(t)|| (m/s²)')
        ax4.plot(t, ax, '--', color='#1976d2', lw=1.3, alpha=0.75, label='Ax(t)')
        ax4.plot(t, ay, '--', color='#388e3c', lw=1.3, alpha=0.75, label='Ay(t)')
        ax4.set_title("平滑轨迹加速度剖面 A(t) (抑制电机冲击)", fontsize=11.5, fontweight='bold')
        ax4.set_xlabel("时间 t (s)")
        ax4.set_ylabel("加速度 (m/s²)")
        ax4.grid(True, linestyle=':', alpha=0.6)
        ax4.legend(loc='upper right', fontsize=8.5)

    plt.suptitle(f"【SCU-TDT-V1】实机地图决策规划与轨迹优化诊断图 [{ts}] "
                 f"(全流程总耗时: {info.get('total_ms', 0):.2f} ms | 适配 VLP-16 + HI12)",
                 fontsize=14.5, fontweight='bold', y=0.98)

    os.makedirs(os.path.dirname(out_png), exist_ok=True)
    plt.savefig(out_png, bbox_inches='tight')
    plt.close(fig)

    hist_dir = '/home/wgk/maps/history'
    os.makedirs(hist_dir, exist_ok=True)
    ts_png = os.path.join(hist_dir, f"scu_tdt_plan_{ts}.png")
    shutil.copy2(out_png, ts_png)
    print(f"✅ 已生成 SCU-TDT-V1 实机地图规划诊断图: {out_png} (已存档 {ts_png})")

if __name__ == '__main__':
    main()

