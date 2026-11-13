#!/usr/bin/env -S python3 -s
"""
绘制宇树 Go2 (VLP-16 + HI12) 在 ROBOCON 2027 障碍赛 14m x 6m 场地的：
  - 上图：全赛场 8 大障碍完整越障平滑轨迹 (Minimum Snap + 速度方向约束)
  - 下图：梅花桩 (Plum Piles) 1:1 局部放大 —— 机身质心 S 微摆轨迹 + Go2 四足 (FL/FR/RL/RR) 落足点序列
输出：
  /home/wgk/decision_planning/SCU-TDT-V1/maps/rc_go2_obstacle_traversal.png
"""
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle

plt.rcParams['font.sans-serif'] = ['WenQuanYi Micro Hei', 'WenQuanYi Zen Hei', 'Droid Sans Fallback', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False


def load_xy(path):
    if not os.path.exists(path) or os.path.getsize(path) == 0:
        return np.zeros((0, 2))
    return np.atleast_2d(np.loadtxt(path))


def draw_arena(ax):
    ax.set_facecolor("#fdf6b2")
    ax.set_xlim(0, 14.0)
    ax.set_ylim(0, 6.0)
    ax.set_aspect("equal")

    # 1. 砖木坑 L形
    ax.add_patch(Rectangle((0.62, 3.39), 1.0, 2.0, fc="#c97a57", ec="#2b6cb0", lw=1.5))
    ax.add_patch(Rectangle((1.62, 4.39), 1.0, 1.0, fc="#8b5a2b", ec="#2b6cb0", lw=1.5))
    ax.text(1.62, 5.50, "1.砖木坑(L形)", ha="center", fontsize=9, weight="bold")

    # 2. 单木桥
    ax.add_patch(Rectangle((3.98, 4.68), 3.0, 0.40, fc="#68d391", ec="#276749", lw=1.5))
    ax.text(5.48, 5.20, "2.单木桥(3.0m×0.4m)", ha="center", fontsize=9, weight="bold")

    # 3. 高墙
    ax.add_patch(Rectangle((8.46, 4.48), 0.08, 0.80, fc="#2f855a", ec="black", lw=1.5))
    ax.text(8.50, 5.40, "3.高墙(h=0.3m)", ha="center", fontsize=9, weight="bold")

    # 4. 梅花桩
    ax.add_patch(Rectangle((9.90, 4.48), 0.80, 0.80, fc="#68d391", ec="#276749", lw=1.5))
    ax.add_patch(Rectangle((12.60, 4.48), 0.80, 0.80, fc="#68d391", ec="#276749", lw=1.5))
    for k in range(3):
        ax.add_patch(Rectangle((10.80 + k * 0.6, 4.98), 0.20, 0.20, fc="#48bb78", ec="#22543d", lw=1.2))
        ax.add_patch(Rectangle((11.10 + k * 0.6, 4.58), 0.20, 0.20, fc="#48bb78", ec="#22543d", lw=1.2))
    ax.text(11.65, 5.42, "4.梅花桩(6桩交错)", ha="center", fontsize=9, weight="bold")

    # 5. 沟壑组合
    ax.add_patch(Rectangle((0.70, 0.60), 0.80, 0.80, fc="#f6e05e", ec="red", lw=1.5, ls="--"))
    ax.add_patch(Rectangle((1.86, 0.60), 0.80, 0.80, fc="#68d391", ec="#276749", lw=1.5))
    ax.add_patch(Rectangle((2.66, 0.60), 0.64, 0.80, fc="#9ae6b4", ec="#276749", lw=1.2))
    ax.add_patch(Rectangle((0.70, 1.40), 0.80, 0.99, fc="#9ae6b4", ec="#276749", lw=1.2))
    ax.text(1.90, 2.52, "5.沟壑组合(台阶+36cm沟+20°坡)", ha="center", fontsize=9, weight="bold")

    # 6. 轮胎阵 2x4
    for i in range(4):
        for j in range(2):
            cx = 5.70 + (i - 1.5) * 0.55
            cy = 1.15 + (j - 0.5) * 0.55
            ax.add_patch(Circle((cx, cy), 0.275, fc="#1a202c", ec="black"))
            ax.add_patch(Circle((cx, cy), 0.165, fc="#fdf6b2", ec="black"))
    ax.text(5.70, 1.82, "6.轮胎阵(2×4)", ha="center", fontsize=9, weight="bold")

    # 7. 限高框架
    ax.add_patch(Rectangle((8.06, 0.75), 0.80, 0.80, fill=False, ec="#276749", lw=2.5))
    ax.text(8.46, 1.68, "7.限高架(内高290mm)", ha="center", fontsize=9, weight="bold")

    # 8. 绕桩
    poles = [(10.25, 1.05), (11.15, 1.05), (12.05, 1.05), (12.95, 1.05), (12.95, 1.95), (12.95, 2.85)]
    for px, py in poles:
        ax.add_patch(Circle((px, py), 0.05, fc="#276749", ec="black", zorder=5))
    for mx, my in [(9.95, 1.05), (13.25, 0.75), (12.95, 3.15)]:
        ax.add_patch(Circle((mx, my), 0.15, fill=False, ec="red", lw=1.5, ls="--", zorder=4))
    ax.text(11.6, 2.15, "8.L形绕桩(6桩+3必达区)", ha="center", fontsize=9, weight="bold")

    # 启动区1
    ax.add_patch(Rectangle((6.60, 2.60), 0.80, 0.80, fill=False, ec="red", lw=2.0, ls="--"))
    ax.text(7.00, 3.00, "启动区1", ha="center", va="center", color="red", fontsize=9, weight="bold")


def main():
    d = "/tmp/rc_go2_plan"
    s1 = load_xy(os.path.join(d, "seg1_approach.txt"))
    s2 = load_xy(os.path.join(d, "seg2_plum.txt"))
    s3 = load_xy(os.path.join(d, "seg3_slalom.txt"))
    s4 = load_xy(os.path.join(d, "seg4_frame_tire.txt"))
    s5 = load_xy(os.path.join(d, "seg5_west_loop.txt"))

    fig = plt.figure(figsize=(15, 11.5))
    gs = fig.add_gridspec(2, 1, height_ratios=[1.25, 1.0], hspace=0.26)

    # ==================== 上图：全场 8 障碍越障轨迹总览 ====================
    ax1 = fig.add_subplot(gs[0])
    draw_arena(ax1)

    if len(s1): ax1.plot(s1[:, 0], s1[:, 1], "-", color="#2b6cb0", lw=2.5, label="[阶段1] 启动区1→梅花桩对准 (YAstar+SFC)")
    if len(s2): ax1.plot(s2[:, 0], s2[:, 1], "-", color="#d53f8c", lw=2.8, label="[阶段2] 梅花桩质心微摆穿越")
    if len(s3): ax1.plot(s3[:, 0], s3[:, 1], "-", color="#dd6b20", lw=2.5, label="[阶段3] L形6桩S弯绕桩 (含3必达区)")
    if len(s4): ax1.plot(s4[:, 0], s4[:, 1], "-", color="#805ad5", lw=2.5, label="[阶段4] 限高架(匍匐h=0.22m) + 轮胎阵")
    if len(s5): ax1.plot(s5[:, 0], s5[:, 1], "-", color="#e53e3e", lw=2.4, label="[阶段5] 沟壑+砖木坑+单木桥+高墙→回启动区1")

    ax1.set_xticks(np.arange(0, 14.1, 1.0))
    ax1.set_yticks(np.arange(0, 6.1, 1.0))
    ax1.grid(alpha=0.25, ls=":")
    ax1.legend(loc="upper center", bbox_to_anchor=(0.5, 1.12), ncol=3, fontsize=9, framealpha=0.92)
    ax1.set_title("ROBOCON 2027 障碍赛 (14m×6m) —— 宇树 Go2 + VLP-16 全场 8 障碍连续越障规划轨迹", fontsize=12, weight="bold", pad=44)

    # ==================== 下图：梅花桩 1:1 放大与 Go2 四足落足点规划 ====================
    ax2 = fig.add_subplot(gs[1])
    ax2.set_facecolor("#f7fafc")
    ax2.set_xlim(9.65, 13.55)
    ax2.set_ylim(4.32, 5.44)
    ax2.set_aspect("equal")

    # 两侧 800x800mm 平台
    ax2.add_patch(Rectangle((9.90, 4.48), 0.80, 0.80, fc="#c6f6d5", ec="#276749", lw=2.0, label="两侧 800×800×100mm 平台"))
    ax2.add_patch(Rectangle((12.60, 4.48), 0.80, 0.80, fc="#c6f6d5", ec="#276749", lw=2.0))
    ax2.text(10.30, 5.32, "西平台 (800×800mm)", ha="center", fontsize=9.5, weight="bold", color="#22543d")
    ax2.text(13.00, 5.32, "东平台 (800×800mm)", ha="center", fontsize=9.5, weight="bold", color="#22543d")

    # 6 个 200x200mm 梅花桩方块
    blocks = [
        (10.90, 5.08, "#1 上排"), (11.20, 4.68, "#2 下排"),
        (11.50, 5.08, "#3 上排"), (11.80, 4.68, "#4 下排"),
        (12.10, 5.08, "#5 上排"), (12.40, 4.68, "#6 下排"),
    ]
    for idx, (bx, by, lbl) in enumerate(blocks):
        ax2.add_patch(Rectangle((bx - 0.10, by - 0.10), 0.20, 0.20, fc="#9ae6b4", ec="#22543d", lw=2.0))
        ax2.text(bx, by + (0.135 if by > 4.88 else -0.155), f"桩{lbl}\n({bx:.2f},{by:.2f})",
                 ha="center", va="center", fontsize=8, color="#22543d", weight="bold")

    # 绘制质心轨迹
    if len(s2):
        ax2.plot(s2[:, 0], s2[:, 1], "-", color="#d53f8c", lw=3.0, zorder=4, label="Go2 机身质心 MinimumSnap 微摆轨迹")

    # 绘制 Go2 机身包络示意 (在 x=11.50m 处)
    ax2.add_patch(Rectangle((11.50 - 0.188, 4.88 - 0.095), 0.376, 0.190,
                            fc="#4a5568", ec="#ed8936", lw=2.0, alpha=0.35, zorder=3, label="Go2 躯干包络 (0.376×0.19m) + VLP-16"))
    ax2.add_patch(Circle((11.50 + 0.08, 4.88), 0.052, fc="#3182ce", ec="white", alpha=0.7, zorder=5))

    # 读取并绘制四足落足点
    fs_file = os.path.join(d, "plum_footsteps.txt")
    leg_colors = {"FL": "#e53e3e", "FR": "#3182ce", "RL": "#dd6b20", "RR": "#805ad5"}
    leg_names = {"FL": "左前足 FL (踩上排)", "FR": "右前足 FR (踩下排)",
                 "RL": "左后足 RL (踩上排)", "RR": "右后足 RR (踩下排)"}
    seen_leg = set()
    if os.path.exists(fs_file):
        with open(fs_file) as f:
            for line in f:
                parts = line.strip().split()
                if len(parts) < 6:
                    continue
                sid = int(parts[0])
                leg = parts[1]
                fx, fy = float(parts[2]), float(parts[3])
                lbl = leg_names[leg] if leg not in seen_leg else None
                seen_leg.add(leg)
                # 球形足端直径 44mm (半径 0.022m)
                ax2.add_patch(Circle((fx, fy), 0.024, fc=leg_colors[leg], ec="white", lw=1.0, zorder=6, label=lbl))
                ax2.text(fx, fy, str(sid), ha="center", va="center", color="white", fontsize=6.5, weight="bold", zorder=7)

    ax2.axhline(4.88, color="gray", ls="--", lw=0.8, alpha=0.5)
    ax2.grid(alpha=0.3, ls=":")
    ax2.legend(loc="upper right", fontsize=8.5, ncol=2, framealpha=0.92)
    ax2.set_title("梅花桩 (Plum Piles) 1:1 局部放大 —— 宇树 Go2 质心轨迹 + 四足 20 步落足点规划 (编号0~19为迈步顺序，6桩100%接触)",
                  fontsize=11.5, weight="bold")
    ax2.set_xlabel("场地 X 坐标 (m)", fontsize=10)
    ax2.set_ylabel("场地 Y 坐标 (m)", fontsize=10)

    out_path = "/home/wgk/decision_planning/SCU-TDT-V1/maps/rc_go2_obstacle_traversal.png"
    fig.savefig(out_path, dpi=130, bbox_inches="tight")
    print(f"✅ 已生成宇树 Go2 障碍赛越障与梅花桩落足点规划大图: {out_path}")


if __name__ == "__main__":
    main()
