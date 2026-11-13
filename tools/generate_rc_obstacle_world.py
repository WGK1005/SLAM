#!/usr/bin/env -S python3 -s
"""
生成 2027 足式机器人挑战赛 (ROBOCON) 障碍赛场地的 1:1 Gazebo 世界 + 2D 导航栅格图

依据：《2027年足式机器人挑战赛-V2.0》
  - 3.1 节：场地 14000mm x 6000mm，8 个障碍物，障碍物间距 >= 800mm，距边界 >= 600mm，
           启动区1 在场地中心，启动区2 在沟壑组合靠近斜坡的平台上
  - 表 1  ：各障碍物形状尺寸（含修订：梅花桩方块 200x200x100，两侧平台 800x800x100）
  - 图 1-2：参考俯视布局（规则注明：具体排列顺序与安装位置赛前公布）

尺寸全部取自表 1；位置按图 1-2 像素实测（约 70.5 px/m），并修正到满足
"障碍物间距 >= 0.8m、离边界 >= 0.6m" 的规则约束。

坐标系（与 SCU-TDT-V1 规划器一致）：
  原点 = 场地左下角（俯视图左下），X 沿 14m 长边向右，Y 沿 6m 短边向上，Z 向上，单位 m。
  PGM 第 r 行对应 y = r * res（不翻转），与 generate_rmuc_gazebo_world.py 约定相同。

输出：
  worlds/rc_obstacle_2027.world
  maps/rc_obstacle_2027.pgm / .yaml
  maps/rc_obstacle_2027_preview.png

用法：
  python3 tools/generate_rc_obstacle_world.py            # 无围挡（规则未规定障碍赛围墙）
  python3 tools/generate_rc_obstacle_world.py --fence 0.6 # 加 0.6m 高围挡，便于激光定位测试
"""
import argparse
import math
import os

import numpy as np

ROOT = "/home/wgk/decision_planning/SCU-TDT-V1"
FIELD_L, FIELD_W = 14.0, 6.0

# 颜色 (ambient/diffuse RGBA)
GREEN = "0.45 0.75 0.30 1"
DARK_GREEN = "0.20 0.55 0.20 1"
YELLOW = "0.95 0.85 0.15 1"
RED = "0.90 0.10 0.10 1"
GREY = "0.55 0.55 0.58 1"
BLACK = "0.06 0.06 0.06 1"
BRICK = "0.65 0.30 0.20 1"
BARK = "0.40 0.25 0.12 1"
BLUE = "0.10 0.20 0.85 1"

# 每个元素: dict(name, kind='box'|'cyl', pose(x,y,z,roll,pitch,yaw), size, color, collide, occ)
#  occ=True 表示写入 2D 导航栅格为障碍
ELEMS = []
OBSTACLES = {}  # 障碍名 -> 外接框 (x0,y0,x1,y1)，用于校核间距
NAV_FILL = []   # 导航栅格中整块填实的矩形 (x0,y0,x1,y1)：规划器只负责"绕到障碍入口"，不从障碍内部穿行


def box(name, cx, cy, cz, sx, sy, sz, color, collide=True, occ=True, roll=0.0, yaw=0.0, group=None):
    ELEMS.append(dict(name=name, kind="box", pose=(cx, cy, cz, roll, 0.0, yaw),
                      size=(sx, sy, sz), color=color, collide=collide, occ=occ))
    if group and occ:
        _grow(group, cx - sx / 2, cy - sy / 2, cx + sx / 2, cy + sy / 2)


def cyl(name, cx, cy, cz, r, h, color, collide=True, occ=True, group=None):
    ELEMS.append(dict(name=name, kind="cyl", pose=(cx, cy, cz, 0, 0, 0),
                      size=(r, h), color=color, collide=collide, occ=occ))
    if group and occ:
        _grow(group, cx - r, cy - r, cx + r, cy + r)


def _grow(group, x0, y0, x1, y1):
    b = OBSTACLES.get(group)
    OBSTACLES[group] = (x0, y0, x1, y1) if b is None else (
        min(b[0], x0), min(b[1], y0), max(b[2], x1), max(b[3], y1))


def floor_mark_square(name, cx, cy, size, color, w=0.03, z=0.0):
    """方框标记（仅可视化，无碰撞，不入栅格）；z 为所在表面高度"""
    h = 0.002
    for i, (dx, dy, sx, sy) in enumerate([
        (0, size / 2, size, w), (0, -size / 2, size, w),
        (size / 2, 0, w, size), (-size / 2, 0, w, size)]):
        box(f"{name}_{i}", cx + dx, cy + dy, z + h / 2 + 0.001, sx, sy, h, color, collide=False, occ=False)


def floor_mark_circle(name, cx, cy, d, color, n=16, w=0.02):
    r = d / 2
    seg = 2 * r * math.tan(math.pi / n)
    for i in range(n):
        a = 2 * math.pi * i / n
        box(f"{name}_{i}", cx + r * math.cos(a), cy + r * math.sin(a), 0.002, w, seg, 0.002,
            color, collide=False, occ=False, yaw=a)


def build_field(fence_h):
    # 黄色场地地面 (薄板，避免和 ground_plane z-fighting)
    box("field_floor", FIELD_L / 2, FIELD_W / 2, 0.0005, FIELD_L, FIELD_W, 0.001, YELLOW,
        collide=False, occ=False)
    # 蓝色边界线
    for i, (cx, cy, sx, sy) in enumerate([
        (FIELD_L / 2, 0.0, FIELD_L, 0.05), (FIELD_L / 2, FIELD_W, FIELD_L, 0.05),
        (0.0, FIELD_W / 2, 0.05, FIELD_W), (FIELD_L, FIELD_W / 2, 0.05, FIELD_W)]):
        box(f"border_line_{i}", cx, cy, 0.0015, sx, sy, 0.002, BLUE, collide=False, occ=False)
    if fence_h > 0:
        t = 0.05
        for i, (cx, cy, sx, sy) in enumerate([
            (FIELD_L / 2, -t / 2, FIELD_L + 2 * t, t), (FIELD_L / 2, FIELD_W + t / 2, FIELD_L + 2 * t, t),
            (-t / 2, FIELD_W / 2, t, FIELD_W), (FIELD_L + t / 2, FIELD_W / 2, t, FIELD_W)]):
            box(f"fence_{i}", cx, cy, fence_h / 2, sx, sy, fence_h, GREY, occ=False)


def build_obstacles():
    # ---------------- 1. 砖木坑 (L 形, 外包 2000x2000, 臂宽 1000, 护栏高 150) ----------------
    g = "brick_pit"
    x0, y1 = 0.62, 5.39          # 左上角 (图 1-2 实测)
    x1, y0 = x0 + 2.0, y1 - 2.0  # 外包右下
    rail_t, rail_h = 0.03, 0.15
    # 护栏 (沿 L 外轮廓 6 条边)；护栏中心线内缩 rail_t/2，使外包尺寸严格为 2000x2000
    i0, i1 = x0 + rail_t / 2, x1 - rail_t / 2
    j0, j1 = y0 + rail_t / 2, y1 - rail_t / 2
    arm = 1.0 - rail_t          # 内缩后的臂宽中心线
    pts = [(i0, j1), (i1, j1), (i1, j1 - arm), (i0 + arm, j1 - arm), (i0 + arm, j0), (i0, j0)]
    for i in range(6):
        (ax, ay), (bx, by) = pts[i], pts[(i + 1) % 6]
        cx, cy = (ax + bx) / 2, (ay + by) / 2
        sx = abs(bx - ax) + rail_t if ax != bx else rail_t
        sy = abs(by - ay) + rail_t if ay != by else rail_t
        box(f"{g}_rail_{i}", cx, cy, rail_h / 2, sx, sy, rail_h, GREEN, group=g)
    # 两区填充物 (碎砖 / 松树皮)，图中以对角线分区，此处近似为竖臂=碎砖、横臂=树皮
    box(f"{g}_bricks", x0 + 0.5, y0 + 1.0, 0.04, 1.0 - rail_t, 2.0 - rail_t, 0.08, BRICK, group=g)
    box(f"{g}_bark", x0 + 1.5, y1 - 0.5, 0.03, 1.0 - rail_t, 1.0 - rail_t, 0.06, BARK, group=g)

    # ---------------- 2. 单木桥 3000 x 400 x 100 ----------------
    box("single_plank", 5.48, 4.88, 0.05, 3.0, 0.4, 0.10, GREEN, group="single_plank")

    # ---------------- 3. 高墙 高300 宽800 厚80 (墙面法向沿 X) ----------------
    box("high_wall", 8.50, 4.88, 0.15, 0.08, 0.80, 0.30, DARK_GREEN, group="high_wall")

    # ---------------- 4. 梅花桩 ----------------
    # 方块 200x200x100，两侧平台 800x800x100；
    # 上排首块距左平台 100，下排首块距左平台 400；同排间距 400，两排间距 200；整体中心对称
    #   => 两平台间净距 = 0.1+0.2+0.4+0.2+0.4+0.2+0.4 = 1.9m，总长 0.8+1.9+0.8 = 3.5m
    g = "plum_piles"
    cy = 4.88
    lx0 = 9.90                       # 左平台左边缘 (右平台右边缘 = 13.40，离边界 0.60)
    box(f"{g}_platform_L", lx0 + 0.4, cy, 0.05, 0.8, 0.8, 0.10, GREEN, group=g)
    box(f"{g}_platform_R", lx0 + 0.8 + 1.9 + 0.4, cy, 0.05, 0.8, 0.8, 0.10, GREEN, group=g)
    gap0 = lx0 + 0.8
    for row, (y_off, first) in enumerate([(+0.2, 0.1), (-0.2, 0.4)]):   # 上排 / 下排
        for k in range(3):
            bx = gap0 + first + 0.1 + k * 0.6
            box(f"{g}_block_{row}{k}", bx, cy + y_off, 0.05, 0.2, 0.2, 0.10, GREEN, group=g)

    # ---------------- 5. 沟壑组合 (L 形) ----------------
    # 两顶部平台高 360、800x800；沟壑(两平台间隔)宽 360；台阶深 320 高 120 宽 800；斜坡 20°
    g = "gully_combo"
    py0, py1 = 0.60, 1.40                      # 两平台 Y 范围 (离下边界 0.6)
    pB_x0 = 0.70                               # 斜坡侧平台 (=启动区2)
    pA_x0 = pB_x0 + 0.8 + 0.36                 # 台阶侧平台
    h_top = 0.36
    box(f"{g}_platform_start2", pB_x0 + 0.4, 1.0, h_top / 2, 0.8, 0.8, h_top, YELLOW, group=g)
    box(f"{g}_platform_step", pA_x0 + 0.4, 1.0, h_top / 2, 0.8, 0.8, h_top, GREEN, group=g)
    # 台阶：两级 (240, 120)，再上到 360 的平台
    s_x = pA_x0 + 0.8
    for k, hk in enumerate([0.24, 0.12]):
        box(f"{g}_step_{k}", s_x + 0.16 + k * 0.32, 1.0, hk / 2, 0.32, 0.8, hk, GREEN, group=g)
    # 斜坡：从启动区2平台向 +Y 方向下降到地面，水平投影 = 0.36 / tan20°
    ang = math.radians(20.0)
    run = h_top / math.tan(ang)
    slope_len = h_top / math.sin(ang)
    th = 0.04
    mid_y, mid_z = py1 + run / 2, h_top / 2
    nrm_y, nrm_z = math.sin(ang), math.cos(ang)      # 斜面外法向
    box(f"{g}_slope", pB_x0 + 0.4, mid_y - nrm_y * th / 2, mid_z - nrm_z * th / 2,
        0.8, slope_len, th, GREEN, roll=-ang, group=g)
    _grow(g, pB_x0, py1, pB_x0 + 0.8, py1 + run)
    NAV_FILL.append((pB_x0, py0, s_x + 0.64, py1))      # 两平台 + 沟壑 + 台阶
    NAV_FILL.append((pB_x0, py1, pB_x0 + 0.8, py1 + run))  # 斜坡
    # 启动区2 红色虚线框
    floor_mark_square("start_zone_2_mark", pB_x0 + 0.4, 1.0, 0.76, RED, z=h_top)

    # ---------------- 6. 轮胎阵 2x4，165/70R13：胎宽165 外径550 内径330 ----------------
    g = "tire_array"
    tcx, tcy = 5.70, 1.15            # 阵列中心 (下边缘 0.60)
    r_out, r_in, tw = 0.275, 0.165, 0.165
    nseg = 16
    seg_len = 2 * r_out * math.tan(math.pi / nseg)
    for i in range(4):
        for j in range(2):
            ox = tcx + (i - 1.5) * 2 * r_out
            oy = tcy + (j - 0.5) * 2 * r_out
            for s in range(nseg):
                a = 2 * math.pi * s / nseg
                rm = (r_out + r_in) / 2
                box(f"{g}_t{i}{j}_s{s}", ox + rm * math.cos(a), oy + rm * math.sin(a), tw / 2,
                    r_out - r_in, seg_len, tw, BLACK, yaw=a, occ=False)
            _grow(g, ox - r_out, oy - r_out, ox + r_out, oy + r_out)

    # ---------------- 7. 限高框架 内高290 内宽800 长800 (沿 X 方向穿越) ----------------
    g = "height_limit_frame"
    fcx, fcy = 8.46, 1.15
    tube = 0.04
    in_w, in_l, in_h = 0.80, 0.80, 0.29
    for sx_ in (-1, 1):
        for sy_ in (-1, 1):
            box(f"{g}_post_{sx_}{sy_}", fcx + sx_ * (in_l / 2 - tube / 2), fcy + sy_ * (in_w / 2 + tube / 2),
                (in_h + tube) / 2, tube, tube, in_h + tube, DARK_GREEN, group=g)
    for sy_ in (-1, 1):
        box(f"{g}_rail_{sy_}", fcx, fcy + sy_ * (in_w / 2 + tube / 2), in_h + tube / 2,
            in_l, tube, tube, DARK_GREEN, group=g)
    for k, xo in enumerate((-in_l / 2 + tube / 2, 0.0, in_l / 2 - tube / 2)):
        box(f"{g}_cross_{k}", fcx + xo, fcy, in_h + tube / 2, tube, in_w + 2 * tube, tube, DARK_GREEN, group=g)

    # ---------------- 8. 绕桩 L 形：竖桩 φ100 高100，中心距 900；必达区 φ300 ----------------
    g = "slalom"
    cxp, cyp = 12.95, 1.05          # 拐角桩 (拐角必达区外缘距右/下边界 0.60)
    poles = [(cxp - 0.9 * k, cyp) for k in range(3, 0, -1)] + [(cxp, cyp + 0.9 * k) for k in range(0, 3)]
    for k, (px, py) in enumerate(poles):
        cyl(f"{g}_pole_{k}", px, py, 0.05, 0.05, 0.10, DARK_GREEN, group=g)
    must = [(cxp - 0.9 * 3 - 0.30, cyp), (cxp + 0.30, cyp - 0.30), (cxp, cyp + 0.9 * 2 + 0.30)]
    for k, (mx, my) in enumerate(must):
        floor_mark_circle(f"{g}_must_reach_{k}", mx, my, 0.30, RED)
        _grow(g, mx - 0.15, my - 0.15, mx + 0.15, my + 0.15)

    # ---------------- 启动区1：场地中心 (规则未给尺寸，按图 1-2 约 800x800) ----------------
    floor_mark_square("start_zone_1_mark", FIELD_L / 2, FIELD_W / 2, 0.8, RED)


def check_rules():
    names = list(OBSTACLES)
    ok = True
    print("\n[规则校核] 障碍物距边界 >= 0.6m, 两两间距 >= 0.8m")
    for n in names:
        x0, y0, x1, y1 = OBSTACLES[n]
        m = min(x0, y0, FIELD_L - x1, FIELD_W - y1)
        flag = "OK " if m >= 0.6 - 1e-3 else "XX "
        ok &= m >= 0.6 - 1e-3
        print(f"  {flag}{n:20s} bbox=({x0:5.2f},{y0:4.2f})-({x1:5.2f},{y1:4.2f})  离边界 {m:.2f} m")
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            a, b = OBSTACLES[names[i]], OBSTACLES[names[j]]
            dx = max(0.0, max(a[0], b[0]) - min(a[2], b[2]))
            dy = max(0.0, max(a[1], b[1]) - min(a[3], b[3]))
            d = math.hypot(dx, dy)
            if d < 0.8 - 1e-3:
                ok = False
                print(f"  XX {names[i]} <-> {names[j]} 间距 {d:.2f} m")
    print("  全部满足" if ok else "  存在违规项！")
    return ok


def to_sdf(e, idx):
    x, y, z, r, p, yw = e["pose"]
    if e["kind"] == "box":
        geo = "<box><size>%.4f %.4f %.4f</size></box>" % e["size"]
    else:
        geo = "<cylinder><radius>%.4f</radius><length>%.4f</length></cylinder>" % e["size"]
    col = f"""
        <collision name="c"><geometry>{geo}</geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
        </collision>""" if e["collide"] else ""
    return f"""      <link name="{e['name']}">
        <pose>{x:.4f} {y:.4f} {z:.4f} {r:.5f} {p:.5f} {yw:.5f}</pose>{col}
        <visual name="v"><geometry>{geo}</geometry>
          <material><ambient>{e['color']}</ambient><diffuse>{e['color']}</diffuse></material>
        </visual>
      </link>"""


def write_world(path):
    links = "\n".join(to_sdf(e, i) for i, e in enumerate(ELEMS))
    xml = f"""<?xml version="1.0" ?>
<!-- 2027 足式机器人挑战赛 障碍赛场地 1:1 (由 tools/generate_rc_obstacle_world.py 生成，勿手改) -->
<sdf version="1.6">
  <world name="rc_obstacle_2027">
    <include><uri>model://sun</uri></include>
    <include><uri>model://ground_plane</uri></include>
    <physics type="ode">
      <real_time_update_rate>1000.0</real_time_update_rate>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <gui fullscreen="0">
      <camera name="user_camera">
        <pose>7.0 -6.5 9.0 0 0.85 1.5708</pose>
        <view_controller>orbit</view_controller>
      </camera>
    </gui>
    <model name="rc_obstacle_field">
      <static>true</static>
{links}
    </model>
  </world>
</sdf>
"""
    with open(path, "w") as f:
        f.write(xml)


def rasterize(res):
    W, H = int(round(FIELD_L / res)), int(round(FIELD_W / res))
    grid = np.full((H, W), 255, dtype=np.uint8)       # 255 = 可通行, 0 = 障碍
    xs = (np.arange(W) + 0.5) * res
    ys = (np.arange(H) + 0.5) * res
    X, Y = np.meshgrid(xs, ys)
    for e in ELEMS:
        if not e["occ"]:
            continue
        x, y = e["pose"][0], e["pose"][1]
        yaw = e["pose"][5]
        if e["kind"] == "cyl":
            r = e["size"][0]
            m = (X - x) ** 2 + (Y - y) ** 2 <= (r + res / 2) ** 2
        else:
            sx, sy = e["size"][0], e["size"][1]
            if abs(e["pose"][3]) > 1e-6:            # 斜坡: 取水平投影
                sy = sy * math.cos(e["pose"][3])
            c, s = math.cos(-yaw), math.sin(-yaw)
            lx = c * (X - x) - s * (Y - y)
            ly = s * (X - x) + c * (Y - y)
            m = (np.abs(lx) <= sx / 2 + res / 2) & (np.abs(ly) <= sy / 2 + res / 2)
        grid[m] = 0
    # 整体填实：轮胎阵 / 限高框架 / 梅花桩 / 单木桥 / 高墙 取外接框，沟壑组合取 L 形两块
    rects = [OBSTACLES[n] for n in ("tire_array", "height_limit_frame", "plum_piles",
                                     "single_plank", "high_wall")] + NAV_FILL
    for x0, y0, x1, y1 in rects:
        grid[(X >= x0 - res / 2) & (X <= x1 + res / 2) & (Y >= y0 - res / 2) & (Y <= y1 + res / 2)] = 0
    # 场地边界一圈障碍，防止规划出界
    grid[0, :] = grid[-1, :] = 0
    grid[:, 0] = grid[:, -1] = 0
    return grid


def write_map(grid, res, pgm, yaml_path):
    H, W = grid.shape
    with open(pgm, "wb") as f:
        f.write(f"P5\n{W} {H}\n255\n".encode("ascii"))
        f.write(grid.tobytes())                       # 第 r 行 = y ∈ [r*res, (r+1)*res)
    with open(yaml_path, "w") as f:
        f.write(f"""image: {os.path.basename(pgm)}
mode: trinary
resolution: {res}
origin: [0.0, 0.0, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
# 2027 足式机器人挑战赛 障碍赛 14m x 6m；原点为场地左下角
# 注意: PGM 第 0 行对应 y=0 (与 SCU-TDT-V1 规划器约定一致，与 nav2 map_server 上下相反)
""")


def write_preview(grid, res, path):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return
    plt.rcParams['font.sans-serif'] = ['WenQuanYi Micro Hei', 'WenQuanYi Zen Hei', 'Droid Sans Fallback', 'DejaVu Sans']
    fig, ax = plt.subplots(figsize=(14, 6.6))
    ax.imshow(grid, cmap="gray", origin="lower", extent=(0, FIELD_L, 0, FIELD_W))
    for n, (x0, y0, x1, y1) in OBSTACLES.items():
        ax.add_patch(plt.Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False, ec="tab:green", lw=1.2, ls="--"))
        ax.text((x0 + x1) / 2, y1 + 0.08, n, ha="center", fontsize=9, color="tab:green")
    for (cx, cy, lbl) in [(FIELD_L / 2, FIELD_W / 2, "start1"), (1.10, 1.0, "start2")]:
        ax.add_patch(plt.Rectangle((cx - 0.4, cy - 0.4), 0.8, 0.8, fill=False, ec="red", lw=1.5, ls=":"))
        ax.text(cx, cy, lbl, ha="center", va="center", color="red", fontsize=9)
    ax.set_xticks(np.arange(0, 14.01, 1.0))
    ax.set_yticks(np.arange(0, 6.01, 1.0))
    ax.grid(alpha=0.25)
    ax.set_title("RC 2027 legged robot obstacle field 14m x 6m (nav grid %.2fm)" % res)
    fig.tight_layout()
    fig.savefig(path, dpi=110)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--fence", type=float, default=0.0, help="围挡高度(m)，0 = 不加 (规则未规定障碍赛围墙)")
    ap.add_argument("--res", type=float, default=0.05, help="导航栅格分辨率 (m)")
    args = ap.parse_args()

    build_field(args.fence)
    build_obstacles()
    check_rules()

    os.makedirs(os.path.join(ROOT, "worlds"), exist_ok=True)
    os.makedirs(os.path.join(ROOT, "maps"), exist_ok=True)
    world = os.path.join(ROOT, "worlds", "rc_obstacle_2027.world")
    pgm = os.path.join(ROOT, "maps", "rc_obstacle_2027.pgm")
    yml = os.path.join(ROOT, "maps", "rc_obstacle_2027.yaml")
    png = os.path.join(ROOT, "maps", "rc_obstacle_2027_preview.png")

    write_world(world)
    grid = rasterize(args.res)
    write_map(grid, args.res, pgm, yml)
    write_preview(grid, args.res, png)
    print(f"\n✅ Gazebo 世界 : {world}  ({len(ELEMS)} 个 link)")
    print(f"✅ 导航栅格   : {pgm}  ({grid.shape[1]}x{grid.shape[0]} @ {args.res}m)")
    print(f"✅ 预览图     : {png}")


if __name__ == "__main__":
    main()
