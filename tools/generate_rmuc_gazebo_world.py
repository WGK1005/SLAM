#!/usr/bin/env -S python3 -s
"""
将 TDT-Nav-Kit 原版 images/rmuc2025.png (560x300, res=0.05m, 28m x 15m)
自动转换为 Gazebo 11 3D 物理赛场世界文件: worlds/rmuc2025.world
同时导出标准 ROS 2 nav2 map_server 栅格图: maps/rmuc2025.pgm + maps/rmuc2025.yaml
"""
import os
import numpy as np
from PIL import Image

def extract_merged_rectangles(occ_mask):
    """
    将二值障碍物矩阵 (H, W) 分解为互不重叠的合并矩形块 (r0, c0, r1, c1)，
    大幅减少 Gazebo link 数量，使 3D 物理与 VLP-16 激光仿真保持 >60 FPS 流畅运行。
    """
    H, W = occ_mask.shape
    visited = np.zeros_like(occ_mask, dtype=bool)
    rects = []

    # 降采样到 2x2 像素 (0.10m 栅格) 进行游程矩形合并，保持 10cm 几何精度且极大减少 box 数量
    step = 2
    sH, sW = H // step, W // step
    small_occ = np.zeros((sH, sW), dtype=bool)
    for r in range(sH):
        for c in range(sW):
            block = occ_mask[r*step:(r+1)*step, c*step:(c+1)*step]
            if np.mean(block) >= 0.5:
                small_occ[r, c] = True

    vis = np.zeros_like(small_occ, dtype=bool)
    for r in range(sH):
        c = 0
        while c < sW:
            if small_occ[r, c] and not vis[r, c]:
                # 向右扩展
                c1 = c + 1
                while c1 < sW and small_occ[r, c1] and not vis[r, c1]:
                    c1 += 1
                # 向下扩展
                r1 = r + 1
                while r1 < sH:
                    row_ok = True
                    for cc in range(c, c1):
                        if not small_occ[r1, cc] or vis[r1, cc]:
                            row_ok = False
                            break
                    if row_ok:
                        r1 += 1
                    else:
                        break
                vis[r:r1, c:c1] = True
                rects.append((r * step, c * step, r1 * step, c1 * step))
                c = c1
            else:
                c += 1
    return rects

def main():
    root_dir = "/home/wgk/decision_planning/SCU-TDT-V1"
    img_path = os.path.join(root_dir, "images/rmuc2025.png")
    worlds_dir = os.path.join(root_dir, "worlds")
    maps_dir = os.path.join(root_dir, "maps")
    os.makedirs(worlds_dir, exist_ok=True)
    os.makedirs(maps_dir, exist_ok=True)

    im = Image.open(img_path).convert('L')
    raw = np.array(im)
    H, W = raw.shape
    res = 0.05

    # 保存 PGM 与 YAML 供 ROS 2 / RViz2 使用
    # 注意：为了让原版 TDT-Nav-Kit 坐标 (x = col * 0.05, y = row * 0.05) 在 Gazebo/RViz 右手系中直观一致，
    # 我们令世界坐标系 X ∈ [0, 28.0m], Y ∈ [0, 15.0m]，并将图像上下翻转存入 PGM (因为 ROS map_server 从左下角读取 PGM)
    pgm_path = os.path.join(maps_dir, "rmuc2025.pgm")
    with open(pgm_path, "wb") as f:
        f.write(f"P5\n{W} {H}\n255\n".encode('ascii'))
        # 直接按原始行写入，让 C++ ScuTdtPlannerEngine 和 RViz 保持 100% 相同坐标映射
        f.write(raw.tobytes())

    yaml_path = os.path.join(maps_dir, "rmuc2025.yaml")
    with open(yaml_path, "w") as f:
        f.write(f"""image: rmuc2025.pgm
mode: trinary
resolution: {res}
origin: [0.0, 0.0, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
""")

    occ_mask = (raw < 128)
    rects = extract_merged_rectangles(occ_mask)

    wall_height = 1.10
    links_xml = []
    for idx, (r0, c0, r1, c1) in enumerate(rects):
        x0 = c0 * res
        x1 = c1 * res
        y0 = r0 * res
        y1 = r1 * res
        cx = 0.5 * (x0 + x1)
        cy = 0.5 * (y0 + y1)
        sx = max(0.05, x1 - x0)
        sy = max(0.05, y1 - y0)
        cz = 0.5 * wall_height

        # 区分外围边界墙与内部战术掩体颜色
        is_border = (r0 <= 4 or r1 >= H - 4 or c0 <= 4 or c1 >= W - 4)
        mat_name = "Gazebo/Grey" if is_border else "Gazebo/DarkGrey"
        ambient = "0.35 0.38 0.42 1" if is_border else "0.22 0.30 0.45 1"

        links_xml.append(f"""      <link name="wall_{idx}">
        <pose>{cx:.3f} {cy:.3f} {cz:.3f} 0 0 0</pose>
        <collision name="collision">
          <geometry><box><size>{sx:.3f} {sy:.3f} {wall_height:.3f}</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>{sx:.3f} {sy:.3f} {wall_height:.3f}</size></box></geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>{mat_name}</name>
            </script>
            <ambient>{ambient}</ambient>
          </material>
        </visual>
      </link>""")

    world_xml = f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="rmuc2025_world">
    <include>
      <uri>model://sun</uri>
    </include>
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <physics type="ode">
      <real_time_update_rate>1000.0</real_time_update_rate>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <gui fullscreen="0">
      <camera name="user_camera">
        <pose>14.0 -8.5 22.0 0 0.92 1.5708</pose>
        <view_controller>orbit</view_controller>
      </camera>
    </gui>

    <model name="rmuc2025_arena">
      <static>true</static>
{chr(10).join(links_xml)}
    </model>
  </world>
</sdf>
"""
    world_path = os.path.join(worlds_dir, "rmuc2025.world")
    with open(world_path, "w") as f:
        f.write(world_xml)

    print(f"✅ 已生成 RMUC 2025 3D Gazebo 赛场世界: {world_path} (共合并为 {len(rects)} 个 3D 障碍物实体)")
    print(f"✅ 已生成 RMUC 2025 2D 栅格地图: {pgm_path}")

if __name__ == '__main__':
    main()

