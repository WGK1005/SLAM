#!/usr/bin/env python3
"""点云地图质量分析（不依赖 ROS，纯 numpy + scipy）

用法:
    python3 tools/map_quality.py map1.pcd [map2.pcd ...]

输出两类指标
  1) 基本统计：点数、占地、占用面积、密度、z 分布
  2) 结构厚度：用 2D 距离变换量"墙体有多厚"，这是判断涂抹最可靠的指标
       中位 ≈ 0.05~0.15 m  → 单层薄墙，地图干净
       中位 >  0.30 m      → 涂抹（走太快 / 时间偏移 / 外参）

参考基线（本项目实测）
    pc_base2.pcd（笔记本，缓慢匀速）      厚度中位 0.10 m  P75 0.14  P90 0.20
    map_1017_0926.pcd（派，走快了）        厚度中位 0.14 m  P75 0.32  P90 0.51
    map_1017_1017.pcd（派，走+停，达标）   厚度中位 0.10 m  P75 0.14  P90 0.20
"""
import sys

import numpy as np
from scipy import ndimage


def load_pcd(p):
    fields, sizes, types, points = [], [], [], 0
    with open(p, 'rb') as f:
        while True:
            line = f.readline()
            if not line:
                raise SystemExit('PCD 头解析失败: ' + p)
            k = line.split()[0].decode(errors='ignore').upper()
            if k == 'FIELDS':
                fields = [x.decode() for x in line.split()[1:]]
            elif k == 'SIZE':
                sizes = [int(x) for x in line.split()[1:]]
            elif k == 'TYPE':
                types = [x.decode() for x in line.split()[1:]]
            elif k == 'POINTS':
                points = int(line.split()[1])
            elif k == 'DATA':
                break
        tmap = {'F': 'f', 'U': 'u', 'I': 'i'}
        dt = np.dtype([(n, '<%s%d' % (tmap[t], s)) for n, s, t in zip(fields, sizes, types)])
        a = np.frombuffer(f.read(points * dt.itemsize), dtype=dt)
    P = np.stack([np.asarray(a['x'], float), np.asarray(a['y'], float),
                  np.asarray(a['z'], float)], 1)
    return P[np.isfinite(P).all(1)]


def detect_floor(z):
    hist, edges = np.histogram(z, bins=120)
    k = int(np.argmax(hist))
    return float(0.5 * (edges[k] + edges[k + 1]))


def analyze(p, cell=0.05, band=(0.15, 3.0)):
    P = load_pcd(p)
    x, y, z = P[:, 0], P[:, 1], P[:, 2]
    r = np.hypot(x, y)
    keep = r <= np.percentile(r, 99.5)
    x, y, z = x[keep], y[keep], z[keep]
    floor = detect_floor(z)
    m = (z >= floor + band[0]) & (z <= floor + band[1])
    xw, yw = x[m], y[m]
    out = {'name': p.split('/')[-1], 'n': len(P), 'floor': floor,
           'sx': x.max() - x.min(), 'sy': y.max() - y.min(),
           'zmin': z.min(), 'zmax': z.max()}
    if len(xw) < 200:
        out['err'] = '墙面点太少(%d)' % len(xw)
        return out
    ix = np.floor((xw - xw.min()) / cell).astype(np.int64)
    iy = np.floor((yw - yw.min()) / cell).astype(np.int64)
    grid = np.zeros((iy.max() + 3, ix.max() + 3), dtype=bool)
    grid[iy + 1, ix + 1] = True
    dist = ndimage.distance_transform_edt(grid)[grid] * cell * 2.0
    out.update(cells=int(grid.sum()), area=grid.sum() * cell * cell,
               thick_p25=float(np.percentile(dist, 25)),
               thick_med=float(np.median(dist)),
               thick_p75=float(np.percentile(dist, 75)),
               thick_p90=float(np.percentile(dist, 90)),
               pts_per_cell=len(xw) / max(grid.sum(), 1))
    return out


def main(paths):
    print('%-24s %8s %7s %7s %8s %8s %8s %8s %7s' %
          ('地图', '点数', 'X跨度', 'Y跨度', '占用m²', '厚度中位', 'P75', 'P90', '点/格'))
    print('-' * 104)
    for p in paths:
        try:
            o = analyze(p)
        except SystemExit as e:
            print('%-24s 读取失败: %s' % (p.split('/')[-1], e))
            continue
        if 'err' in o:
            print('%-24s %8d  %s' % (o['name'], o['n'], o['err']))
            continue
        med, p75 = o['thick_med'], o['thick_p75']
        if med <= 0.12 and p75 <= 0.18:
            verdict = '✅ 优秀'
        elif med <= 0.16 and p75 <= 0.25:
            verdict = '🟢 良好'
        elif p75 <= 0.35:
            verdict = '🟡 局部涂抹'
        else:
            verdict = '❌ 明显涂抹'
        print('%-24s %8d %7.1f %7.1f %8.1f %8.3f %8.3f %8.3f %7.1f  %s' %
              (o['name'], o['n'], o['sx'], o['sy'], o['area'],
               o['thick_med'], o['thick_p75'], o['thick_p90'],
               o['pts_per_cell'], verdict))
        print('%24s   z∈[%.2f, %.2f]  地面检测 %.2f m' % ('', o['zmin'], o['zmax'], o['floor']))
    print()
    print('判读：')
    print('  ✅ 优秀      厚度中位 ≤0.12 且 P75 ≤0.18   （参考 pc_base2: 0.100 / 0.141）')
    print('  🟢 良好      中位 ≤0.16 且 P75 ≤0.25')
    print('  🟡 局部涂抹  P75 ≤0.35 —— 约 1/4 结构偏厚，能看但有重影')
    print('  ❌ 明显涂抹  P75 >0.35 —— 走太快')


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    main(sys.argv[1:])
