#!/usr/bin/env python3
"""把 LiDAR-IMU 外参写进 config/velodyne.yaml（免得手改缩进出错）

用法：
  # 只改平移（LiDAR 中心在 IMU 坐标系下的位置，单位 m）
  python3 tools/set_extrinsic.py --t -0.02 0.00 -0.03

  # 只改旋转（LiDAR→IMU，即 v_imu = R · v_lidar，按行给 9 个数）
  python3 tools/set_extrinsic.py --r 0,-1,0, 1,0,0, 0,0,1

  # 两个一起
  python3 tools/set_extrinsic.py --t 0 0 0.05 --r 1,0,0, 0,1,0, 0,0,1

  # 只看当前值
  python3 tools/set_extrinsic.py --show

改完记得重启 launch 才生效。
"""
import argparse
import re
import sys
from pathlib import Path

YAML = Path(__file__).resolve().parent.parent / "src" / "FAST_LIO" / "config" / "velodyne.yaml"


def read_curr(text):
    t = re.search(r'extrinsic_T:\s*\[([^\]]*)\]', text)
    r = re.search(r'extrinsic_R:\s*\[([^\]]*)\]', text, flags=re.S)
    return (t.group(1).strip() if t else '?'), (r.group(1).strip() if r else '?')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--t', nargs=3, type=float, metavar=('X', 'Y', 'Z'),
                    help='LiDAR 在 IMU 坐标系下的位置(m)')
    ap.add_argument('--r', nargs='+', default=None,
                    help='LiDAR→IMU 旋转矩阵，按行 9 个数（可写成 "0,-1,0, 1,0,0, 0,0,1"，空格会被自动吞掉）')
    ap.add_argument('--show', action='store_true', help='只显示当前值')
    args = ap.parse_args()

    if not YAML.exists():
        print(f'❌ 找不到 {YAML}', file=sys.stderr)
        return 1
    text = YAML.read_text(encoding='utf-8')
    cur_t, cur_r = read_curr(text)

    if args.show or (args.t is None and args.r is None):
        print(f'当前 {YAML.name}:')
        print(f'  extrinsic_T: [{cur_t}]')
        print(f'  extrinsic_R: [{cur_r}]')
        return 0

    if args.t is not None:
        new_t = '[ %.4f, %.4f, %.4f]' % tuple(args.t)
        text, n = re.subn(r'extrinsic_T:\s*\[[^\]]*\]', 'extrinsic_T: ' + new_t, text)
        if n != 1:
            print('❌ 没找到唯一的 extrinsic_T 行', file=sys.stderr)
            return 1
        print(f'✅ extrinsic_T -> {new_t}')

    if args.r is not None:
        raw = ','.join(args.r)                      # nargs='+' → 允许 "0,-1,0, 1,0,0, 0,0,1" 带空格
        nums = [x.strip() for x in raw.split(',') if x.strip()]
        if len(nums) != 9:
            print(f'❌ extrinsic_R 需要 9 个数，收到 {len(nums)} 个', file=sys.stderr)
            return 1
        f = [float(x) for x in nums]
        new_r = ('[ %.4f, %.4f, %.4f,\n'
                 '                           %.4f, %.4f, %.4f,\n'
                 '                           %.4f, %.4f, %.4f]' % tuple(f))
        text, n = re.subn(r'extrinsic_R:\s*\[[^\]]*\]', 'extrinsic_R: ' + new_r, text, flags=re.S)
        if n != 1:
            print('❌ 没找到唯一的 extrinsic_R 块', file=sys.stderr)
            return 1
        # 顺带做个合法性检查：旋转矩阵行列式应为 +1
        det = (f[0] * (f[4] * f[8] - f[5] * f[7])
               - f[1] * (f[3] * f[8] - f[5] * f[6])
               + f[2] * (f[3] * f[7] - f[4] * f[6]))
        print(f'✅ extrinsic_R -> {new_r}')
        print(f'   行列式 = {det:+.3f}' + ('  ✅ 合法旋转' if abs(det - 1) < 0.01 else '  ⚠️ 不是合法旋转矩阵(det 应为 +1)'))

    YAML.write_text(text, encoding='utf-8')
    print(f'\n已写入 {YAML}\n重启 launch 后生效。')
    return 0


if __name__ == '__main__':
    sys.exit(main())
