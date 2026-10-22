#!/usr/bin/env python3
"""时间偏移（time_offset_lidar_to_imu）自动扫描标定

原理
    雷达与 IMU 之间存在固定的时间偏移 Δt。Δt 不对时，运动中的点会被"抹开"：
        涂抹量 ≈ 角速度 ω × Δt × 距离 r
    所以：把同一段 bag 用不同的 Δt 各跑一遍 FAST-LIO，**地图最锐（墙最薄）的那个 Δt 就是正确值**。

指标（都是"越小越锐"）
    · vox5cm   : 5cm 体素占用数（同样场景，涂抹越大占的体素越多）
    · thick    : 表面厚度 = 各 10cm 体素内点云沿最小主方向的散布均值（墙越薄值越小）

用法
    # 默认扫 7 个值，每段用 bag 的前 45 秒
    python3 tools/scan_time_offset.py --bag ~/bags/pc_base2

    # 自定义候选值与段长
    python3 tools/scan_time_offset.py --bag ~/bags/dyn --offsets -0.06 -0.03 0 0.03 0.06 --segment 30

    # 只看运动强度不做扫描（判断这份 bag 能不能标）
    python3 tools/scan_time_offset.py --bag ~/bags/pc_base2 --check-only

注意
    0) ★ **bag 开头必须静止 3~5 秒**（比"运动够猛"更硬性！）
       FAST-LIO 用第一帧雷达对应的 ~20 个 IMU 样本初始化（IMU_Processing.hpp:193-196）：
           grav = 加速度均值方向        ← 在动 → 重力方向估歪
           bg   = 角速度均值（陀螺零偏）← 在动 → 把运动当成零偏，之后一直减掉
       开头在动 = 整段全错，而且每个候选 offset 错得一样，扫描结果毫无区分度。
       脚本会自动检查并拒绝扫描（真的想硬跑加 --force）。
    1) 扫描期间**不要跑其它重负载**（RViz/编译/拷贝），否则卡顿会让时间戳出现空档 → 发散
    2) 段长越长信号越强但越慢；建议 segment 30~60 秒，且包含**快速转动**的部分
    3) 结束后会把最优值写回 config/velodyne.yaml
"""
import argparse
import os
import re
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path

WORKSPACE = Path(__file__).resolve().parent.parent
CONFIG = WORKSPACE / 'src' / 'FAST_LIO' / 'config' / 'velodyne.yaml'
TOOLS = WORKSPACE / 'tools'
DEFAULT_MAP = Path.home() / 'maps' / 'scan_current.pcd'
# 落盘备份：进程被 SIGKILL（例如外层工具超时）时 finally 不会执行，
# 下次启动靠它把配置还原回来，绝不让 dense_publish_en / map_file_path 留在中间状态
BACKUP = CONFIG.with_name(CONFIG.name + '.scanbak')

CLEAN_KEYS = ['PKG_CONFIG_PATH', 'CMAKE_PREFIX_PATH', 'COLCON_PREFIX_PATH', 'AMENT_PREFIX_PATH',
              'LD_LIBRARY_PATH', 'PYTHONPATH', 'DDSGEN', 'AMENT_CURRENT_PREFIX']


def clean_env():
    env = dict(os.environ)
    for k in CLEAN_KEYS:
        env.pop(k, None)
    return env


def ros_shell(cmd: str):
    """在干净的 ROS 环境里执行命令"""
    inner = f'source /opt/ros/humble/setup.bash && source {WORKSPACE}/install/setup.bash && {cmd}'
    return subprocess.Popen(['bash', '--noprofile', '--norc', '-c', inner],
                            env=clean_env(), stdin=subprocess.DEVNULL,
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                            start_new_session=True)


def ros_shell_log(cmd: str, logfile):
    """同 ros_shell，但把 stdout/stderr 落到文件（诊断用）"""
    inner = f'source /opt/ros/humble/setup.bash && source {WORKSPACE}/install/setup.bash && {cmd}'
    return subprocess.Popen(['bash', '--noprofile', '--norc', '-c', inner],
                            env=clean_env(), stdin=subprocess.DEVNULL,
                            stdout=logfile, stderr=subprocess.STDOUT,
                            start_new_session=True)


def recover_backup():
    if BACKUP.exists():
        CONFIG.write_text(BACKUP.read_text(encoding='utf-8'), encoding='utf-8')
        BACKUP.unlink()
        print('[恢复] 上次运行被强制中断，已用备份还原配置')


def restore_original(text: str):
    CONFIG.write_text(text, encoding='utf-8')
    if BACKUP.exists():
        BACKUP.unlink()


def set_config_value(pattern: str, repl: str):
    text = CONFIG.read_text(encoding='utf-8')
    new, n = re.subn(pattern, repl, text)
    if n != 1:
        raise RuntimeError(f'配置里匹配 {pattern} 得到 {n} 处，期望 1 处')
    CONFIG.write_text(new, encoding='utf-8')


def set_time_offset(v: float):
    set_config_value(r'time_offset_lidar_to_imu:\s*[-+0-9.eE]+',
                     f'time_offset_lidar_to_imu: {v:.4f}')


def set_topic(key: str, topic: str):
    set_config_value(key + r':\s*"[^"]*"', f'{key}: "{topic}"')


def set_dense_map(enabled: bool):
    v = 'true' if enabled else 'false'
    set_config_value(r'dense_publish_en:\s*(true|false)', f'dense_publish_en: {v}')


def set_filter_size_surf(v: float):
    set_config_value(r'filter_size_surf:\s*[-+0-9.eE]+', f'filter_size_surf: {v:g}')


def set_map_path(p: Path):
    set_config_value(r'map_file_path:\s*"[^"]*"', f'map_file_path: "{p}"')


def kill_all(procs):
    for p in procs:
        if p.poll() is None:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGTERM)
            except Exception:
                p.terminate()
    for p in procs:
        try:
            p.wait(timeout=5)
        except Exception:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGKILL)
            except Exception:
                pass


HEAD_SEC = 3.0        # 开头检查窗口
STATIC_W = 0.25       # 静止判据：|ω| < 0.25 rad/s
STATIC_A_TOL = 1.0    # 且 ||a| - 9.81| < 1.0


def _static_mask(ws, accs):
    import numpy as np
    return (ws < STATIC_W) & (np.abs(accs - 9.81) < STATIC_A_TOL)


def _longest_run(mask, ts):
    """最长连续 True 段 → (起始时刻, 持续秒数)"""
    best = (-1.0, 0.0)
    i = 0
    while i < len(mask):
        if mask[i]:
            j = i
            while j < len(mask) and mask[j]:
                j += 1
            dur = ts[j - 1] - ts[i]
            if dur > best[1]:
                best = (ts[i], dur)
            i = j
        else:
            i += 1
    return best


def bag_motion(bag: str, segment: float):
    """读 bag 里 IMU 的角速度/加速度统计，判断这份数据适不适合标时间偏移

    除运动强度外，还必须检查**开头是否静止**——见文件头 注意 0)。
    """
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from sensor_msgs.msg import Imu
    except Exception as e:
        return None, f'（无法读取 bag: {e}）'
    import numpy as np
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'),
           rosbag2_py.ConverterOptions('cdr', 'cdr'))
    t0 = None
    ts, ws, accs = [], [], []
    while r.has_next():
        topic, data, stamp = r.read_next()
        if topic != '/imu/data':
            continue
        if t0 is None:
            t0 = stamp * 1e-9
        if stamp * 1e-9 - t0 > segment:
            break
        m = deserialize_message(data, Imu)
        g, a = m.angular_velocity, m.linear_acceleration
        ts.append(stamp * 1e-9 - t0)
        ws.append((g.x * g.x + g.y * g.y + g.z * g.z) ** 0.5)
        accs.append((a.x * a.x + a.y * a.y + a.z * a.z) ** 0.5)
    if not ws:
        return None, '（bag 里没有 IMU）'
    ts = np.array(ts)
    ws = np.array(ws)
    accs = np.array(accs)
    mask = _static_mask(ws, accs)
    head_end = min(segment, HEAD_SEC)
    hm = ts < head_end
    st = dict(
        max=float(ws.max()),
        p95=float(np.percentile(ws, 95)),
        mean=float(ws.mean()),
        n=len(ws),
        head_frac=float(mask[hm].mean()) if hm.any() else 0.0,
        static_frac=float(mask.mean()),
    )
    st['long_t0'], st['long_dur'] = _longest_run(mask, ts)
    return st, ''


def map_metrics(pcd: Path):
    """返回 (vox5cm 数, 表面厚度 m, 点数, 参与厚度统计的体素数)"""
    import numpy as np
    import math
    # ---- 解析 binary PCD（同 render_map.py 的逻辑，避免互相 import 出问题）----
    fields, sizes, types, points, fmt = [], [], [], 0, None
    with open(pcd, 'rb') as f:
        while True:
            line = f.readline()
            if not line:
                raise RuntimeError('PCD 头解析失败')
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
                fmt = line.split()[1].decode().lower()
                break
        if fmt != 'binary':
            raise RuntimeError('只支持 binary PCD')
        tmap = {'F': 'f', 'U': 'u', 'I': 'i'}
        dt = np.dtype([(n, f'<{tmap[t]}{s}') for n, s, t in zip(fields, sizes, types)])
        a = np.frombuffer(f.read(points * dt.itemsize), dtype=dt)

    x = np.asarray(a['x'], np.float64)
    y = np.asarray(a['y'], np.float64)
    z = np.asarray(a['z'], np.float64)
    ok = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
    x, y, z = x[ok], y[ok], z[ok]
    r = np.sqrt(x * x + y * y + z * z)
    m = r < 40                      # 只统计场地区域，排除远处离群
    x, y, z = x[m], y[m], z[m]
    P = np.stack([x, y, z], axis=1)

    vox5 = np.unique(np.floor(P / 0.05).astype(np.int64), axis=0)
    g, inv = np.unique(np.floor(P / 0.10).astype(np.int64), axis=0, return_inverse=True)
    thick = []
    order = np.argsort(inv, kind='stable')
    bounds = np.searchsorted(inv[order], np.arange(len(g) + 1))
    for i in range(len(g)):
        idx = order[bounds[i]:bounds[i + 1]]
        if len(idx) < 5:
            continue
        Q = P[idx] - P[idx].mean(axis=0)
        ev = np.linalg.eigvalsh(np.cov(Q.T))
        thick.append(math.sqrt(max(ev[0], 0.0)))
    return len(vox5), (sum(thick) / len(thick) if thick else float('nan')), len(P), len(thick)


MAPSAVE_SNIPPET = r'''
import os
import sys
import rclpy
from std_srvs.srv import Trigger

rclpy.init()
node = rclpy.create_node('map_save_caller_%d' % os.getpid())
cli = node.create_client(Trigger, '/map_save')
if not cli.wait_for_service(timeout_sec=60.0):
    print('NO_SERVICE', flush=True)
    sys.exit(2)
if len(sys.argv) > 1 and sys.argv[1] == 'wait':
    print('SERVICE_OK', flush=True)
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0)
fut = cli.call_async(Trigger.Request())
rclpy.spin_until_future_complete(node, fut, timeout_sec=25.0)
ok = bool(fut.done() and fut.result() is not None and fut.result().success)
print('RESULT', ok, flush=True)
node.destroy_node()
rclpy.shutdown()
sys.exit(0 if ok else 3)
'''


def _run_mapsave_py(procs, args='', timeout=90):
    import tempfile
    fd, path = tempfile.mkstemp(suffix='.py', prefix='map_save_')
    with os.fdopen(fd, 'w') as f:
        f.write(MAPSAVE_SNIPPET)
    logpath = '/tmp/scan_mapsave.log'
    lf = open(logpath, 'w')
    p = ros_shell_log(f'exec python3 {path} {args}'.rstrip(), lf)
    procs.append(p)
    try:
        p.wait(timeout=timeout)
        return p.returncode
    except subprocess.TimeoutExpired:
        try:
            p.terminate()
        except Exception:
            pass
        return -1
    finally:
        try:
            lf.close()
        except Exception:
            pass
        try:
            os.unlink(path)
        except OSError:
            pass
        try:
            with open(logpath) as f:
                txt = f.read().strip()
            if txt:
                print(f'      [map_save 输出] {txt[:400]}', flush=True)
        except OSError:
            pass


def probe_map_save(procs, timeout=45):
    """回放前先确认 /map_save 可发现。不可发现说明 DDS 拓扑有问题
    （典型原因：接了/拔了网线、Wi-Fi 换网段，导致参与者绑在旧接口上）"""
    for _ in (1, 2):
        rc = _run_mapsave_py(procs, args='wait', timeout=timeout)
        if rc == 0:
            return True, ''
        time.sleep(2)
    return False, ('/map_save 服务不可发现 —— DDS 拓扑异常。'
                   '多半是网络接口变了（插拔网线/换 Wi-Fi 网段），重启所有 ROS 节点即可')


def call_map_save(procs):
    """调用 /map_save。用 rclpy 客户端而不是 `ros2 service call` CLI：
    CLI 会挂住不退出，变成僵尸客户端——下次 FAST-LIO 一启动它就立刻调用，
    此时地图还是空的 → PCDWriter 空点云 → SIGABRT (exit -6)。"""
    import tempfile
    fd, path = tempfile.mkstemp(suffix='.py', prefix='map_save_')
    with os.fdopen(fd, 'w') as f:
        f.write(MAPSAVE_SNIPPET)
    try:
        for attempt in (1, 2):
            p = ros_shell(f'exec python3 {path}')
            procs.append(p)              # ★ 必须登记，否则异常时杀不掉
            try:
                p.wait(timeout=90)
            except subprocess.TimeoutExpired:
                try:
                    p.terminate()
                except Exception:
                    pass
                if attempt == 1:
                    time.sleep(3)
                    continue
                return False, '存图服务无响应（超时）'
            if p.returncode == 0:
                return True, ''
            if attempt == 1:
                time.sleep(3)
                continue
            return False, f'存图服务返回码 {p.returncode}（2=服务发现失败 3=调用失败）'
        return False, '存图失败'
    finally:
        try:
            os.unlink(path)
        except OSError:
            pass


def kill_stale_map_save():
    """清掉历史遗留的僵尸 map_save 调用进程（它们会打死新起的 FAST-LIO）"""
    try:
        subprocess.run(['pkill', '-f', 'ros2 service call /map_save'],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=10)
        subprocess.run(['pkill', '-f', 'map_save_caller'],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=10)
        time.sleep(1)
    except Exception:
        pass


def run_once(bag: str, segment: float, rate: float, map_path: Path, warmup: float = 8.0,
             raw: bool = False):
    """跑一遍：规范化 + FAST-LIO + 回放 segment 秒，然后存图。返回 (ok, 备注)"""
    if map_path.exists():
        map_path.unlink()
    kill_stale_map_save()
    procs = []
    map_proc = play_proc = None
    try:
        if not raw:
            procs.append(ros_shell(f'exec python3 {TOOLS}/timestamp_normalizer.py'))
            time.sleep(2)
        map_proc = ros_shell('exec ros2 launch fast_lio mapping.launch.py '
                             'config_file:=velodyne.yaml rviz:=false')
        procs.append(map_proc)
        time.sleep(warmup)
        if map_proc.poll() is not None:
            return False, f'FAST-LIO 启动即退出（返回码 {map_proc.returncode}）'
        ok, why = probe_map_save(procs)
        if not ok:
            return False, why
        # 回放指定长度（用 timeout 截断）
        play_proc = ros_shell(f'exec timeout {segment:.0f} ros2 bag play {bag} --rate {rate}')
        procs.append(play_proc)
        time.sleep(segment + 4)
        if play_proc.poll() is None:
            play_proc.terminate()
        if map_proc.poll() is not None:
            return False, f'FAST-LIO 在回放中退出（返回码 {map_proc.returncode}，-6=空点云 abort）'
        # 存图
        alive = subprocess.run(['pgrep', '-f', 'fastlio_mapping'],
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode == 0
        if not alive:
            return False, 'FAST-LIO 进程已消失（看 PCDWriter 空点云 abort / OOM）'
        ok, why = call_map_save(procs)
        time.sleep(2)
        if not map_path.exists():
            return False, f'存图失败：{why or "文件未生成"}'
        return True, ''
    finally:
        kill_all(procs)
        kill_stale_map_save()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--bag', required=True)
    ap.add_argument('--offsets', nargs='*', type=float,
                    default=[-0.06, -0.04, -0.02, 0.0, 0.02, 0.04, 0.06])
    ap.add_argument('--segment', type=float, default=45.0, help='每段使用的 bag 时长（秒）')
    ap.add_argument('--rate', type=float, default=1.0, help='回放倍速（建议 1.0，倍速过大会掩盖问题）')
    ap.add_argument('--check-only', action='store_true', help='只检查 bag 能不能标（运动强度 + 开头是否静止），不扫描')
    ap.add_argument('--force', action='store_true', help='开头不静止也强行扫描（已知会得到无意义结果）')
    ap.add_argument('--dense-map', dest='dense_map', action='store_true', default=True,
                    help='扫描期间用全分辨率点累积地图（默认开）。关掉会让地图过稀、厚度指标退化')
    ap.add_argument('--no-dense-map', dest='dense_map', action='store_false')
    ap.add_argument('--raw-topics', action='store_true',
                    help='不用规范化节点，直接用 bag 里原始 header 时间戳（确定性回放，推荐）')
    ap.add_argument('--filter-size', type=float, default=0.0,
                    help='扫描期间临时覆盖 filter_size_surf（如 0.05 让每帧特征更密）；0=不改')
    args = ap.parse_args()

    recover_backup()
    bag = os.path.expanduser(args.bag)
    if not os.path.isdir(bag):
        print(f'❌ 找不到 bag: {bag}')
        return 1

    print(f'=== 1) bag 可标性检查 ===')
    st, note = bag_motion(bag, args.segment)
    head_ok = True
    if st:
        print(f'  IMU 样本 {st["n"]}  角速度 |ω|: 均值 {st["mean"]:.4f} '
              f'P95 {st["p95"]:.4f}  最大 {st["max"]:.4f} rad/s')
        if st['p95'] < 0.3:
            print('  ⚠️⚠️ 运动太轻（P95 < 0.3 rad/s）：时间偏移的涂抹信号很弱，标出来的值不可靠')
            print('        需要"手持快速左右转 ±90°"，角速度 1~2 rad/s')
        elif st['p95'] < 0.8:
            print('  ⚠️ 运动偏轻（P95 < 0.8 rad/s）：可以标，但最优值可能不够尖锐')
        else:
            print('  ✅ 运动强度足够')

        print(f'\n  开头 {HEAD_SEC:.0f}s 静止占比: {st["head_frac"]*100:.0f}%   '
              f'（FAST-LIO 初始化窗口，需 ≥60%）')
        print(f'  全程静止占比: {st["static_frac"]*100:.1f}%   '
              f'最长连续静止: {st["long_dur"]:.2f}s'
              + (f' @t={st["long_t0"]:.1f}s' if st['long_dur'] > 0 else ''))
        if st['head_frac'] >= 0.6:
            print('  ✅ 开头静止，FAST-LIO 能正确估重力与陀螺零偏')
        else:
            head_ok = False
            print('  ❌ 开头在动！FAST-LIO 会这样初始化（IMU_Processing.hpp:193-196）：')
            print('        grav = 加速度均值方向   → 重力方向估歪 → 地图整体倾斜')
            print(f'        bg   = 角速度均值        → 把 ~{st["mean"]:.2f} rad/s 的运动当成陀螺零偏，之后一直减掉')
            print('     结果：每个候选 offset 都错得一样，扫描无区分度。')
            print('     ★ 重录：按下录制后【先静止数 3~5 秒】再开始晃，其余动作照旧。')
    else:
        print(f'  跳过检查：{note}')
    if args.check_only:
        return 0 if head_ok else 2

    if not head_ok and not args.force:
        print('\n❌ 拒绝扫描：bag 开头不静止（见上）。加 --force 可强行扫描（结果不可信）。')
        return 1

    print(f'\n=== 2) 开始扫描 {len(args.offsets)} 个候选值（每段 {args.segment:.0f}s，约需 '
          f'{len(args.offsets) * (args.segment + 15) / 60:.1f} 分钟） ===')
    original = CONFIG.read_text(encoding='utf-8')
    BACKUP.write_text(original, encoding='utf-8')
    set_map_path(DEFAULT_MAP)
    set_dense_map(args.dense_map)
    if args.raw_topics:
        set_topic('lid_topic', '/velodyne_points')
        set_topic('imu_topic', '/imu/data')
    if args.filter_size > 0:
        set_filter_size_surf(args.filter_size)
    print(f'  扫描期设置: dense_publish_en={args.dense_map}'
          + (f'  filter_size_surf={args.filter_size:g}' if args.filter_size > 0 else '')
          + ('  时间源=bag原始戳(确定性)' if args.raw_topics else '  时间源=墙钟规范化(每次不同!)'))
    DEFAULT_MAP.parent.mkdir(parents=True, exist_ok=True)

    results = []
    completed = False
    try:
        for i, off in enumerate(args.offsets, 1):
            print(f'\n--- [{i}/{len(args.offsets)}] time_offset = {off:+.3f} s ---', flush=True)
            set_time_offset(off)
            t0 = time.time()
            ok, why = run_once(bag, args.segment, args.rate, DEFAULT_MAP, raw=args.raw_topics)
            if not ok:
                print(f'    ❌ {why}（耗时 {time.time()-t0:.0f}s）')
                results.append((off, None, None, None, None))
                continue
            keep = DEFAULT_MAP.with_name(f'scan_offset_{off:+.3f}.pcd')
            shutil.move(str(DEFAULT_MAP), str(keep))
            v, th, n, nc = map_metrics(keep)
            print(f'    点数 {n}  5cm体素 {v}  表面厚度 {th*100:.2f} cm  有效体素 {nc}'
                  f'  （耗时 {time.time()-t0:.0f}s）  → {keep.name}', flush=True)
            if nc < 2000:
                print(f'    ⚠️ 只有 {nc} 个体素够 5 点，厚度指标不可靠 → 用 --dense-map（默认）'
                      f'或加大 --filter-size 密度', flush=True)
            results.append((off, v, th, n, nc))
        completed = True
    finally:
        if not completed:
            restore_original(original)
            print('\n[中断] 配置已还原')

    good = [r for r in results if r[1] is not None]
    print('\n=== 3) 结果对比（越小越锐） ===')
    print(f'  {"offset(s)":>10} {"vox5cm":>9} {"厚度(cm)":>9} {"点数":>8} {"有效体素":>9}')
    for off, v, th, n, nc in results:
        if v is None:
            print(f'  {off:>+10.3f} {"FAIL":>9} {"-":>9} {"-":>8} {"-":>9}')
        else:
            print(f'  {off:>+10.3f} {v:>9d} {th*100:>9.2f} {n:>8d} {nc:>8d}')

    if len(good) >= 2:
        best_v = min(good, key=lambda r: r[2] if r[2] == r[2] else 9e9)   # 按表面厚度
        print(f'\n=== 4) 推荐 ===')
        print(f'  表面厚度最小: time_offset = {best_v[0]:+.3f} s（{best_v[2]*100:.2f} cm）')
        best_vox = min(good, key=lambda r: r[1])
        print(f'  体素数最小  : time_offset = {best_vox[0]:+.3f} s（{best_vox[1]} 体素）')
        if best_v[0] == best_vox[0]:
            # 先整体还原（含扫描期间临时改掉的 map_file_path），再只写 time_offset
            restore_original(original)
            set_time_offset(best_v[0])
            print(f'  两者一致 ✅ 已把 {best_v[0]:+.3f} 写回 {CONFIG.name}'
                  f'（map_file_path 已还原为原值）')
        else:
            print('  两者不一致 ⚠️ 说明信号不够尖锐（很可能运动太轻）→ 建议用更剧烈的 bag 重标；'
                  '配置暂保持原值')
            restore_original(original)
    else:
        print('\n  ⚠️ 有效结果不足 2 个，无法比较；配置已还原')
        restore_original(original)
    print(f'\n地图文件都留在 {DEFAULT_MAP.parent}/scan_offset_*.pcd，可用 render_map.py 逐个出图对比')
    return 0


if __name__ == '__main__':
    sys.exit(main())
