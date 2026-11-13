#!/usr/bin/env bash
# 把香橙派上的最新定位图（PNG）与实机扫描（PCD）拉到笔记本，并以【月日_时分秒】命名保存到 Windows 桌面，绝不覆盖历史结果
#
# 用法:
#   bash ~/scripts/fetch_pi_maps.sh
set +u

PI_HOST="${PI_HOST:-orangepi@10.207.70.100}"
PI_PW="${PI_PW:-orangepi}"
LOCAL=~/maps/pi
WIN=/mnt/c/Users/WGK/Desktop/pi_maps
mkdir -p "$LOCAL" "$WIN"

# 当前时间戳：月日_时分秒 (例如 1025_120730)，确保同分钟内多次拉取也绝不覆盖
TS="$(date +%m%d_%H%M%S)"

# 用 pexpect 驱动 scp，避免装 sshpass
scp_pw() {
  python3 - "$@" <<'PY'
import sys, pexpect
PW = 'orangepi'
cmd = ['scp', '-o', 'StrictHostKeyChecking=no', '-o', 'UserKnownHostsFile=/dev/null',
       '-o', 'LogLevel=ERROR'] + sys.argv[1:]
ch = pexpect.spawn(' '.join(cmd), encoding='utf-8', timeout=1800, codec_errors='replace')
for _ in range(4):
    i = ch.expect([r'(?i)password:', pexpect.EOF, pexpect.TIMEOUT], timeout=60)
    if i == 0:
        ch.sendline(PW); continue
    break
try:
    ch.expect(pexpect.EOF, timeout=1800)
except pexpect.TIMEOUT:
    print('超时')
out = (ch.before or '').strip()
ch.close(force=True)
if out:
    print(out)
PY
}

LOC_IMG="reloc_${TS}.png"
PROC_IMG="reloc_process_${TS}.png"
SCAN_PCD="scan_${TS}.pcd"

echo "[1/3] 从香橙派拉取最新定位图像..."
scp_pw "$PI_HOST:~/maps/real_robot_reloc_location.png" "$LOCAL/$LOC_IMG" 2>/dev/null || echo "  ⚠️ 未找到 real_robot_reloc_location.png"
scp_pw "$PI_HOST:~/maps/reloc_process_result.png" "$LOCAL/$PROC_IMG" 2>/dev/null || echo "  ⚠️ 未找到 reloc_process_result.png"

echo "[2/3] 从香橙派拉取当次实机雷达点云..."
scp_pw "$PI_HOST:~/maps/latest_real_scan.pcd" "$LOCAL/$SCAN_PCD" 2>/dev/null || true

echo "[3/3] 保存带时间戳的新图像到 Windows 桌面 (不覆盖历史图片)..."
[ -f "$LOCAL/$LOC_IMG" ] && cp -f "$LOCAL/$LOC_IMG" "$WIN/$LOC_IMG"
[ -f "$LOCAL/$PROC_IMG" ] && cp -f "$LOCAL/$PROC_IMG" "$WIN/$PROC_IMG"

echo "=================================================================="
echo "  ✅ 本次拉取完成 (时间戳: ${TS}，历史图片均完整保留)！"
echo "  📂 Windows 桌面目录: C:\\Users\\WGK\\Desktop\\pi_maps"
echo ""
echo "  ★ 本次最终位置大图 : $LOC_IMG"
echo "  · 本次双栏诊断详图 : $PROC_IMG"
echo "------------------------------------------------------------------"
echo "  🕒 最近生成的 6 张定位图片 (按时间从新到旧):"
ls -lt --time-style=+%m-%d_%H:%M:%S "$WIN"/reloc_*.png 2>/dev/null | head -6 | awk '{print "     " $6 "  ->  " $7 " (" $5 ")"}'
echo "=================================================================="
