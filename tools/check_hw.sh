#!/usr/bin/env bash
# =============================================================================
#  check_hw.sh —— 硬件接线自检（雷达以太网 + IMU 串口）
#
#  每次上电/接线后跑一遍，全绿再启动建图。
#
#  用法:
#     bash check_hw.sh                      # 默认雷达 192.168.1.201
#     bash check_hw.sh --lidar-ip 192.168.1.201
#     bash check_hw.sh --tty /dev/ttyUSB0
#
#  退出码: 0 = 硬件就绪   1 = 有问题
# =============================================================================

set -uo pipefail

LIDAR_IP="192.168.1.201"
TTY=""
for a in "$@"; do
  case "$a" in
    --lidar-ip=*) LIDAR_IP="${a#*=}" ;;
    --tty=*)      TTY="${a#*=}" ;;
    -h|--help)    sed -n '2,13p' "$0"; exit 0 ;;
  esac
done
# 支持 "--lidar-ip 1.2.3.4" 形式
prev=""
for a in "$@"; do
  [[ "$prev" == "--lidar-ip" ]] && LIDAR_IP="$a"
  [[ "$prev" == "--tty" ]]      && TTY="$a"
  prev="$a"
done

if [[ -t 1 ]]; then
  C_OK=$'\033[32m'; C_WARN=$'\033[33m'; C_ERR=$'\033[31m'; C_HL=$'\033[36m'; C_RST=$'\033[0m'
else
  C_OK=''; C_WARN=''; C_ERR=''; C_HL=''; C_RST=''
fi
FAIL=0; WARN=0
step() { echo; echo "${C_HL}── $* ──${C_RST}"; }
ok()   { printf '  %s✓%s %s\n' "$C_OK" "$C_RST" "$*"; }
warn() { WARN=$((WARN+1)); printf '  %s!%s %s\n' "$C_WARN" "$C_RST" "$*"; }
bad()  { FAIL=$((FAIL+1)); printf '  %s✗%s %s\n' "$C_ERR" "$C_RST" "$*"; }

# ─────────────────────────────────────────────── 0. 残留进程
step "0. 残留 ROS 进程"
LEFT=$(pgrep -a -f "fastlio_mapping|velodyne_driver_node|velodyne_transform_node|serial_node|timestamp_normalizer|ros2 launch velodyne" 2>/dev/null)
if [[ -z "$LEFT" ]]; then
  ok "无残留"
else
  warn "有残留（会抢话题/端口），先清理: bash ~/fast_lio_ws/tools/stop_all.sh"
  echo "$LEFT" | sed 's/^/      /'
fi

# ─────────────────────────────────────────────── 1. 以太网
step "1. 雷达以太网 (目标 $LIDAR_IP)"
IFACE=$(ip -4 -o addr show 2>/dev/null | awk '$4 ~ /^192\.168\.1\./ {print $2" "$4}')
if [[ -n "$IFACE" ]]; then
  ok "接口已配 192.168.1.x: $IFACE"
else
  bad "没有任何接口配到 192.168.1.x 网段"
  echo "      Windows 侧要给接雷达的那个网口配静态 IP，例如 192.168.1.100 / 255.255.255.0"
  echo "      （WSL mirrored 模式下会自动继承）"
fi

# 链路状态
DOWN_IF=$(ip -br link 2>/dev/null | awk '$2=="DOWN"{print $1}' | tr '\n' ' ')
[[ -n "$DOWN_IF" ]] && echo "      注意：DOWN 的接口有 [$DOWN_IF]（网线插对了吗）"

if ping -c 2 -W 1 "$LIDAR_IP" >/dev/null 2>&1; then
  ok "ping $LIDAR_IP 通"
else
  bad "ping $LIDAR_IP 不通（雷达没上电 / 网线松 / 网段不对）"
fi

NEIGH=$(ip neigh 2>/dev/null | grep "^$LIDAR_IP ")
[[ -n "$NEIGH" ]] && echo "      ARP: $NEIGH"

# ─────────────────────────────────────────────── 2. 雷达 HTTP 配置
step "2. 雷达 HTTP 配置（关键：returns 必须是 Strongest）"
for cgi in status settings; do
  OUT=$(curl --noproxy '*' -s --max-time 4 "http://$LIDAR_IP/cgi/$cgi.json" 2>/dev/null)
  if [[ -z "$OUT" ]]; then
    bad "http://$LIDAR_IP/cgi/$cgi.json 无响应（连不上就查上一步）"
    continue
  fi
  ok "$cgi.json 可读"
  if command -v python3 >/dev/null 2>&1; then
    python3 - "$cgi" <<PY 2>/dev/null || echo "      (JSON 解析失败，原始前 200 字) ${OUT:0:200}"
import json, sys
raw = '''$OUT'''
try:
    d = json.loads(raw)
except Exception:
    print("      (JSON 解析失败)")
    sys.exit(0)

def walk(o, p=""):
    if isinstance(o, dict):
        for k, v in o.items():
            yield from walk(v, f"{p}.{k}" if p else k)
    elif isinstance(o, list):
        for i, v in enumerate(o):
            yield from walk(v, f"{p}[{i}]")
    else:
        yield p, o

for k, v in walk(d):
    lk = k.lower()
    if any(t in lk for t in ("rpm", "return", "laser", "fov", "pps", "gps", "serial", "ip")):
        print(f"      {k} = {v}")
        if "return" in lk and str(v).lower() not in ("strongest", "1", "true"):
            print("      \033[31m✗ returns 不是 Strongest → 每帧只有 ~90°，建图必崩\033[0m")
PY
  else
    echo "      $OUT" | head -c 300; echo
  fi
done

# ─────────────────────────────────────────────── 3. IMU 串口
step "3. IMU 串口"
if [[ -z "$TTY" ]]; then
  TTY=$(ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null | head -1)
fi

if [[ -z "$TTY" ]]; then
  bad "没找到 /dev/ttyUSB* —— IMU 没插 / usbipd 没 attach"
  if grep -qi microsoft /proc/version 2>/dev/null; then
    echo "      Windows PowerShell(管理员):"
    echo "        usbipd list"
    echo "        usbipd attach --wsl --busid <BUSID>"
  fi
else
  ok "串口设备: $TTY"
  ls -l "$TTY" | sed 's/^/      /'
  if id -nG "$USER" | tr ' ' '\n' | grep -qx dialout; then
    ok "$USER 在 dialout 组"
  else
    bad "$USER 不在 dialout 组 → sudo usermod -aG dialout \$USER 后重新登录"
  fi
  if [[ -r "$TTY" ]]; then
    ok "$TTY 可读"
  else
    warn "$TTY 当前不可读（组权限或设备占用）"
  fi
  # 波特率自检：读 2 秒看有没有数据
  N=$(sg dialout -c "timeout 2 cat $TTY 2>/dev/null | head -c 2000 | wc -c" 2>/dev/null || echo 0)
  if [[ "${N:-0}" -gt 100 ]]; then
    ok "2 秒内收到 ${N} 字节（设备在输出）"
  else
    warn "2 秒内只收到 ${N:-0} 字节（波特率/输出率设置不对？设备是否上电）"
  fi
fi

# ─────────────────────────────────────────────── 汇总
echo
echo "${C_HL}════════════ 汇总 ════════════${C_RST}"
if (( FAIL == 0 )); then
  printf '  %s硬件就绪%s' "$C_OK" "$C_RST"
  (( WARN )) && echo "（$WARN 项警告）" || echo
  echo
  echo "  下一步:"
  echo "    source ~/fast_lio_ws/ros_env.sh"
  echo "    ros2 launch fast_lio velodyne_hi12_bringup.launch.py"
  exit 0
else
  printf '  %s硬件未就绪%s —— %d 项失败\n' "$C_ERR" "$C_RST" "$FAIL"
  exit 1
fi
