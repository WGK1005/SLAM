#!/usr/bin/env bash
# =============================================================================
#  check_env.sh —— FAST-LIO + VLP-16 + HI12 环境体检（只读，不装任何东西）
#
#  用法:
#     bash check_env.sh          # 体检并汇总
#     bash check_env.sh -v       # 逐个列出每个 apt 包的状态
#     bash check_env.sh -q       # 只输出失败/警告
#
#  退出码: 0 = 全部通过（可能有警告）  1 = 存在致命缺失
# =============================================================================

set -uo pipefail

VERBOSE=0
QUIET=0
for a in "$@"; do
  case "$a" in
    -v|--verbose) VERBOSE=1 ;;
    -q|--quiet)   QUIET=1 ;;
    -h|--help)    sed -n '2,12p' "$0"; exit 0 ;;
  esac
done

if [[ -t 1 ]]; then
  C_OK=$'\033[32m'; C_WARN=$'\033[33m'; C_ERR=$'\033[31m'
  C_HL=$'\033[36m'; C_DIM=$'\033[2m'; C_RST=$'\033[0m'
else
  C_OK=''; C_WARN=''; C_ERR=''; C_HL=''; C_DIM=''; C_RST=''
fi

PASS=0; WARN=0; FAIL=0
WARNED=(); FAILED=()

step() { (( QUIET )) || { echo; echo "${C_HL}── $* ──${C_RST}"; }; }
ok()   { PASS=$((PASS+1)); (( QUIET )) || printf '  %s✓%s %s\n' "$C_OK" "$C_RST" "$*"; }
warn() { WARN=$((WARN+1)); WARNED+=("$1"); printf '  %s!%s %s\n' "$C_WARN" "$C_RST" "$*"; }
fail() { FAIL=$((FAIL+1)); FAILED+=("$1"); printf '  %s✗%s %s\n' "$C_ERR" "$C_RST" "$*"; }
info() { (( QUIET )) || printf '    %s%s%s\n' "$C_DIM" "$*" "$C_RST"; }

# 长路径截断，避免汇总刷屏
trunc() {
  local s="$1" n="${2:-70}"
  if (( ${#s} > n )); then printf '%s…(+%d字符)' "${s:0:n}" "$(( ${#s} - n ))"; else printf '%s' "$s"; fi
}

# 一组 apt 包：全齐 → ✓ ；否则 ✗ 并列出缺的
check_group() {
  local name="$1"; shift
  local total=$# p missing=()
  for p in "$@"; do
    if dpkg -s "$p" >/dev/null 2>&1; then
      (( VERBOSE )) && ok "$p"
    else
      missing+=("$p")
    fi
  done
  if (( ${#missing[@]} == 0 )); then
    ok "$name ($total/$total)"
  else
    fail "$name 缺 ${#missing[@]}/$total: ${missing[*]}"
  fi
}

# =============================================================== 0. 系统
step "0. 系统与硬件"
if [[ -r /etc/os-release ]]; then
  . /etc/os-release
  if [[ "${ID:-}" == "ubuntu" && "${VERSION_ID:-}" == "22.04" ]]; then
    ok "系统 $PRETTY_NAME"
  else
    fail "系统 ${PRETTY_NAME:-未知} —— 需要 Ubuntu 22.04（Humble 只支持 22.04）"
  fi
else
  fail "无法识别发行版"
fi

ARCH=$(uname -m)
case "$ARCH" in
  x86_64) ok "架构 $ARCH（小主机 / WSL 正常路径）" ;;
  aarch64|arm64) warn "架构 $ARCH —— 这是 Raspberry Pi / Orange Pi 路径，包名相同但需确认源" ;;
  *) fail "架构 $ARCH 未验证" ;;
esac

if grep -qi microsoft /proc/version 2>/dev/null; then
  info "运行在 WSL 中（串口需 usbipd-win 透传）"
  WSL=1
else
  WSL=0
fi

KERNEL=$(uname -r); info "内核 $KERNEL"
MEM_GB=$(awk '/MemTotal/{printf "%d", $2/1024/1024}' /proc/meminfo)
(( MEM_GB >= 4 )) && ok "内存 ${MEM_GB} GB" || warn "内存仅 ${MEM_GB} GB，建图建议 ≥4 GB"
AVAIL_GB=$(df -BG --output=avail "$HOME" 2>/dev/null | tail -1 | tr -dc '0-9')
(( AVAIL_GB >= 10 )) && ok "家目录剩余 ${AVAIL_GB} GB" || warn "剩余仅 ${AVAIL_GB} GB，建议 ≥10 GB"
CORES=$(nproc); info "CPU 核心 $CORES"

# =============================================================== 1. ROS 2
step "1. ROS 2 Humble"
if [[ -d /opt/ros/humble ]]; then
  ok "/opt/ros/humble 已安装 ($(du -sh /opt/ros/humble 2>/dev/null | cut -f1))"
  if [[ -f /opt/ros/humble/setup.bash ]]; then
    # 在子 shell 里 source，避免污染当前环境
    ROSV=$( bash -c 'source /opt/ros/humble/setup.bash 2>/dev/null
      echo "ROS_VERSION=$ROS_VERSION  ROS_DISTRO=$ROS_DISTRO"
      echo "已注册包 $(ros2 pkg list 2>/dev/null | wc -l) 个"' )
    while read -r l; do [[ -n "$l" ]] && info "$l"; done <<<"$ROSV"
  fi
else
  fail "/opt/ros/humble 不存在 —— ROS 2 Humble 未安装"
fi

# ROS 源：官方叫 ros2.list，fishros 一键脚本叫 ros-fish.list，新版用 .sources 格式
ROS_SRC=$(grep -rlE "packages\.ros\.org/ros2|tuna\.tsinghua\.edu\.cn/ros2|ros-fish" \
            /etc/apt/sources.list.d/ /etc/apt/sources.list 2>/dev/null | head -1)
if [[ -n "$ROS_SRC" ]]; then
  ok "ros2 apt 源: $(basename "$ROS_SRC")"
  if grep -q "tuna.tsinghua" "$ROS_SRC" 2>/dev/null; then
    info "使用清华镜像"
  elif grep -q "packages.ros.org" "$ROS_SRC" 2>/dev/null; then
    info "使用官方源"
  fi
else
  warn "未找到 ros2 apt 源（后续 apt 装 ROS 包会失败）"
fi

[[ -f /etc/ros/rosdep/sources.list.d/20-default.list ]] && ok "rosdep 已初始化" \
  || warn "rosdep 未初始化（sudo rosdep init && rosdep update）"

# =============================================================== 2. apt 包
step "2. 工程 apt 依赖"

check_group "PCL / 点云" \
  ros-humble-pcl-ros ros-humble-pcl-conversions ros-humble-pcl-msgs

check_group "TF" \
  ros-humble-tf2 ros-humble-tf2-ros ros-humble-tf2-eigen \
  ros-humble-tf2-geometry-msgs ros-humble-tf2-tools

check_group "Velodyne VLP-16 驱动" \
  ros-humble-velodyne ros-humble-velodyne-driver ros-humble-velodyne-msgs \
  ros-humble-velodyne-pointcloud ros-humble-velodyne-laserscan \
  ros-humble-diagnostic-updater ros-humble-diagnostic-msgs

check_group "bag / 可视化" \
  ros-humble-rosbag2 ros-humble-rosbag2-storage-default-plugins \
  ros-humble-rviz2 ros-humble-rviz-visual-tools ros-humble-rqt-common-plugins

check_group "串口 (robot_serial)" ros-humble-serial-driver

check_group "Python 数值/画图" python3-numpy python3-matplotlib python3-scipy

check_group "构建工具" \
  python3-colcon-common-extensions python3-rosdep python3-pip

check_group "Nav2 规划层（可选）" \
  ros-humble-nav2-planner ros-humble-nav2-navfn-planner ros-humble-nav2-smac-planner \
  ros-humble-nav2-map-server ros-humble-nav2-costmap-2d \
  ros-humble-nav2-lifecycle-manager ros-humble-nav2-msgs

# =============================================================== 3. 系统库
step "3. 系统开发库"
check_group "编译工具链" build-essential cmake git

for h in /usr/include/eigen3/Eigen/Core; do
  [[ -f $h ]] && ok "Eigen3 头文件" || fail "缺 Eigen3 头文件 (libeigen3-dev)"
done
for h in /usr/include/pcl-1.12/pcl/pcl_config.h /usr/include/yaml-cpp/yaml.h; do
  case "$h" in
    *pcl*) [[ -f $h ]] && ok "PCL 头文件" || fail "缺 PCL 头文件 (libpcl-dev)" ;;
    *yaml*) [[ -f $h ]] && ok "yaml-cpp 头文件" || fail "缺 yaml-cpp 头文件 (libyaml-cpp-dev)" ;;
  esac
done

# =============================================================== 4. Python
step "4. Python 模块"
check_mod() {
  local m="$1" v
  v=$(python3 -c "import $m; print(getattr($m,'__version__','ok'))" 2>/dev/null)
  if [[ -n "$v" ]]; then ok "$m $v"; else fail "$m 导入失败"; fi
}
check_mod numpy
check_mod matplotlib
check_mod scipy

# ROS Python 模块需要 source 后才有
if bash -c 'source /opt/ros/humble/setup.bash 2>/dev/null; python3 -c "import rclpy, sensor_msgs_py" ' 2>/dev/null; then
  ok "rclpy / sensor_msgs_py"
else
  fail "rclpy / sensor_msgs_py 导入失败"
fi

# =============================================================== 5. 权限
step "5. 权限与设备"
if id -nG "$USER" | tr ' ' '\n' | grep -qx dialout; then
  ok "$USER 在 dialout 组（串口可用）"
else
  fail "$USER 不在 dialout 组 —— sudo usermod -aG dialout \$USER 后重新登录"
fi

TTY_LIST=$(ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null | tr '\n' ' ')
if [[ -n "$TTY_LIST" ]]; then
  ok "串口设备: $TTY_LIST"
  for t in $TTY_LIST; do
    [[ -r $t ]] && info "$t 可读" || warn "$t 不可读（权限或驱动）"
  done
else
  warn "未发现 /dev/ttyUSB*（IMU 没插 / 没透传）"
  (( WSL )) && info "WSL 需在 PowerShell 里 usbipd attach --wsl --busid <id>"
fi

if ls /dev/input/js* >/dev/null 2>&1; then
  info "发现手柄设备 /dev/input/js*"
fi

# =============================================================== 6. 环境变量
step "6. 环境变量污染检查"
POLLUTED=0
for v in LD_LIBRARY_PATH PYTHONPATH CMAKE_PREFIX_PATH PKG_CONFIG_PATH AMENT_PREFIX_PATH; do
  val="${!v:-}"
  if [[ -n "$val" ]] && grep -q "/mnt/" <<<"$val"; then
    warn "$v 含 Windows 路径: $(trunc "$val" 60)"; POLLUTED=1
  fi
done
(( POLLUTED )) && info "→ 用 ros_env.sh 启动可清除（source ~/fast_lio_ws/ros_env.sh）" \
                || ok "无 /mnt/ 路径污染"

info "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-未设置(默认0)}  RMW=${RMW_IMPLEMENTATION:-默认}  LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-未设置}"

# =============================================================== 7. 工作空间
step "7. 工作空间"
WS="$HOME/fast_lio_ws"
if [[ -d "$WS" ]]; then
  ok "$WS 存在"

  [[ -d "$WS/src/FAST_LIO" ]] && ok "src/FAST_LIO" || fail "缺 src/FAST_LIO"
  [[ -f "$WS/src/FAST_LIO/include/ikd-Tree/ikd_Tree.cpp" ]] && ok "ikd-Tree 子模块完整" \
    || fail "ikd-Tree 缺失 → git submodule update --init --recursive"

  [[ -d "$WS/src/products/c/hipnuc" ]] && ok "HiPNUC SDK (products/c/hipnuc)" \
    || fail "缺 products/c/hipnuc → hipnuc_imu 会 FATAL_ERROR"

  [[ -f "$WS/src/FAST_LIO/config/velodyne.yaml" ]] && ok "config/velodyne.yaml" || warn "缺 velodyne.yaml"
  [[ -f "$WS/src/FAST_LIO/launch/velodyne_hi12_bringup.launch.py" ]] && ok "launch/velodyne_hi12_bringup.launch.py" \
    || warn "缺总 launch 文件（可能只搬了上游代码）"
  [[ -f "$WS/ros_env.sh" ]] && ok "ros_env.sh" || warn "缺 ros_env.sh"

  # 已编译的包
  if [[ -f "$WS/install/setup.bash" ]]; then
    BUILT=$(ls "$WS/install" 2>/dev/null | grep -vE '^(COLCON_IGNORE|_local|local_setup|setup|\.)' | tr '\n' ' ')
    ok "已编译: $BUILT"
    # 关键可执行文件
    EXE_FAIL=0
    OUT=$( bash -c "source $WS/ros_env.sh >/dev/null 2>&1; ros2 pkg executables fast_lio 2>/dev/null" )
    [[ -n "$OUT" ]] && ok "fast_lio 可执行文件: $(echo $OUT)" || { fail "fast_lio 无可执行文件"; EXE_FAIL=1; }
    OUT=$( bash -c "source $WS/ros_env.sh >/dev/null 2>&1; ros2 pkg executables hipnuc_imu 2>/dev/null" )
    [[ -n "$OUT" ]] && ok "hipnuc_imu 可执行文件: $(echo $OUT)" || { fail "hipnuc_imu 无可执行文件"; EXE_FAIL=1; }
  else
    warn "未编译（无 install/setup.bash）—— 见 SETUP_X86.md §6"
  fi

  # tools
  if [[ -d "$WS/tools" ]]; then
    N=$(ls "$WS/tools"/*.py 2>/dev/null | wc -l)
    ok "tools/ $N 个脚本"
    [[ -f "$WS/tools/timestamp_normalizer.py" ]] || warn "缺 tools/timestamp_normalizer.py（launch 会用到）"
  else
    warn "缺 tools/ 目录"
  fi
else
  warn "$WS 不存在 —— 代码还没搬过来"
fi

# =============================================================== 8. 运行期
step "8. 运行期状态（若有节点在跑）"
if command -v ros2 >/dev/null 2>&1 || [[ -f /opt/ros/humble/setup.bash ]]; then
  NODES=$( bash -c 'source /opt/ros/humble/setup.bash 2>/dev/null; source "$HOME/fast_lio_ws/ros_env.sh" >/dev/null 2>&1; timeout 5 ros2 node list 2>/dev/null' )
  if [[ -n "$NODES" ]]; then
    ok "当前活跃节点:"
    while read -r n; do [[ -n "$n" ]] && info "$n"; done <<<"$NODES"

    TOPICS=$( bash -c 'source /opt/ros/humble/setup.bash 2>/dev/null; source "$HOME/fast_lio_ws/ros_env.sh" >/dev/null 2>&1; timeout 5 ros2 topic list 2>/dev/null' )
    for t in /Odometry /cloud_registered /Laser_map; do
      grep -qx "$t" <<<"$TOPICS" && info "话题 $t 存在"
    done

    # 交叉核对：有 IMU 节点但没有串口设备 → 多半是上次没退干净的残留进程
    if grep -q "hipnuc_serial" <<<"$NODES" && [[ -z "${TTY_LIST:-}" ]]; then
      warn "有 /hipnuc_serial 在跑但没有 /dev/ttyUSB*，疑似残留进程：ps -ef | grep hipnuc"
    fi
  else
    info "无活跃节点（ROS 未启动，属正常）"
  fi
else
  warn "ros2 命令不可用，跳过运行期检查"
fi

# =============================================================== 汇总
echo
echo "${C_HL}════════════════════ 汇总 ════════════════════${C_RST}"
printf '  %s通过 %d%s    %s警告 %d%s    %s失败 %d%s\n' \
  "$C_OK" "$PASS" "$C_RST" "$C_WARN" "$WARN" "$C_RST" "$C_ERR" "$FAIL" "$C_RST"

if (( ${#FAILED[@]} )); then
  echo
  echo "  ${C_ERR}必须处理:${C_RST}"
  for f in "${FAILED[@]}"; do echo "    · $f"; done
fi
if (( ${#WARNED[@]} )); then
  echo
  echo "  ${C_WARN}建议处理:${C_RST}"
  for w in "${WARNED[@]}"; do echo "    · $w"; done
fi

echo
if (( FAIL == 0 )); then
  echo "  ${C_OK}环境就绪${C_RST}"
  (( WARN )) && echo "  （有 $WARN 项警告，不影响主流程）"
  echo
  echo "  下一步:"
  echo "    source ~/fast_lio_ws/ros_env.sh"
  echo "    ros2 launch fast_lio velodyne_hi12_bringup.launch.py"
  exit 0
else
  echo "  ${C_ERR}环境不完整${C_RST} —— 修法:"
  echo "    bash ~/fast_lio_ws/setup_x86.sh --mirror tuna"
  echo "  详细说明见 SETUP_X86.md"
  exit 1
fi
