#!/usr/bin/env bash
# =============================================================================
#  FAST-LIO + Velodyne VLP-16 + HiPNUC HI12 环境一键配置
#  目标机: Ubuntu 22.04 LTS x86_64 + ROS 2 Humble
#  与本机 (WSL2 / Ubuntu 22.04.5 / x86_64) 架构一致，包名可直接照抄
#
#  用法:
#     bash setup_x86.sh                     # 官方 ROS 源
#     bash setup_x86.sh --mirror tuna       # 清华镜像（国内推荐）
#     bash setup_x86.sh --skip-nav2         # 不装导航相关包
#
#  覆盖范围: 系统依赖 / ROS2 Humble / 工程所需 ROS 包 / 串口权限 / 自检
#  不覆盖  : 搬代码、colcon build（见同目录 SETUP_X86.md §5 §6）
# =============================================================================

set -uo pipefail

MIRROR="official"
INSTALL_NAV2=1

for arg in "$@"; do
  case "$arg" in
    --mirror)   shift; MIRROR="${1:-official}" ;;
    --mirror=*) MIRROR="${arg#*=}" ;;
    --skip-nav2) INSTALL_NAV2=0 ;;
    -h|--help)  sed -n '2,14p' "$0"; exit 0 ;;
  esac
done

C_OK=$'\033[32m'; C_WARN=$'\033[33m'; C_ERR=$'\033[31m'; C_HL=$'\033[36m'; C_RST=$'\033[0m'
step() { echo; echo "${C_HL}==== $* ====${C_RST}"; }
ok()   { echo "  ${C_OK}✓${C_RST} $*"; }
warn() { echo "  ${C_WARN}!${C_RST} $*"; }
die()  { echo "  ${C_ERR}✗${C_RST} $*" >&2; exit 1; }

# ---------------------------------------------------------------- 0. 环境检查
step "0/7 检查系统"
[[ -r /etc/os-release ]] || die "无法读取 /etc/os-release"
. /etc/os-release
echo "  发行版 : $PRETTY_NAME"
echo "  架构   : $(uname -m)"

[[ "${ID:-}" == "ubuntu" ]] || die "只支持 Ubuntu"
[[ "${VERSION_ID:-}" == "22.04" ]] || die "需要 Ubuntu 22.04（Humble 只支持 22.04；20.04→Foxy，24.04→Jazzy）"
[[ "$(uname -m)" == "x86_64" ]] || warn "当前是 $(uname -m)，本脚本按 x86_64 写；arm64 包名相同但请确认源"
ok "系统匹配"

AVAIL_GB=$(df -BG --output=avail "$HOME" | tail -1 | tr -dc '0-9')
(( AVAIL_GB >= 10 )) || warn "剩余磁盘 ${AVAIL_GB}GB，建议 ≥10GB"
ok "磁盘剩余 ${AVAIL_GB}GB"

# ---------------------------------------------------------------- 1. 系统依赖
step "1/7 系统基础依赖"
sudo apt update
sudo apt install -y \
  build-essential cmake git wget curl \
  python3-pip python3-dev python3-venv \
  libeigen3-dev libpcl-dev libyaml-cpp-dev \
  software-properties-common usbutils gnupg lsb-release

sudo add-apt-repository -y universe
sudo apt update
ok "基础依赖完成"

# ---------------------------------------------------------------- 2. ROS 2 Humble
step "2/7 ROS 2 Humble（来源: $MIRROR）"

if [[ -d /opt/ros/humble ]]; then
  ok "已安装 (/opt/ros/humble)，跳过"
else
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg \
    || die "下载 ROS key 失败，检查网络或换 --mirror tuna"

  case "$MIRROR" in
    tuna|tsinghua|cn)
      echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu jammy main" \
        | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
      ok "使用清华镜像"
      ;;
    official|*)
      echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" \
        | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
      ok "使用官方源"
      ;;
  esac

  sudo apt update
  sudo apt install -y ros-humble-desktop ros-dev-tools || die "ROS 2 Humble 安装失败"
  ok "ROS 2 Humble 安装完成"
fi

step "2b/7 rosdep 初始化"
sudo rosdep init 2>/dev/null && ok "rosdep 已初始化" || warn "rosdep 已初始化过，忽略"
rosdep update || warn "rosdep update 失败（不致命，可稍后手动重试）"

# ---------------------------------------------------------------- 3. 工程 ROS 包
step "3/7 本工程所需 ROS 包"

PKGS=(
  # PCL / 点云
  ros-humble-pcl-ros ros-humble-pcl-conversions ros-humble-pcl-msgs
  # TF
  ros-humble-tf2 ros-humble-tf2-ros ros-humble-tf2-eigen
  ros-humble-tf2-geometry-msgs ros-humble-tf2-tools
  # Velodyne VLP-16
  ros-humble-velodyne ros-humble-velodyne-driver ros-humble-velodyne-msgs
  ros-humble-velodyne-pointcloud ros-humble-velodyne-laserscan
  ros-humble-diagnostic-updater ros-humble-diagnostic-msgs
  # bag / rviz
  ros-humble-rosbag2 ros-humble-rosbag2-storage-default-plugins
  ros-humble-rviz2 ros-humble-rviz-visual-tools ros-humble-rqt-common-plugins
  # 串口（robot_serial 下位机通信）
  ros-humble-serial-driver
  # Python 数值 / 画图（tools/ 脚本）
  python3-numpy python3-matplotlib python3-scipy
  # 构建工具
  python3-colcon-common-extensions python3-rosdep
)

sudo apt install -y "${PKGS[@]}" || die "工程依赖安装失败"
ok "工程依赖完成（${#PKGS[@]} 项）"

if (( INSTALL_NAV2 )); then
  step "3b/7 导航（路径规划）相关包"
  sudo apt install -y \
    ros-humble-nav2-bringup \
    ros-humble-nav2-planner ros-humble-nav2-navfn-planner ros-humble-nav2-smac-planner \
    ros-humble-nav2-theta-star-planner \
    ros-humble-nav2-map-server ros-humble-nav2-costmap-2d \
    ros-humble-nav2-lifecycle-manager ros-humble-nav2-msgs \
    ros-humble-nav2-simple-commander ros-humble-nav2-rviz-plugins \
    ros-humble-nav2-regulated-pure-pursuit-controller \
    && ok "Nav2 规划层完成" || warn "Nav2 安装失败（不影响建图）"
else
  warn "已跳过 Nav2 (--skip-nav2)"
fi

# ---------------------------------------------------------------- 4. 串口权限
step "4/7 IMU 串口权限"
if id -nG "$USER" | tr ' ' '\n' | grep -qx dialout; then
  ok "$USER 已在 dialout 组"
else
  sudo usermod -aG dialout "$USER"
  warn "已加入 dialout —— 必须重新登录（或 newgrp dialout）才生效"
fi

# ---------------------------------------------------------------- 5. 自检
step "5/7 依赖自检"
MISSING=()
for p in pcl_ros tf2_ros velodyne_driver serial_driver rviz2; do
  [[ -d "/opt/ros/humble/share/$p" ]] || MISSING+=("$p")
done
if (( ${#MISSING[@]} )); then
  die "缺失: ${MISSING[*]}"
fi
ok "关键 ROS 包齐全"

python3 - <<'PY' || die "Python 数值/画图链路异常"
import numpy, matplotlib
print(f"  \033[32m✓\033[0m numpy {numpy.__version__} / matplotlib {matplotlib.__version__}")
PY

# ---------------------------------------------------------------- 6. 工作空间自检
step "6/7 工作空间状态"
WS="$HOME/fast_lio_ws"
if [[ -d "$WS/src/FAST_LIO" ]]; then
  ok "找到 $WS"
  if [[ -f "$WS/src/FAST_LIO/include/ikd-Tree/ikd_Tree.cpp" ]]; then
    ok "ikd-Tree 子模块完整"
  else
    warn "ikd-Tree 缺失 → cd $WS/src/FAST_LIO && git submodule update --init --recursive"
  fi
  if [[ -d "$WS/src/products/c/hipnuc" ]]; then
    ok "HiPNUC SDK 完整 (products/c/hipnuc)"
  else
    warn "products/c/hipnuc 缺失 → hipnuc_imu 会 FATAL_ERROR，需完整 clone hipnuc/products"
  fi
  if [[ -f "$WS/install/setup.bash" ]]; then
    ok "已编译过 (install/ 存在)"
    NEED_BUILD=0
  else
    warn "尚未编译"
    NEED_BUILD=1
  fi
else
  warn "未找到 $WS —— 先按 SETUP_X86.md §5 把代码搬过来"
  NEED_BUILD=1
fi

# ---------------------------------------------------------------- 7. 下一步
step "7/7 下一步"
cat <<EOF
  环境已就绪。接下来：

  1) 搬代码（若还没搬，在旧机器上执行）：
       cd ~ && tar czf fast_lio_ws_export.tgz \\
         --exclude='fast_lio_ws/build' --exclude='fast_lio_ws/install' \\
         --exclude='fast_lio_ws/log' --exclude='__pycache__' fast_lio_ws

  2) 编译：
       cd ~/fast_lio_ws
       source /opt/ros/humble/setup.bash
       rosdep install --from-paths src/FAST_LIO src/products/ros/ros2 --ignore-src -r -y
       colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

  3) 加载环境：
       source ~/fast_lio_ws/ros_env.sh
       grep -q 'fast_lio_ws/ros_env.sh' ~/.bashrc || \\
         echo 'source ~/fast_lio_ws/ros_env.sh' >> ~/.bashrc

  4) 验证：
       ros2 pkg executables fast_lio        # → laserMapping
       ros2 pkg executables hipnuc_imu      # → serial_node / can_node
       ros2 pkg executables velodyne_driver # → velodyne_driver_node

  5) 接硬件跑：
       ros2 launch fast_lio velodyne_hi12_bringup.launch.py
       # IMU 波特率默认 230400（不是 115200！）

  ${C_WARN}串口权限改动需要重新登录终端${C_RST}
EOF
echo
ok "完成"
