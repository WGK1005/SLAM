# FAST-LIO + VLP-16 + HI12 环境配置指令（x86_64）

> 目标机：**Ubuntu 22.04 LTS x86_64** + **ROS 2 Humble**
> 与本机（WSL2 / Ubuntu 22.04.5 / x86_64）架构一致，所以下面的包名可以原样照抄。
> 一键脚本：`setup_x86.sh`（本文件是逐条说明 + 踩坑记录）

---

## 0. 先确认这三件事

```bash
lsb_release -ds          # 必须是 Ubuntu 22.04.x LTS
uname -m                 # 必须是 x86_64
df -h ~                  # 建议预留 ≥ 10 GB（ROS Humble 本体 876 MB，加 PCL 编译产物约 4 GB）
free -h                  # 建议 ≥ 4 GB，建图时 ≥ 6 GB 更稳
```

**必须是 22.04**。20.04 对应 Foxy、24.04 对应 Jazzy，Humble 装不上。

---

## 1. 系统基础依赖

```bash
sudo apt update && sudo apt upgrade -y

sudo apt install -y \
  build-essential cmake git wget curl \
  python3-pip python3-dev python3-venv \
  libeigen3-dev libpcl-dev libyaml-cpp-dev \
  software-properties-common usbutils

sudo add-apt-repository -y universe
sudo apt update
```

> `libpcl-dev` 会自动带 `libboost-*`。
> **不需要** Livox-SDK2 / `livox_ros_driver2`——本工程已剥离（`USE_LIVOX=OFF`）。

---

## 2. ROS 2 Humble

### 方式 A：官方源（有稳定国际网络）

```bash
sudo apt install -y curl gnupg lsb-release

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-humble-desktop ros-dev-tools
```

### 方式 B：清华镜像（国内推荐，快 10 倍以上）

```bash
sudo apt install -y curl gnupg lsb-release

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu jammy main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-humble-desktop ros-dev-tools
```

### rosdep 初始化（`rosdep install` 需要）

```bash
sudo rosdep init          # 若提示已存在，忽略
rosdep update
```

---

## 3. 本工程实际用到的 ROS 包

```bash
sudo apt install -y \
  `# —— PCL / 点云 ——` \
  ros-humble-pcl-ros ros-humble-pcl-conversions ros-humble-pcl-msgs \
  `# —— TF ——` \
  ros-humble-tf2 ros-humble-tf2-ros ros-humble-tf2-eigen \
  ros-humble-tf2-geometry-msgs ros-humble-tf2-tools \
  `# —— Velodyne VLP-16 驱动 ——` \
  ros-humble-velodyne ros-humble-velodyne-driver ros-humble-velodyne-msgs \
  ros-humble-velodyne-pointcloud ros-humble-velodyne-laserscan \
  ros-humble-diagnostic-updater ros-humble-diagnostic-msgs \
  `# —— 录制 / 回放 / 可视化 ——` \
  ros-humble-rosbag2 ros-humble-rosbag2-storage-default-plugins \
  ros-humble-rviz2 ros-humble-rviz-visual-tools ros-humble-rqt-common-plugins \
  `# —— 串口（狗的下位机通信 robot_serial 需要）——` \
  ros-humble-serial-driver \
  `# —— Python 数值 / 画图（tools/ 里的脚本用）——` \
  python3-numpy python3-matplotlib python3-scipy

# colcon / rosdep 构建工具（ros-dev-tools 已含，这里再保一遍）
sudo apt install -y python3-colcon-common-extensions python3-rosdep
```

### 导航阶段（路径规划）需要的 —— 现在装也行，省得以后再配

```bash
sudo apt install -y \
  ros-humble-nav2-bringup \
  ros-humble-nav2-planner ros-humble-nav2-navfn-planner ros-humble-nav2-smac-planner \
  ros-humble-nav2-theta-star-planner \
  ros-humble-nav2-map-server ros-humble-nav2-costmap-2d \
  ros-humble-nav2-lifecycle-manager ros-humble-nav2-msgs \
  ros-humble-nav2-simple-commander ros-humble-nav2-rviz-plugins \
  ros-humble-nav2-regulated-pure-pursuit-controller
```

> 注意：这些包在 **arm64 上同样有 apt 预编译二进制**，到时候推到 Orange Pi 上不用编译 Nav2——这也是我建议"只取规划层"而不是自研 A* 的额外理由。

---

## 4. 硬件权限（IMU 串口）

```bash
sudo usermod -aG dialout $USER
# 必须重新登录（或 newgrp dialout）才生效
groups | grep dialout
```

> 本机就是靠这个组才能开 `/dev/ttyUSB0`。之前踩过 `stty: Permission denied` 的坑。

---

## 5. 把代码搬过去

我们的改动**大部分还没 commit**（6 个已跟踪文件被改 + 4 个新文件未跟踪），所以**别只 clone 上游**，否则跑不起来。

### 本机打包（在**这台**机器执行）

```bash
cd ~
tar czf ~/fast_lio_ws_export.tgz \
  --exclude='fast_lio_ws/build' \
  --exclude='fast_lio_ws/install' \
  --exclude='fast_lio_ws/log' \
  --exclude='*/.git' \
  --exclude='__pycache__' \
  fast_lio_ws

# 地图和 bag 建议一起带上（可选，约 1.3 GB）
tar czf ~/bags_export.tgz bags maps
```

体积参考：`fast_lio_ws` 全量 354 MB，其中 `src/FAST_LIO/.git` 独占 **219 MB**。
排除 `.git` 后约 **103 MB**，工作树内容完全一致（未提交的改动都在文件里，不依赖 git）。
想保留历史就把 `--exclude='*/.git'` 去掉，代价是包大 3 倍。

拷到小主机后：

```bash
cd ~ && tar xzf fast_lio_ws_export.tgz && tar xzf bags_export.tgz
```

### 如果要从小主机重新拉源码（不搬 tar 包时）

```bash
mkdir -p ~/fast_lio_ws/src && cd ~/fast_lio_ws/src

# FAST-LIO：必须带 --recursive，否则缺 ikd-Tree 子模块
git clone -b ros2 --recursive https://github.com/Ericsii/FAST_LIO.git
cd FAST_LIO
git checkout 2fffc57          # 我们验证过的 commit
git submodule update --init --recursive

# HiPNUC SDK：hipnuc_imu 硬依赖完整的 products/c/hipnuc + products/c/serial
cd ~/fast_lio_ws/src
git clone https://github.com/hipnuc/products.git
cd products && git checkout 2676ffb

# 然后把本机的改动搬过来：见 SETUP_X86.md §5 的补丁导出
```

> **关键**：`hipnuc_imu/CMakeLists.txt` 第 12-19 行会去找 `src/products/c/hipnuc/CMakeLists.txt`，
> 找不到直接 `FATAL_ERROR`。所以 `products` 必须是**完整仓库**，不能只拷 `ros/ros2` 两个包。

本机导出补丁（在这台机器执行）：

```bash
cd ~/fast_lio_ws/src/FAST_LIO
git diff > ~/fast_lio_local.patch          # 6 个已改文件
git status --short                          # 4 个新文件需单独拷
#   config/velodyne_driver.yaml
#   config/velodyne_raw.yaml
#   launch/velodyne_hi12_bringup.launch.py
```

---

## 6. 编译

```bash
cd ~/fast_lio_ws
source /opt/ros/humble/setup.bash

# 装剩下的 rosdep 依赖（只扫我们真正用的两个目录，避开 products/ros/ros1 的 catkin 包）
rosdep install --from-paths src/FAST_LIO src/products/ros/ros2 \
  --ignore-src -r -y

# 编译
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

预期产出 3 个包：`fast_lio`、`hipnuc_imu`、`hipnuc_msgs`。

**`--symlink-install` 的副作用（我们踩过）**：改 YAML / launch **不用重编**，
但**新增文件不会被链接进 `install/`**——新增 `config/xxx.yaml` 或新 launch 之后必须重新
`colcon build --symlink-install`，否则运行时报
`Parameter file path is not a file`，然后因为默认参数走错分支而 SIGSEGV。

只要 Velodyne、不要 Livox 就**不用加任何 cmake 参数**（默认 `USE_LIVOX=OFF`）。
真要 Livox：

```bash
colcon build --symlink-install --cmake-args -DUSE_LIVOX=ON
```

---

## 7. 环境脚本

`ros_env.sh` 已经在工作空间里，直接拷过去即可。它做两件事：

1. 清掉外部工作空间的 `PKG_CONFIG_PATH / CMAKE_PREFIX_PATH / LD_LIBRARY_PATH / PYTHONPATH` 污染
2. 按正确顺序 source `/opt/ros/humble` + 本工作空间

```bash
source ~/fast_lio_ws/ros_env.sh
```

建议加到 `~/.bashrc` 末尾（注意：**不要**再往上面加别的 `source .../install/setup.bash`）：

```bash
grep -q 'fast_lio_ws/ros_env.sh' ~/.bashrc || \
  echo 'source ~/fast_lio_ws/ros_env.sh' >> ~/.bashrc
```

---

## 8. 验证清单

### 8.0 一条指令全查（推荐）

```bash
bash ~/fast_lio_ws/check_env.sh
```

只读体检，不装任何东西。检查 8 大项、约 40 个检查点，输出 `✓ / ! / ✗` 并给出汇总：

| 段 | 内容 |
|---|---|
| 0 | 系统版本、架构（x86_64 / arm64）、WSL 检测、内存、磁盘、核数 |
| 1 | `/opt/ros/humble`、ros2 源（含 fishros 的 `ros-fish.list`）、rosdep |
| 2 | 28 个工程 apt 包（分 8 组）+ Nav2 规划层 7 个 |
| 3 | 编译工具链、Eigen3 / PCL / yaml-cpp 头文件 |
| 4 | numpy / matplotlib / scipy / rclpy / sensor_msgs_py |
| 5 | dialout 组、`/dev/ttyUSB*`、手柄 |
| 6 | `LD_LIBRARY_PATH` 等是否被 `/mnt/` 污染、RMW / ROS_DOMAIN_ID |
| 7 | 工作空间：子模块、HiPNUC SDK、config/launch、已编译包、可执行文件、tools |
| 8 | 运行期：活跃节点 + 关键话题 + 残留进程交叉核对 |

```bash
bash check_env.sh -v    # 逐包列出每个 apt 包状态
bash check_env.sh -q    # 只显示失败/警告
echo $?                 # 0 = 就绪   1 = 有致命缺失
```

### 8.1 手工逐条验证

```bash
source ~/fast_lio_ws/ros_env.sh

# ① 两个包都能找到
ros2 pkg prefix fast_lio
ros2 pkg prefix hipnuc_imu

# ② 可执行文件在位
ros2 pkg executables fast_lio      # 应有 fast_lio / fastlio_mapping
ros2 pkg executables hipnuc_imu    # 应有 serial_node / can_node

# ③ Velodyne 驱动在
ros2 pkg executables velodyne_driver   # 应有 velodyne_driver_node

# ④ 离线画图（不需要硬件，验证 PCL/numpy/matplotlib 链路）
python3 ~/fast_lio_ws/tools/render_map.py \
  --pcd ~/maps/pc_base2.pcd --out /tmp/check.png
ls -l /tmp/check.png
```

全过就说明环境没问题，可以接硬件了。

---

## 9. 接硬件跑（回顾）

```bash
source ~/fast_lio_ws/ros_env.sh

# 一条命令起全套：驱动 → 时间戳规范化 → IMU → 建图
ros2 launch fast_lio velodyne_hi12_bringup.launch.py

# 关键默认值（已改好，不用传参）
#   imu_port:=/dev/ttyUSB0  imu_baudrate:=230400  config_file:=velodyne.yaml
#   fix_timestamp:=true  rviz:=true
```

**IMU 波特率是 230400，不是 115200** —— 之前默认 115200 时现象是
`bytes received but no valid frames`，很容易误判成接线问题。

---

## 10. WSL 特有配置（如果小主机上也用 WSL）

### 10.1 网络（DDS 跨进程通信异常时必改）

`C:\Users\<你>\.wslconfig`：

```ini
[wsl2]
networkingMode=mirrored
```

改完 `wsl --shutdown` 再进。

### 10.2 USB 串口透传（IMU）

Windows PowerShell（管理员）：

```powershell
usbipd list
usbipd bind --busid <BUSID>
usbipd attach --wsl --busid <BUSID>
```

WSL 里确认：

```bash
ls -l /dev/ttyUSB*
sudo usermod -aG dialout $USER   # 第一次要加
```

> 如果小主机是**原生 Ubuntu**，这一节完全跳过，插上就有 `/dev/ttyUSB0`。

---

## 11. 已知坑速查

| 现象 | 原因 | 处理 |
|---|---|---|
| `Cannot find source file: include/ikd-Tree/ikd_Tree.cpp` | clone 忘了 `--recursive` | `git submodule update --init --recursive` |
| `Parameter file path is not a file` → SIGSEGV | 新增 YAML 没重新 `colcon build --symlink-install` | 重编 |
| `Error parsing a event near line 1` | YAML 里写了 `/** */` C 风格注释 | 改成 `#` |
| IMU 无数据 | 波特率用的 115200 | `imu_baudrate:=230400` |
| `stty: Permission denied` | 不在 dialout 组 | `usermod -aG dialout` + 重新登录 |
| `/Odometry` 不发布 | 时间戳节点 QoS 不匹配（best-effort 发 / reliable 收） | 已在 `tools/timestamp_normalizer.py` 修好（reliable depth 100） |
| `PCDWriter: Input point cloud has no data` | `publish.map_en` 默认 false | 运行时传 `map_en:=true`（已在 `velodyne.yaml` 设好） |
| `No Effective Points!` | 场地太窄 + 体素太大 | 已调好 `filter_size_surf 0.2 / filter_size_map 0.3 / point_filter_num 1` |
| 建图时点云飞出去（几十米外） | 建图同时跑别的重负载 → 墙钟时间跳变 | **建图/回放时不要并行跑重任务** |

---

## 12. 一键脚本

```bash
bash ~/fast_lio_ws/setup_x86.sh              # 官方源
bash ~/fast_lio_ws/setup_x86.sh --mirror tuna # 清华源
```

脚本覆盖 §1 / §2 / §3 / §4，以及 §8 的自检；
**不**自动搬代码、**不**自动编译（那两步依赖你怎么传文件，见 §5、§6）。

配完之后体检：

```bash
bash ~/fast_lio_ws/check_env.sh        # 只读，约 40 个检查点
bash ~/fast_lio_ws/check_env.sh -q     # 只看问题
```

三个脚本的分工：

| 脚本 | 作用 | 是否改动系统 |
|---|---|---|
| `setup_x86.sh` | 装环境 | 是（apt / usermod） |
| `check_env.sh` | 体检诊断 | 否（只读） |
| `ros_env.sh` | 加载环境变量 | 否（只改当前 shell） |
