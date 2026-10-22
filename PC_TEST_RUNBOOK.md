# PC 测试手册：VLP-16 + 超核 HI12 + FAST-LIO2

> 工作区：`~/fast_lio_ws`　配置：`src/FAST_LIO/config/velodyne.yaml`、`velodyne_driver.yaml`
> 工具：`~/fast_lio_ws/tools/*`　手册：本文件

## 📍 当前进度（2026-10-17）

| 步骤 | 状态 | 说明 |
|---|---|---|
| §0 环境 | ✅ | `ros_env.sh` 清 `/mnt/` 污染变量 |
| §1 一键启动 | ✅ | 5 节点链路（驱动→transform→规范化→IMU→建图）已跑通 |
| §2 验收 | ✅ | 点云 9.86 Hz / IMU 194 Hz / 静止漂移 0.007 m |
| §3 外参 | ✅ | `extrinsic_R` = 单位阵、`extrinsic_T` = `[0, 0, -0.0455]` |
| §4 运动测试 | ◐ | 已出干净地图（121373 点）；**`time_offset_lidar_to_imu` 仍是 0.0** |
| §5 录包 | ◐ | 静态/慢速包 `pc_base2` 已录；**动态包缺**（P95 \|ω\| = 0.064 rad/s，不够标偏移） |
| §6 IMU 200 Hz | ✅ | 194 Hz / 230400 波特率 |
| 正式建图出图 | ⬜ | 等时间偏移标定完再做 |

**下一步（唯一阻塞项）**：录动态包（§5.1）→ `scan_time_offset.py` 扫描 → 填 `time_offset_lidar_to_imu` → 正式建图 `/map_save` → `render_map.py`

---

---

## 0. 每个新终端第一件事

```bash
source ~/fast_lio_ws/ros_env.sh
```
（它清掉 `/mnt/d/...` 那些污染变量，再加载 ROS Humble + 本工作区；会打印 `fast_lio`、`hipnuc_imu` 路径）

---

## 1. 一键启动全套

```bash
source ~/fast_lio_ws/ros_env.sh
ros2 launch fast_lio velodyne_hi12_bringup.launch.py \
  start_imu:=true imu_port:=/dev/ttyUSB0 rviz:=true   # IMU 波特率默认已是 230400
```

**期望日志**
- `velodyne_driver_node`：`rotating at 600.000000 RPM` / `publishing 76 packets per scan` / `expected frequency: 9.921 (Hz)` / `Setting velodyne scan start time to timestamp of first packet`
- `fastlio_mapping`：`IMU Initial Done` → `Initialize the map kdtree`，`p_pre->lidar_type 2`
- RViz 弹出窗口（WSLg）

**只想起部分**：`start_imu:=false`（只雷达）、`rviz:=false`（不开 RViz）

---

## 2. 验收（另开一个终端）

```bash
source ~/fast_lio_ws/ros_env.sh
python3 ~/fast_lio_ws/tools/check_pipeline.py --duration 30
```

**通过标准**
| 项 | 目标 |
|---|---|
| 点云帧率 | ≈10.35 Hz（雷达 600rpm） |
| 点云字段 | 含 `time`，且 `time` 范围 ≈ [0, 0.10] s（**全为正**） |
| IMU 帧率 | ≈194Hz（实测已达标） |
| IMU \|a\| | ≈9.8 m/s²（不是 g 制） |
| `/Odometry` 帧率 | ≈雷达帧率（实时） |
| 静止漂移 | <0.3 m |

**单点排查工具**
```bash
python3 tools/check_point_time.py                  # 只看点云字段/time
python3 tools/imu_live.py 60                       # 实时看 IMU 数值
curl --noproxy '*' -s http://192.168.1.201/cgi/settings.json   # 读雷达配置
curl --noproxy '*' -s http://192.168.1.201/cgi/status.json     # 读雷达转速/激光
```
> 雷达 HTTP 一定要加 `--noproxy '*'`，否则会走 Clash 代理拿到 502。

---

## 3. 外参标定（唯一需要手工的一步）

**已标定完成**：`extrinsic_T: [0, 0, -0.0455]`（IMU 在雷达上方 45.5mm）、`extrinsic_R: 单位阵`。
本节保留作为复标流程参考；换安装方式时重做。

### 3.1 平移 extrinsic_T（用尺子量）
量 **LiDAR 中心 → IMU 中心** 的偏移（米），坐标系按 **IMU 机体轴**：
```bash
# 例：IMU 在雷达下方 3cm、后方 2cm
python3 tools/set_extrinsic.py --t -0.02 0.00 -0.03
```

### 3.2 旋转 extrinsic_R（低头 + 旋转测试）
```bash
python3 tools/imu_live.py 60
```
做两个动作，看输出：

**动作 1：低头** —— 让 LiDAR 的 x 方向（出线口/正面）朝下倾斜约 30°
- 哪个分量明显**变负**，那个分量就是 IMU 的"前方"轴

**动作 2：俯视逆时针转** —— 看 `wz`

**决策表**（`extrinsic_R` 含义：把 LiDAR 系向量转到 IMU 系，即 `v_imu = R · v_lidar`）

| 低头时变负的分量 | 俯视逆时针 wz | IMU 轴系 | extrinsic_R（按行） |
|---|---|---|---|
| `ax` | > 0 | x前 y左 z上（与雷达同） | `1,0,0, 0,1,0, 0,0,1` |
| `ay` | > 0 | x右 y前 z上 | `0,-1,0, 1,0,0, 0,0,1` |
| `ax` 变**正** | > 0 | x后 y左 z上（180°绕z） | `-1,0,0, 0,-1,0, 0,0,1` |
| `ay` 变**正** | > 0 | x左 y后 z上 | `0,1,0, -1,0,0, 0,0,1` |

> 静止时 `az ≈ +9.8` 已经确认 **z 朝上**，所以只需在这 4 种里选。
> 如果低头时是 **`az`** 变化（说明 z 朝前，安装很怪），或 wz 为负（左手系），**把 `imu_live.py` 输出发我**，我来算。

### 3.3 写入并重启

```bash
python3 tools/set_extrinsic.py --r 0,-1,0, 1,0,0, 0,0,1     # 举例
# 然后 Ctrl-C 掉 §1 的 launch，重新执行 §1
```

---

## 4. 运动测试（判外参对不对）

RVIz 窗口里看 `/cloud_registered`（全局地图）：

| 操作 | 正确表现 | 错误表现 → 原因 |
|---|---|---|
| 原地缓慢转一圈 | 墙钉在原地，只是补上新区域 | **双层墙 / 墙重影** → 外参 R 或 `time_offset_lidar_to_imu` 不对 |
| 缓慢走一圈回到起点 | 回到起点时误差小、墙是单层 | **香蕉形弯曲 / 整体倾斜** → 外参 T 或 IMU 噪声参数不对 |
| 快速晃动 | 点云仍贴合 | 拖影严重 → IMU 帧率太低（去 §6 提到 200Hz） |

---

## 5. 录包（推香橙派要用）

```bash
source ~/fast_lio_ws/ros_env.sh
mkdir -p ~/bags
ros2 bag record -o ~/bags/pc_run_$(date +%m%d_%H%M) /velodyne_points /imu/data
# 录 30~60s（转圈 + 走直线），Ctrl-C 结束
ros2 bag info ~/bags/pc_run_xxxx        # 确认两个话题都有消息
```

**回放验证**（**不要**同时开雷达/IMU 驱动，否则话题冲突）
```bash
# 终端 1 —— 回放（★ 不要加 --clock）
ros2 bag play ~/bags/pc_run_xxxx --rate 1
# 终端 2 —— 时间戳规范化节点（点云与 IMU 都用墙钟重打戳）
source ~/fast_lio_ws/ros_env.sh
python3 ~/fast_lio_ws/tools/timestamp_normalizer.py
# 终端 3 —— FAST-LIO（★ 不要加 use_sim_time）
ros2 launch fast_lio mapping.launch.py config_file:=velodyne.yaml rviz:=true
```

> ⚠️ **回放为什么不用 `--clock` / `use_sim_time`**
> VLP-16 没接 GPS/PPS，其时间戳带着雷达内部时钟的跳变（实测 100 秒倒退 6 次、最大 1.75 秒），
> 会让 FAST-LIO 打 `lidar loop back, clear buffer`。`--clock` 的模拟时钟会**继承这些跳变**，
> 规范化节点跟着跳 = 白修。统一走**墙钟**后，实时与回放行为一致，PC 调好的参数搬到香橙派可直接对比。
> （实测：规范化前 loop back 31 次 → 规范化后 **0 次**）

---

### 5.1 动态包（标定 `time_offset_lidar_to_imu` 必需）

`pc_base2` 是慢速转圈（P95 \|ω\| = **0.064 rad/s**），**不足以标定时间偏移**：
FAST-LIO 的畸变补偿量 ≈ `|ω| × offset`，ω 太小时补偿量淹没在噪声里，扫描结果没有区分度。

**录制要求**

终端 1 —— 只起驱动，**不起 FAST-LIO**（`start_mapping:=false`）：
剧烈运动下 FAST-LIO 初始化会失败刷屏，且白占 CPU 影响录包：

```bash
source ~/fast_lio_ws/ros_env.sh
ros2 launch fast_lio velodyne_hi12_bringup.launch.py \
  start_mapping:=false rviz:=false
```

终端 2 —— 录包：

```bash
source ~/fast_lio_ws/ros_env.sh
mkdir -p ~/bags
ros2 bag record -o ~/bags/dyn_$(date +%m%d_%H%M) /velodyne_points /imu/data
```

手持雷达+IMU 组合体（或让狗做动作），**25~30 秒**：

0. **★ 按下录制后先别动，静止数 3~5 秒** —— 这是硬性要求，不是可选项
   FAST-LIO 用第一帧雷达对应的 ~20 个 IMU 样本（约 100 ms）初始化
   （`IMU_Processing.hpp:193-196`）：
   - `grav = 加速度均值方向` → 开头在动 → 重力方向估歪 → 地图整体倾斜
   - `bg = 角速度均值`（陀螺零偏）→ 开头在动 → **把运动当成零偏，之后一直减掉**
   开头在动的话，每个候选 offset 都错得一模一样，扫描毫无区分度。
1. **原地快速左右转**：|ω| 要达到 **1~2 rad/s**（≈ 每秒 60~115°），左右各 3~5 次
2. 叠加 **±2~3 m 平移**：前后/左右来回 2~3 次
3. 动作**连续平顺**，不要急停急起（避免 IMU 陀螺饱和削顶）
4. 全程留在**开阔处**，别撞墙（撞了会污染点云）
5. 收尾**再静止 2~3 秒**再按 Ctrl-C（方便事后判断尾部）

**录完先验证运动强度**

```bash
ros2 bag info ~/bags/dyn_xxxx          # /velodyne_points 和 /imu/data 都要有消息
python3 ~/fast_lio_ws/tools/scan_time_offset.py --bag ~/bags/dyn_xxxx --check-only
```

`--check-only` 会打印两项，**两个都要过**：

| 检查项 | 目标 | 不过的后果 |
|---|---|---|
| 运动强度 P95 \|ω\| | **>0.5 rad/s** | 涂抹信号淹在噪声里，最优值不尖锐 |
| 开头 3s 静止占比 | **≥60%** | FAST-LIO 初始化估错零偏，整段全错 |

脚本发现开头不静止时会**直接拒绝扫描**（退出码 2），避免白跑 7 分钟。

**通过后跑扫描**

```bash
python3 ~/fast_lio_ws/tools/scan_time_offset.py --bag ~/bags/dyn_xxxx \
  --offsets -0.10 -0.05 -0.02 0.0 0.02 0.05 0.10
```

自动挑最优值写回 `config/velodyne.yaml` 的 `time_offset_lidar_to_imu`；
若几个候选的指标打平（说明 ω 还是不够大），脚本会拒绝写入并恢复原配置。

---

## 6. （可选）把 IMU 提到 200Hz

**已完成**：IMU 已提到 200Hz（实测 194Hz），波特率 230400。以下保留作为重做步骤。

1. Windows PowerShell（管理员）：`usbipd detach --busid 1-2`
2. Windows 里用 **CHCenter** 连上该 COM 口：输出率设 **200Hz**、波特率设 **230400**（115200 带宽只有 ~11.5KB/s，200Hz 会溢出）
3. Windows PowerShell：`usbipd attach --wsl --busid 1-2`
4. 启动时改参数：`imu_baudrate:=230400`

> 之后想改回 Windows 用，随时 `usbipd detach --busid 1-2`。

---

## 7. 停止所有程序

```bash
bash ~/fast_lio_ws/tools/stop_all.sh
```

---

## 8. 故障对照表

| 现象 | 检查命令 | 原因 / 处理 |
|---|---|---|
| 点云没有数据 | `ros2 topic hz /velodyne_points` | 雷达没上电/网线松；`ping 192.168.1.201` |
| 驱动日志刷 `Velodyne poll() timeout` | — | 雷达输出流中断（改过设置/重启雷达）→ 重启驱动 |
| 每帧只有 ~90°、帧率 40Hz | `curl --noproxy '*' .../cgi/settings.json` | 雷达是 `returns: Dual`（双回波）→ **必须改成 Strongest**；顺便确认 `rpm` 与驱动 `rpm` 一致 |
| `time` 出现负值 | `python3 tools/check_point_time.py` | 驱动 `timestamp_first_packet` 未生效（应为 true） |
| `time` 最大值远大于 0.1 | 同上 | `timestamp_unit` 不对（Velodyne 应为 0=秒） |
| 点云只有部分 ring | 同上 | 双回波没关，或 `scan_line` 不等于 16 |
| `/imu/data` 没数据 | `ros2 topic list` | 串口没 attach / 不在 dialout 组 / 波特率错 |
| IMU \|a\|≈1.0 | `python3 tools/imu_live.py` | 设备输出 g 制 → CHCenter 改成 m/s² |
| 建图飘 / 双层墙 | RViz | 外参没标（§3）、`time_offset_lidar_to_imu` 需微调 |
| `colcon build` 报错 | — | 先 `source ~/fast_lio_ws/ros_env.sh` 清环境 |
| **SSH 突然全断、ping 不通** | `ping <派IP>` | ★ 检查是不是自己关了网络：`nmcli radio wifi off` 会剪断你自己的 SSH，而且**重启不恢复**（状态存在 `/var/lib/NetworkManager/NetworkManager.state`）。恢复：接 HDMI 键鼠执行 `sudo nmcli radio wifi on`，或用网线把派 eth0 直连笔记本（笔记本配同网段静态 IP）后 SSH 进去 |

---

## 9. 工具清单（都在 `~/fast_lio_ws/tools/`）

| 文件 | 用途 |
|---|---|
| `check_pipeline.py` | 整机验收单（点云+IMU+里程计一次全查） |
| `check_point_time.py` | 只查点云字段与逐点时间契约 |
| `imu_live.py` | 实时打印 IMU，用于外参轴系判定 |
| `set_extrinsic.py` | 写入 extrinsic_T / extrinsic_R 到 yaml |
| `synthetic_sensor_pub.py` | 无硬件合成数据（`--az-steps` / `--scan-hz` 可调） |
| `bench_fastlio.sh` | 压测 FAST-LIO CPU 与实时性 |
| `run_imu.sh` | 启动 IMU 驱动（自动处理 dialout 权限） |
| `stop_all.sh` | 一键停止所有相关程序 |
| `ros_env.sh`（上一级目录） | 干净环境脚本 |
