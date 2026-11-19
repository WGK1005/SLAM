# Orange Pi 4 Pro 边缘端协同部署与结果同步脚本集

[![Python](https://img.shields.io/badge/Python-3.10%2B-3776AB?logo=python&logoColor=white)](https://www.python.org/)
[![Shell](https://img.shields.io/badge/Shell-Bash-4EAA25?logo=gnu-bash&logoColor=white)](#)
[![Target](https://img.shields.io/badge/Edge%20Device-Orange%20Pi%204%20Pro%20(ARM64)-orange)](#)

> 隶属仓库 [`WGK1005/SLAM`](https://github.com/WGK1005/SLAM) 的 **`scripts`** 分支（感知算法主线请见 [`main`](https://github.com/WGK1005/SLAM/tree/main) 分支）。

用于在 **上位机（WSL2 / Ubuntu）** 与 **边缘端香橙派（Orange Pi 4 Pro, ARM64）** 之间完成源码增量同步、远程 `colcon` 编译自检、无依赖 SSH/SCP 自动化运维，以及实机重定位结果图与点云扫描的**时间戳非覆盖拉取**。

---

## 🛠️ 脚本清单与功能

| 脚本文件 | 用途 | 核心特性 |
| :--- | :--- | :--- |
| [`sync_reloc_v2_to_pi.sh`](sync_reloc_v2_to_pi.sh) | **GlobalMatcher V2 一键同步与编译** | 将 `open3d_loc` 源码、绘图/压测工具及 `.df_v2.bin` 纯墙面距离场缓存同步至香橙派，清理旧版地面缓存，远程触发 `colcon build --packages-select open3d_loc` 并运行 `--offline --sim-yaw 137.5` 自检。 |
| [`fetch_pi_maps.sh`](fetch_pi_maps.sh) | **实机定位图与点云一键拉取** | 从香橙派拉取 `real_robot_reloc_location.png`、`reloc_process_result.png` 与 `latest_real_scan.pcd`，按 `MMdd_HHMMSS` 时间戳归档到本地与 Windows 桌面目录，绝不覆盖历史实验记录。 |
| [`pi.py`](pi.py) | **香橙派非交互 SSH 执行器** | 基于 `pexpect` 自动处理 SSH 与 `sudo` 交互，支持单行命令与 `--file` 本地脚本远程执行（无需额外安装 `sshpass`）。 |
| [`pi_scp.py`](pi_scp.py) | **香橙派非交互 SCP 传输器** | 透传标准 `scp` 参数，自动完成双向文件/目录传输。 |
| [`ssh_opi.py`](ssh_opi.py) | **香橙派交互/快捷连接助手** | 简化开发机到香橙派的快速远程命令调试。 |
| [`setup_pc.py`](setup_pc.py) | **SSH 公钥自动部署脚本** | 向目标主机自动写入 `ed25519` 公钥并配置 `~/.ssh/authorized_keys` 权限。 |

---

## 🚀 常用命令示例

### 1. 配置目标主机环境变量（按实际局域网 IP 覆盖）
```bash
export PI_HOST="orangepi@10.207.70.100"
export PI_PW="<your_password>"
```

### 2. 同步重定位算法到香橙派并远程编译自检
```bash
bash sync_reloc_v2_to_pi.sh
```

### 3. 实机定位完成后拉取结果图到桌面（带时间戳归档）
```bash
bash fetch_pi_maps.sh
```

### 4. 远程执行单条指令或上传测试脚本
```bash
python3 pi.py "source ~/fast_lio_ws/ros_env.sh && ros2 topic list"
python3 pi.py --file /path/to/local_check.sh
python3 pi_scp.py /path/to/local_file "$PI_HOST:~/fast_lio_ws/"
```

> **安全提示**：本分支已通过 `.gitignore` 忽略 `keys/` 目录，请勿将私钥或明文生产密码提交至公开仓库。
