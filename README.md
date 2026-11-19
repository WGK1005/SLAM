# VanJee WLR-716 2D 激光雷达 ROS 2 驱动与网络诊断工具集

[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/humble/)
[![C++](https://img.shields.io/badge/C%2B%2B-14%2F17-00599C?logo=cplusplus&logoColor=white)](https://isocpp.org/)
[![Protocol](https://img.shields.io/badge/Interface-TCP%202110%20%7C%20270%C2%B0%20FOV-blue)](#)

> 隶属仓库 [`WGK1005/SLAM`](https://github.com/WGK1005/SLAM) 的 **`wrl716`** 分支（3D 激光惯导 SLAM 主线请见 [`main`](https://github.com/WGK1005/SLAM/tree/main) 分支）。

针对万集（VanJee）**WLR-716** 2D 激光雷达的 **ROS 2 Humble** 驱动移植工作区，包含异步 TCP 通信解析节点、多网段主动扫描器与上电 ARP 抓包寻址工具，开箱发布标准 `sensor_msgs/msg/LaserScan` 话题并支持 RViz2 实时可视化。

---

## ⚙️ 雷达核心参数规格（默认配置）

配置见 [`ros2_ws/src/wj_716_lidar/config/wj_716_params.yaml`](ros2_ws/src/wj_716_lidar/config/wj_716_params.yaml)：

| 参数项 | 默认值 | 说明 |
| :--- | :--- | :--- |
| **网络协议 / 端口** | TCP `2110` (`192.168.100.104`) | 通过 `run_wlr716.sh` 支持命令行覆盖目标 IP 与端口 |
| **扫描视角** | `-135° ~ +135°`（`270°` FOV） | `min_ang = -2.3562 rad`, `max_ang = +2.3562 rad` |
| **角分辨率 / 点数** | `0.00582 rad` / `811` 点 | 帧坐标系 `frame_id: "laser"` |
| **有效测距范围** | `0.05 m ~ 25.0 m` | 兼顾近距离避障与中远距建图 |
| **输出 ROS 2 话题** | `/scan` & `/scan_TwoEcho` | 分别对应首重回波与双重回波 `LaserScan` |

---

## 📂 目录结构

```text
wrl716/
├── README.md                          # 说明文档（本文件）
├── ros2_ws/src/wj_716_lidar/          # ROS 2 Humble 移植版驱动包
│   ├── config/wj_716_params.yaml      # 雷达网络与扫描参数
│   ├── launch/wj_716_lidar.launch.py  # ROS 2 启动文件
│   └── src/
│       ├── async_client.cpp/.h        # Boost/异步 TCP Socket 客户端
│       ├── wj_716_lidar_protocol.cpp  # WLR-716 二进制协议帧解析
│       └── wj_716_lidar_node.cpp      # ROS 2 LaserScan 发布节点
├── WLR-716-ROS_DRIVER-master/         # 原版 ROS 1 驱动参考
├── ros_env.sh                         # 环境变量清理与工作区加载脚本
├── run_wlr716.sh                      # 雷达驱动一键启动脚本
├── scan_lidar.py                      # 多线程 TCP 雷达 IP 自动扫描器
├── sniff_lidar.sh                     # 上电 ARP/IP 抓包寻址脚本
└── wlr716.rviz                        # 预配置 RViz2 可视化视图
```

---

## 🚀 快速开始

### 1. 编译 ROS 2 驱动
```bash
source ros_env.sh
cd ros2_ws
colcon build --symlink-install --packages-select wj_716_lidar
```

### 2. 雷达网络未知 IP 排查（首次接线必备）
- **方法 A：多线程主动心跳探测**（向 `192.168.100.0/24`、`192.168.0.0/24`、`192.168.1.0/24` 的 `2110` 端口并发发送 34 字节激活命令）：
  ```bash
  python3 scan_lidar.py
  # 或指定网段
  python3 scan_lidar.py 192.168.100.0/24
  ```
- **方法 B：上电 ARP 抓包定位**（运行后将雷达断电重启即可捕获真实 IP/MAC）：
  ```bash
  sudo bash sniff_lidar.sh eth0
  ```

### 3. 启动驱动与 RViz2 可视化
```bash
# 使用默认 IP (192.168.100.104:2110)
bash run_wlr716.sh

# 或显式指定雷达 IP 与端口
bash run_wlr716.sh 192.168.0.2 2110

# 查看点云扫描与频率
ros2 topic hz /scan
rviz2 -d wlr716.rviz
```
