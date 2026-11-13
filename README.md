# SCU-TDT-V1 决策规划与轨迹优化套件（适配 VLP-16 + 超核 HI12 + 香橙派 4 Pro）

> **所在目录**：`/home/wgk/decision_planning/SCU-TDT-V1`（中文快捷路径：`~/决策规划/SCU-TDT-V1`）  
> **架构定位**：保持原 `fast_lio_ws`（Fast-LIO 建图 + GlobalMatcher V2 重定位）完全不变，独立构建面向实机硬件的 **“前端势场 A* + 后端 Minimum Snap 方形安全走廊优化 + SCURM 智能向自由空间脱困”** 决策规划系统。

---

## 1. 硬件适配与核心模块说明

| 模块层级 | 核心源码文件 | 适配与功能说明 |
| :--- | :--- | :--- |
| **先验点云转栅格** | [`tools/pcd_to_nav_grid.py`](file:///home/wgk/decision_planning/SCU-TDT-V1/tools/pcd_to_nav_grid.py) | 自动读取实机 3D 先验地图 `/home/wgk/maps/map_1021_1050.pcd`（`z_floor = -0.47m`），截取离地 `[0.22m, 1.85m]` 的全部实验台、机柜、立柱与外墙，生成 `0.05m` 分辨率 2D 导航栅格图 `maps/lab_map_1021.pgm`。 |
| **前端势场规划** | [`src/YAstar/yastar.cpp`](file:///home/wgk/decision_planning/SCU-TDT-V1/src/YAstar/yastar.cpp) | 基于快速符号距离场（SDF）构造人工势场 $\text{cost}(d) = 1 + \frac{0.65}{0.08 + d}$，强迫路径保持在实验室走廊正中央；内置 4 层 64 位位掩码（Mask），支持实时注入 VLP-16 动态障碍物；集成 DP + 视线碰撞检测极速化简。 |
| **后端轨迹优化** | [`src/MinimumSnapOsqp/minimumSnap.cpp`](file:///home/wgk/decision_planning/SCU-TDT-V1/src/MinimumSnapOsqp/minimumSnap.cpp)<br/>[`src/MinimumSnapOsqp/sfcSquare.cpp`](file:///home/wgk/decision_planning/SCU-TDT-V1/src/MinimumSnapOsqp/sfcSquare.cpp) | 为每个化简拐点自动膨胀出最大 `1.4m` 的**方形安全飞行走廊（Square SFC）**，采用内置零外部依赖的 **Eigen ADMM + KKT Polish 二次规划求解器**（[`include/OsqpEigen/OsqpEigen.h`](file:///home/wgk/decision_planning/SCU-TDT-V1/include/OsqpEigen/OsqpEigen.h)）与交替碰撞迭代，生成 100% 无碰撞的五阶平滑轨迹。 |
| **SCURM 智能脱困与跟踪** | [`include/scu_tdt_v1/scu_recovery_tracker.hpp`](file:///home/wgk/decision_planning/SCU-TDT-V1/include/scu_tdt_v1/scu_recovery_tracker.hpp) | 融合 SCURM **“向自由空间智能脱困（Toward-Free-Space Recovery）”**：当小车贴近实验台或受阻时，自动扫描 360° SDF 梯度场朝最开阔方向撤离；内置前馈+纯跟踪速度控制器（适配室内安全限速 `1.2 m/s`，支持差速与全向/小陀螺底盘）。 |
| **ROS 2 实时导航节点** | [`src/scu_tdt_nav_node.cpp`](file:///home/wgk/decision_planning/SCU-TDT-V1/src/scu_tdt_nav_node.cpp) | 对接 `/Odometry`（Fast-LIO）、`/initialpose`（GlobalMatcher V2）、`/velodyne_points_norm`（VLP-16 动态避障掩码）与 `/goal_pose`（RViz 2D Goal Pose），实时发布 `/cmd_vel` 与平滑轨迹。 |

---

## 2. 一键快速运行指令

### 方式 A：在实机实验室地图上运行规划与脱困测试（自动生成三栏高清诊断大图）
```bash
bash ~/decision_planning/SCU-TDT-V1/tools/run_scu_tdt_v1.sh
```
也可以自定义任意起点和终点坐标（单位：米，全局坐标系）：
```bash
bash ~/decision_planning/SCU-TDT-V1/tools/run_scu_tdt_v1.sh --start 0.031 0.063 --goal -1.10 9.60
```

### 方式 B：作为 ROS 2 节点编译并接入实机系统
```bash
source ~/fast_lio_ws/ros_env.sh
cd ~/decision_planning/SCU-TDT-V1
colcon build
source install/setup.bash
ros2 run scu_tdt_v1 scu_tdt_nav_node --ros-args --params-file config/scu_tdt_params.yaml
```

