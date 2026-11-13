#!/usr/bin/env bash
# ==============================================================================
#  SCU-TDT-V1: 一键启动 RMUC 2025 3D Gazebo 仿真 + VLP-16 & HI12 + RViz2 三层膨胀导航
#  用法：
#    bash /home/wgk/decision_planning/SCU-TDT-V1/tools/run_gazebo_rmuc.sh
#    bash /home/wgk/decision_planning/SCU-TDT-V1/tools/run_gazebo_rmuc.sh --demo-only  # 仅运行 TDT-Nav-Kit 原版 play 演示
# ==============================================================================
set -e

ROOT_DIR="/home/wgk/decision_planning/SCU-TDT-V1"
cd "$ROOT_DIR"

if [ "${1:-}" == "--demo-only" ]; then
    echo "🚀 运行 TDT-Nav-Kit 原版 RMUC 2024/2025 赛场规划演示 (play)..."
    "$ROOT_DIR/build/play" "${@:2}"
    exit 0
fi

source /opt/ros/humble/setup.bash

if [ ! -f "$ROOT_DIR/worlds/rmuc2025.world" ] || [ ! -f "$ROOT_DIR/maps/rmuc2025.pgm" ]; then
    echo "[1/3] 正在从 images/rmuc2025.png 生成 Gazebo 3D 赛场世界..."
    python3 -s "$ROOT_DIR/tools/generate_rmuc_gazebo_world.py"
fi

if [ ! -f "$ROOT_DIR/install/setup.bash" ]; then
    echo "[2/3] 正在使用 colcon 编译 ROS 2 节点 scu_tdt_nav_node..."
    colcon build --base-paths "$ROOT_DIR" --cmake-args -DCMAKE_BUILD_TYPE=Release
fi

source "$ROOT_DIR/install/setup.bash"

echo "===================================================================="
echo "  🏆 正在启动 RMUC 2025 3D Gazebo 赛场仿真 + RViz2 三层膨胀闭环导航"
echo "  · 3D 赛场世界 : $ROOT_DIR/worlds/rmuc2025.world (28m x 15m)"
echo "  · 传感器模型  : 16线 3D 激光雷达 (/velodyne_points) + HI12 IMU (/imu/data)"
echo "  · 交互方式    : 在 RViz2 工具栏点击【2D Goal Pose】即可指点任意目标实时导航！"
echo "===================================================================="

ros2 launch scu_tdt_v1 gazebo_rmuc_sim.launch.py "$@"

