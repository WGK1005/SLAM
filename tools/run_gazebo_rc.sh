#!/usr/bin/env bash
# ==============================================================================
#  SCU-TDT-V1: 一键启动 2027 足式机器人挑战赛 障碍赛 (14m x 6m, 1:1) Gazebo 仿真 + RViz2 导航
#  用法：
#    bash /home/wgk/decision_planning/SCU-TDT-V1/tools/run_gazebo_rc.sh
#    bash .../run_gazebo_rc.sh --plan-obstacles       # 运行 Go2 全场 8 障碍越障 + 梅花桩四足落足点规划并出图
#    bash .../run_gazebo_rc.sh --world-only           # 仅打开 Gazebo 查看 1:1 3D 赛场
#    bash .../run_gazebo_rc.sh start_zone:=2          # 从启动区2出生
#    bash .../run_gazebo_rc.sh gui:=false             # 不开 Gazebo 窗口
#    REGEN=1 FENCE=0.6 bash .../run_gazebo_rc.sh      # 重新生成场地并加 0.6m 围挡
# ==============================================================================
set -e

ROOT_DIR="/home/wgk/decision_planning/SCU-TDT-V1"
cd "$ROOT_DIR"

if [ "${REGEN:-0}" == "1" ] || [ ! -f "$ROOT_DIR/worlds/rc_obstacle_2027.world" ] || [ ! -f "$ROOT_DIR/maps/rc_obstacle_2027.pgm" ]; then
    echo "[1/3] 生成 RC 障碍赛 Gazebo 世界与导航栅格..."
    python3 -s "$ROOT_DIR/tools/generate_rc_obstacle_world.py" --fence "${FENCE:-0}"
fi

if [ "${1:-}" == "--plan-obstacles" ]; then
    echo "🐕 运行宇树 Go2 (VLP-16 + HI12) 障碍赛全场越障与梅花桩四足落足点规划..."
    cmake -S "$ROOT_DIR" -B "$ROOT_DIR/build" >/dev/null
    cmake --build "$ROOT_DIR/build" --target plan_rc_obstacles -j4 >/dev/null
    exec "$ROOT_DIR/build/plan_rc_obstacles"
fi

if [ "${1:-}" == "--world-only" ]; then
    echo "🐕 仅启动 Gazebo 查看 RC 2027 障碍赛 1:1 场地..."
    exec gazebo "$ROOT_DIR/worlds/rc_obstacle_2027.world"
fi

source /opt/ros/humble/setup.bash

echo "[2/3] 增量编译并更新 ROS 2 节点 scu_tdt_nav_node..."
colcon build --base-paths "$ROOT_DIR" --cmake-args -DCMAKE_BUILD_TYPE=Release

source "$ROOT_DIR/install/setup.bash"

echo "===================================================================="
echo "  🐕 2027 足式机器人挑战赛 障碍赛 Gazebo 仿真"
echo "  · 场地  : 14m x 6m，8 个障碍 (绕桩/限高框架/高墙/沟壑组合/砖木坑/轮胎阵/梅花桩/单木桥)"
echo "  · 坐标  : 原点=场地左下角，启动区1=(7.0, 3.0)，启动区2=(1.1, 1.0)"
echo "  · 交互  : RViz2 中点击【2D Goal Pose】指点导航"
echo "===================================================================="

ros2 launch "$ROOT_DIR/launch/gazebo_rc_obstacle.launch.py" "$@"

