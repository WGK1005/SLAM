#!/usr/bin/env bash
# ==============================================================================
#  SCU-TDT-V1 决策规划套件一键编译与测试脚本
#  支持：
#    bash tools/run_scu_tdt_v1.sh                     # 默认在实机实验室地图上规划并生成诊断大图
#    bash tools/run_scu_tdt_v1.sh --start 0.03 0.06 --goal -4.8 8.5
# ==============================================================================
set -e

ROOT_DIR="/home/wgk/decision_planning/SCU-TDT-V1"
cd "$ROOT_DIR"

if [ ! -f "$ROOT_DIR/maps/lab_map_1021.pgm" ]; then
    echo "[1/3] 正在从实机 3D 先验点云 (map_1021_1050.pcd) 生成 2D 导航栅格地图..."
    PYTHONNOUSERSITE=1 python3 "$ROOT_DIR/tools/pcd_to_nav_grid.py"
fi

if [ ! -x "$ROOT_DIR/build/test_lab_nav" ]; then
    echo "[2/3] 正在编译 SCU-TDT-V1 核心规划库与测试程序..."
    mkdir -p "$ROOT_DIR/build"
    g++ -std=c++20 -O3 -I"$ROOT_DIR/include" -I"$ROOT_DIR/src" -I/usr/include/eigen3 \
        "$ROOT_DIR/src/YAstar/yastar.cpp" \
        "$ROOT_DIR/src/YAstar/kinodynamicAstar.cpp" \
        "$ROOT_DIR/src/MinimumSnapOsqp/minimumSnap.cpp" \
        "$ROOT_DIR/src/MinimumSnapOsqp/sfcSquare.cpp" \
        "$ROOT_DIR/tools/test_lab_nav.cpp" \
        -o "$ROOT_DIR/build/test_lab_nav"
fi

echo "[3/3] 正在运行 SCU-TDT-V1 实机地图规划与轨迹优化..."
"$ROOT_DIR/build/test_lab_nav" "$@"

