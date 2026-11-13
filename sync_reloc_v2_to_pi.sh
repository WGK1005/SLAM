#!/usr/bin/env bash
# ==============================================================================
#  一键将 GlobalMatcher V2 (纯墙面正负距离场 + 多假设ICP + 自动出图) 同步并编译到香橙派
# ==============================================================================
set -e

export PI_HOST="${PI_HOST:-orangepi@10.207.70.100}"
SCP_TOOL="/home/wgk/scripts/pi_scp.py"
SSH_TOOL="/home/wgk/scripts/pi.py"

echo "=================================================================="
echo "  🚀 正在同步 GlobalMatcher V2 升级包至香橙派 ($PI_HOST)..."
echo "=================================================================="

echo "[1/4] 传输核心算法源码与绘图脚本..."
python3 "$SCP_TOOL" \
    /home/wgk/fast_lio_ws/src/open3d_loc/include/open3d_registration/global_matcher.h \
    "$PI_HOST:~/fast_lio_ws/src/open3d_loc/include/open3d_registration/global_matcher.h"

python3 "$SCP_TOOL" \
    /home/wgk/fast_lio_ws/src/open3d_loc/src/open3d_registration/global_matcher.cpp \
    "$PI_HOST:~/fast_lio_ws/src/open3d_loc/src/open3d_registration/global_matcher.cpp"

python3 "$SCP_TOOL" \
    /home/wgk/fast_lio_ws/src/open3d_loc/src/auto_global_reloc_node.cpp \
    "$PI_HOST:~/fast_lio_ws/src/open3d_loc/src/auto_global_reloc_node.cpp"

python3 "$SCP_TOOL" \
    /home/wgk/fast_lio_ws/src/open3d_loc/CMakeLists.txt \
    "$PI_HOST:~/fast_lio_ws/src/open3d_loc/CMakeLists.txt"

python3 "$SCP_TOOL" \
    /home/wgk/fast_lio_ws/tools/test_global_reloc.cpp \
    /home/wgk/fast_lio_ws/tools/plot_reloc_result.py \
    /home/wgk/fast_lio_ws/tools/run_global_reloc.sh \
    "$PI_HOST:~/fast_lio_ws/tools/"

echo "[2/4] 清理香橙派旧版错误地面缓存 (.df.bin) 并上传 V2 纯墙面距离场缓存 (.df_v2.bin)..."
python3 "$SSH_TOOL" "rm -f ~/maps/*.df.bin"
if [ -f "/home/wgk/maps/map_1021_1050.pcd.df_v2.bin" ]; then
    python3 "$SCP_TOOL" \
        /home/wgk/maps/map_1021_1050.pcd.df_v2.bin \
        "$PI_HOST:~/maps/map_1021_1050.pcd.df_v2.bin"
fi

echo "[3/4] 在香橙派上增量编译 open3d_loc..."
python3 "$SSH_TOOL" "source ~/fast_lio_ws/ros_env.sh && cd ~/fast_lio_ws && colcon build --packages-select open3d_loc && cp ~/fast_lio_ws/install/open3d_loc/lib/open3d_loc/test_global_reloc ~/fast_lio_ws/tools/test_global_reloc"

echo "[4/4] 在香橙派上运行一次离线自检..."
python3 "$SSH_TOOL" "bash ~/fast_lio_ws/tools/run_global_reloc.sh --offline --sim-yaw 137.5"

echo "=================================================================="
echo "  ✅ 香橙派 GlobalMatcher V2 同步与编译全部完成！"
echo "  现在可在香橙派直接运行: bash ~/fast_lio_ws/tools/run_global_reloc.sh"
echo "=================================================================="

