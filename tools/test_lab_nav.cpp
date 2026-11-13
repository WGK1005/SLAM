#include "../include/scu_tdt_v1/scu_recovery_tracker.hpp"
#include <iostream>
#include <fstream>
#include <iomanip>
#include <cstdlib>
#include <sys/stat.h>

using namespace scu_tdt_v1;

int main(int argc, char** argv) {
    std::string pgm_path = "/home/wgk/decision_planning/SCU-TDT-V1/maps/lab_map_1021.pgm";
    // 默认起点：GlobalMatcher V2 实机重定位锁定的真实坐标 (0.031m, 0.063m)
    Eigen::Vector2f start_w(0.031f, 0.063f);
    // 默认终点：穿过实验室中央两排大型实验台岛台，到达北侧走廊 (-1.10m, 9.60m)
    Eigen::Vector2f goal_w(-1.10f, 9.60f);

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--start" && i + 2 < argc) {
            start_w.x() = std::stof(argv[++i]);
            start_w.y() = std::stof(argv[++i]);
        } else if (arg == "--goal" && i + 2 < argc) {
            goal_w.x() = std::stof(argv[++i]);
            goal_w.y() = std::stof(argv[++i]);
        } else if (arg == "--map" && i + 1 < argc) {
            pgm_path = argv[++i];
        }
    }

    HardwareConfig cfg;
    cfg.resolution = 0.05f;
    cfg.origin_x = -7.50f;
    cfg.origin_y = -2.50f;
    cfg.z_floor = -0.470f;
    cfg.robot_radius = 0.22f;
    cfg.max_speed = 1.20f;
    cfg.max_acc = 0.80f;
    cfg.max_corridor_range = 0.45f;

    ScuTdtPlannerEngine engine;
    if (!engine.LoadPGMMap(pgm_path, cfg)) {
        std::cerr << "❌ 无法加载实验室 2D 导航栅格图: " << pgm_path << std::endl;
        return 1;
    }

    std::cout << "====================================================================\n";
    std::cout << "  🚀 【SCU-TDT-V1】决策规划与轨迹优化套件 (适配 VLP-16 + HI12 实机地图)\n";
    std::cout << "  · 实机先验地图 : /home/wgk/maps/map_1021_1050.pcd -> 0.05m 导航栅格\n";
    std::cout << "  · 起点 (重定位): X = " << start_w.x() << " m, Y = " << start_w.y() << " m\n";
    std::cout << "  · 目标点 (Goal): X = " << goal_w.x() << " m, Y = " << goal_w.y() << " m\n";
    std::cout << "  · 底盘动力学   : v_max = " << cfg.max_speed << " m/s, a_max = " << cfg.max_acc << " m/s²\n";
    std::cout << "====================================================================\n";

    // 执行端到端规划 (YAstar 人工势场搜索 + DP 化简 + MinimumSnap + SFC 方形走廊优化)
    auto plan = engine.Plan(start_w, goal_w, Eigen::Vector2f(0.0f, 0.0f));
    if (!plan.success) {
        std::cerr << "❌ 路径规划失败！请检查起点或终点是否在地图外。\n";
        return 1;
    }

    // 测试 SCURM 智能自由空间脱困：假设小车在实验台死角 (-1.85m, 0.55m) 受阻
    Eigen::Vector2f stuck_pt(-1.85f, 0.55f);
    Eigen::Vector2f escaped_pt = engine.FindNearestSafePoint(stuck_pt, cfg.robot_radius + 0.12f);
    Eigen::Vector3f recovery_cmd = engine.ComputeSmartRecoveryTwist(stuck_pt, 2.42f);

    std::cout << "\n======================== SCU-TDT-V1 规划评估报表 ========================\n";
    std::cout << "  [1] 前端 YAstar 搜索耗时       : " << std::fixed << std::setprecision(3) << plan.astar_ms << " ms ("
              << plan.raw_astar_path_w.size() << " 个栅格节点)\n";
    std::cout << "  [2] DP + 视线碰撞化简耗时      : " << plan.simplify_ms << " ms ("
              << plan.simplified_waypoints_w.size() << " 个关键路标拐点)\n";
    std::cout << "  [3] 后端 MinimumSnap+SFC 耗时  : " << plan.min_snap_ms << " ms (迭代 "
              << plan.min_snap_iters << " 次, " << plan.corridors_w.size() << " 个方形安全走廊)\n";
    std::cout << "  ★  全流程端到端总耗时         : \033[1;32m" << plan.total_ms << " ms\033[0m (支持 >100Hz 实时重规划!)\n";
    std::cout << "  ·  优化后平滑轨迹总长         : " << plan.path_length_m << " m ("
              << plan.smooth_traj_w.size() << " 个 20Hz 采样控制点)\n";
    std::cout << "------------------------------------------------------------------------\n";
    std::cout << "  [SCURM 智能自由空间脱困自检]\n";
    std::cout << "    · 模拟受困贴墙点 : (" << stuck_pt.x() << " m, " << stuck_pt.y() << " m)\n";
    std::cout << "    · 自动搜索脱困点 : (" << escaped_pt.x() << " m, " << escaped_pt.y() << " m)\n";
    std::cout << "    · 脱困速度指令   : vx = " << recovery_cmd.x() << " m/s, wz = " << recovery_cmd.z() << " rad/s\n";
    std::cout << "========================================================================\n";

    // 导出数据供 Python 渲染高清可视化诊断大图
    mkdir("/tmp/scu_tdt_eval", 0777);

    {
        std::ofstream ofs("/tmp/scu_tdt_eval/astar_raw.txt");
        for (const auto& p : plan.raw_astar_path_w) ofs << p.x() << " " << p.y() << "\n";
    }
    {
        std::ofstream ofs("/tmp/scu_tdt_eval/waypoints.txt");
        for (const auto& p : plan.simplified_waypoints_w) ofs << p.x() << " " << p.y() << "\n";
    }
    {
        std::ofstream ofs("/tmp/scu_tdt_eval/smooth_traj.txt");
        for (size_t i = 0; i < plan.smooth_traj_w.size(); ++i) {
            float t = i * 0.05f;
            ofs << t << " "
                << plan.smooth_traj_w[i].x() << " " << plan.smooth_traj_w[i].y() << " "
                << plan.smooth_vel_w[i].x() << " " << plan.smooth_vel_w[i].y() << " "
                << plan.smooth_acc_w[i].x() << " " << plan.smooth_acc_w[i].y() << "\n";
        }
    }
    {
        std::ofstream ofs("/tmp/scu_tdt_eval/corridors.txt");
        for (const auto& c : plan.corridors_w) {
            ofs << c[0] << " " << c[1] << " " << c[2] << " " << c[3] << "\n";
        }
    }
    {
        // 导出人工势场代价图 (Costmap) 供可视化
        auto cost_mat = engine.GetAstar().getCostMap();
        std::ofstream ofs("/tmp/scu_tdt_eval/costmap.bin", std::ios::binary);
        int w = engine.Width(), h = engine.Height();
        ofs.write(reinterpret_cast<const char*>(&w), sizeof(int));
        ofs.write(reinterpret_cast<const char*>(&h), sizeof(int));
        ofs.write(reinterpret_cast<const char*>(cost_mat.data()), w * h * sizeof(float));
    }
    {
        std::ofstream ofs("/tmp/scu_tdt_eval/info.txt");
        ofs << "start_x=" << plan.safe_start_w.x() << "\n"
            << "start_y=" << plan.safe_start_w.y() << "\n"
            << "goal_x=" << plan.safe_goal_w.x() << "\n"
            << "goal_y=" << plan.safe_goal_w.y() << "\n"
            << "stuck_x=" << stuck_pt.x() << "\n"
            << "stuck_y=" << stuck_pt.y() << "\n"
            << "escape_x=" << escaped_pt.x() << "\n"
            << "escape_y=" << escaped_pt.y() << "\n"
            << "astar_ms=" << plan.astar_ms << "\n"
            << "simplify_ms=" << plan.simplify_ms << "\n"
            << "min_snap_ms=" << plan.min_snap_ms << "\n"
            << "total_ms=" << plan.total_ms << "\n"
            << "path_len=" << plan.path_length_m << "\n"
            << "corridor_cnt=" << plan.corridors_w.size() << "\n";
    }

    int ret = std::system("PYTHONNOUSERSITE=1 python3 /home/wgk/decision_planning/SCU-TDT-V1/tools/plot_nav_result.py");
    if (ret != 0) {
        std::cerr << "⚠️ 可视化绘图脚本执行返回非零状态: " << ret << std::endl;
    }
    return 0;
}

