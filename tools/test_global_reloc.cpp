#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <open3d/Open3D.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "open3d_registration/global_matcher.h"

int main(int argc, char** argv) {
    const char* home_env = std::getenv("HOME");
    std::string home_str = home_env ? std::string(home_env) : "";
    std::string map_path = home_str + "/maps/map_1021_1050.pcd";
    std::string scan_path = home_str + "/maps/scan_1021_1050.pcd";

    // 模拟开机未知位姿 (Ground Truth)
    double sim_x = 0.0;
    double sim_y = 0.0;
    double sim_z = 0.0; // 可选高度偏移模拟 (如放在桌子上 vs 地板上)
    double sim_yaw = 0.0;
    bool has_sim_xy = false;
    bool has_sim_yaw = false;

    // 粗初值先验范围 (< 0 表示 100% 全图盲开搜索)
    double prior_x = 0.0;
    double prior_y = 0.0;
    double prior_radius = -1.0;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--map" && i + 1 < argc) map_path = argv[++i];
        else if (arg == "--scan" && i + 1 < argc) scan_path = argv[++i];
        else if ((arg == "--sim-x" || arg == "-x") && i + 1 < argc) {
            sim_x = std::stod(argv[++i]);
            has_sim_xy = true;
        }
        else if ((arg == "--sim-y" || arg == "-y") && i + 1 < argc) {
            sim_y = std::stod(argv[++i]);
            has_sim_xy = true;
        }
        else if ((arg == "--sim-z" || arg == "-z") && i + 1 < argc) {
            sim_z = std::stod(argv[++i]);
        }
        else if ((arg == "--sim-yaw" || arg == "--yaw") && i + 1 < argc) {
            sim_yaw = std::stod(argv[++i]);
            has_sim_yaw = true;
        }
        else if (arg == "--prior-x" && i + 1 < argc) prior_x = std::stod(argv[++i]);
        else if (arg == "--prior-y" && i + 1 < argc) prior_y = std::stod(argv[++i]);
        else if (arg == "--prior-r" && i + 1 < argc) prior_radius = std::stod(argv[++i]);
    }

    std::cout << "==============================================================\n";
    std::cout << "  开机无先验全局重定位测试 V2 (地面解耦 + 纯墙面正负距离场 + 多假设ICP)\n";
    std::cout << "  先验地图: " << map_path << "\n";
    if (has_sim_xy) {
        std::cout << "  模拟开机未知坐标 (GT): X = " << sim_x << " m, Y = " << sim_y
                  << " m, dZ = " << sim_z << " m, Yaw = " << sim_yaw << "°\n";
    } else if (has_sim_yaw) {
        std::cout << "  模拟开机未知朝向 (GT): Yaw = " << sim_yaw << "°\n";
        std::cout << "  实测雷达扫描文件: " << scan_path << "\n";
    } else {
        std::cout << "  实测雷达扫描文件: " << scan_path << "\n";
    }

    if (prior_radius > 0) {
        std::cout << "  先验搜索中心: (" << prior_x << ", " << prior_y
                  << "), 限制半径: " << prior_radius << " m\n";
    } else {
        std::cout << "  ★ 算法先验: 完全为 0 (完全盲开，全图 360° 无先验全局搜索！)\n";
    }
    std::cout << "==============================================================\n";

    // 1. 初始化 V2 全局匹配器并加载地图
    open3d_loc::GlobalMatcher matcher;
    auto t_load_s = std::chrono::high_resolution_clock::now();
    if (!matcher.LoadMap(map_path, 0.10)) {
        return 1;
    }
    auto t_load_e = std::chrono::high_resolution_clock::now();
    double load_ms = std::chrono::duration<double, std::milli>(t_load_e - t_load_s).count();
    std::cout << "[1/3] 地图与 V2 纯墙面特征加载完成 (耗时: " << load_ms << " ms)\n";

    // 2. 准备待测雷达扫描点云
    auto scan_cloud = std::make_shared<open3d::geometry::PointCloud>();
    bool has_gt = false;
    double gt_x = 0.0, gt_y = 0.0, gt_yaw = 0.0;

    if (has_sim_xy) {
        has_gt = true;
        gt_x = sim_x;
        gt_y = sim_y;
        gt_yaw = sim_yaw;

        auto map_cloud = std::make_shared<open3d::geometry::PointCloud>();
        open3d::io::ReadPointCloud(map_path, *map_cloud);

        double rad_sim = sim_yaw * M_PI / 180.0;
        double c_sim = std::cos(rad_sim);
        double s_sim = std::sin(rad_sim);

        for (const auto& p : map_cloud->points_) {
            double dx = p.x() - sim_x;
            double dy = p.y() - sim_y;
            double dz = p.z() - sim_z;
            if (std::hypot(dx, dy) < 9.5 && p.z() < 1.5) {
                double lx = c_sim * dx + s_sim * dy;
                double ly = -s_sim * dx + c_sim * dy;
                double lz = dz;
                scan_cloud->points_.push_back(Eigen::Vector3d(lx, ly, lz));
            }
        }

        if (scan_cloud->points_.size() < 3000) {
            std::cerr << "\n❌ 【坐标超界警告】坐标 X=" << sim_x << "m, Y=" << sim_y
                      << "m 位于地图有效建筑外部！\n";
            return 1;
        }
        std::cout << "[2/3] 成功在位置 (" << sim_x << ", " << sim_y << ", " << sim_yaw
                  << "°) 提取模拟雷达帧 (" << scan_cloud->points_.size() << " 点)\n";
    } else {
        if (!open3d::io::ReadPointCloud(scan_path, *scan_cloud) || scan_cloud->IsEmpty()) {
            std::cerr << "❌ 无法读取雷达扫描: " << scan_path << std::endl;
            return 1;
        }
        if (has_sim_yaw) {
            has_gt = true;
            gt_x = 0.0;
            gt_y = 0.0;
            gt_yaw = sim_yaw;
            double rad_rot = sim_yaw * M_PI / 180.0;
            Eigen::Matrix4d T_rot = Eigen::Matrix4d::Identity();
            T_rot(0, 0) = std::cos(rad_rot);
            T_rot(0, 1) = -std::sin(rad_rot);
            T_rot(1, 0) = std::sin(rad_rot);
            T_rot(1, 1) = std::cos(rad_rot);
            T_rot(2, 3) = sim_z;
            scan_cloud->Transform(T_rot.inverse());
        } else if (scan_path.find("scan_1021_1050.pcd") != std::string::npos) {
            has_gt = true;
            gt_x = 0.0;
            gt_y = 0.0;
            gt_yaw = 0.0;
        }
        std::cout << "[2/3] 成功加载实测雷达扫描 (" << scan_cloud->points_.size() << " 点)\n";
    }

    // 3. 执行 V2 全局重定位
    std::cout << "[3/3] 正在执行地面解耦 + 360° 纯墙面正负距离场搜索 + Top-10 多假设 ICP..." << std::endl;
    auto res = matcher.Relocalize(scan_cloud, prior_x, prior_y, prior_radius);

    std::cout << "\n======================== 全局自定位 V2 评估报表 ========================\n";
    if (has_gt) {
        std::cout << "  开机真值坐标 (Ground Truth) : X = " << gt_x << " m, Y = " << gt_y
                  << " m, Yaw = " << gt_yaw << "°\n";
    }
    std::cout << "  粗匹配最佳候选 : X = " << res.coarse_x << " m, Y = " << res.coarse_y
              << " m, Yaw = " << res.coarse_yaw << "°\n";
    std::cout << "  ★ 最终精准位姿 : X = " << res.x << " m, Y = " << res.y
              << " m, Z = " << res.z << " m, Yaw = " << res.yaw << "°\n";
    std::cout << "------------------------------------------------------------------------\n";
    if (has_gt) {
        double err_x = std::abs(res.x - gt_x);
        double err_y = std::abs(res.y - gt_y);
        double err_yaw = std::abs(res.yaw - gt_yaw);
        while (err_yaw > 180.0) err_yaw = std::abs(err_yaw - 360.0);
        std::cout << "  绝对平移定位误差 : dX = " << err_x * 100.0 << " cm, dY = " << err_y * 100.0
                  << " cm (平面总误差: " << std::hypot(err_x, err_y) * 100.0 << " cm)\n";
        std::cout << "  绝对朝向航向误差 : dYaw = " << err_yaw << "°\n";
    }
    std::cout << "  纯墙面真实咬合度 : " << res.wall_fitness * 100.0 << " % (Wall Fitness, 核心防伪指标)\n";
    std::cout << "  纯墙面残差均方根 : " << res.wall_rmse * 100.0 << " cm (Wall RMSE)\n";
    std::cout << "  全点云综合咬合度 : " << res.fitness * 100.0 << " % (Overall Fitness)\n";
    std::cout << "------------------------------------------------------------------------\n";
    std::cout << "  性能耗时明细:\n";
    std::cout << "    · 360°粗匹配耗时   : " << res.time_coarse_ms << " ms\n";
    std::cout << "    · 多假设ICP精修耗时: " << res.time_fine_ms << " ms\n";
    std::cout << "    ★ 在线自定位总耗时 : " << res.time_total_ms << " ms\n";
    std::cout << "========================================================================\n";

    std::string reloc_png = home_str + "/maps/reloc_process_result.png";
    matcher.SaveEvalAndPlot(scan_cloud, res, reloc_png, has_gt, gt_x, gt_y, gt_yaw);
    std::cout << "🖼️  最新匹配过程图像已保存至: " << reloc_png << "\n";

    if (res.success) {
        std::cout << "✅ 【重定位成功】纯墙面几何验证通过，成功锁定真实全局坐标！\n";
        return 0;
    } else {
        std::cout << "⚠️ 定位未通过纯墙面高置信度检验 (Wall Fitness=" << res.wall_fitness * 100.0 << "%)\n";
        return 1;
    }
}
