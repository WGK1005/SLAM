#include <iostream>
#include <fstream>
#include <chrono>
#include <cmath>
#include <open3d/Open3D.h>
#include <Eigen/Core>
#include <Eigen/Dense>

int main(int argc, char** argv) {
    std::string map_path = "/home/wgk/maps/pc_base2.pcd";
    std::string scan_path = "/home/wgk/maps/lab_scan_sample.pcd";
    double offset_x = 1.0;   // meters error
    double offset_y = -1.1;  // meters error
    double offset_yaw = 20.0; // degrees error
    std::string out_dir = "/tmp/o3d_eval";

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--map" && i + 1 < argc) map_path = argv[++i];
        else if (arg == "--scan" && i + 1 < argc) scan_path = argv[++i];
        else if (arg == "--x" && i + 1 < argc) offset_x = std::stod(argv[++i]);
        else if (arg == "--y" && i + 1 < argc) offset_y = std::stod(argv[++i]);
        else if (arg == "--yaw" && i + 1 < argc) offset_yaw = std::stod(argv[++i]);
        else if (arg == "--out-dir" && i + 1 < argc) out_dir = argv[++i];
    }

    std::cout << "====================================================\n";
    std::cout << "  Open3D 离线单帧配准测试 (实验室场景)\n";
    std::cout << "  先验地图: " << map_path << "\n";
    std::cout << "  雷达点云: " << scan_path << "\n";
    std::cout << "  人为注入初始粗位姿偏差: dx=" << offset_x << " m, dy=" << offset_y 
              << " m, dyaw=" << offset_yaw << " deg\n";
    std::cout << "====================================================\n";

    // 1. 读取点云
    auto map_cloud = std::make_shared<open3d::geometry::PointCloud>();
    auto scan_cloud = std::make_shared<open3d::geometry::PointCloud>();

    if (!open3d::io::ReadPointCloud(map_path, *map_cloud) || map_cloud->IsEmpty()) {
        std::cerr << "❌ 无法读取先验地图: " << map_path << std::endl;
        return 1;
    }
    if (!open3d::io::ReadPointCloud(scan_path, *scan_cloud) || scan_cloud->IsEmpty()) {
        std::cerr << "❌ 无法读取雷达扫描: " << scan_path << std::endl;
        return 1;
    }

    std::cout << "  地图点数: " << map_cloud->points_.size() 
              << " | 扫描点数: " << scan_cloud->points_.size() << std::endl;

    // 2. 预处理与法向量估计
    double voxel_size = 0.1;
    auto map_down = map_cloud->VoxelDownSample(voxel_size);
    map_down->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(voxel_size * 2.0, 30));

    auto scan_down = scan_cloud->VoxelDownSample(voxel_size);

    // 3. 构建初始粗坐标矩阵 T_init (注入偏差)
    double rad = offset_yaw * M_PI / 180.0;
    Eigen::Matrix4d T_init = Eigen::Matrix4d::Identity();
    T_init(0, 0) = std::cos(rad);
    T_init(0, 1) = -std::sin(rad);
    T_init(1, 0) = std::sin(rad);
    T_init(1, 1) = std::cos(rad);
    T_init(0, 3) = offset_x;
    T_init(1, 3) = offset_y;

    // 评估配准前的重合度
    auto init_eval = open3d::pipelines::registration::EvaluateRegistration(
        *scan_down, *map_down, voxel_size * 3.0, T_init);

    // 保存初始偏差位置的点云
    auto scan_initial = std::make_shared<open3d::geometry::PointCloud>(*scan_down);
    scan_initial->Transform(T_init);

    // 4. 执行 Open3D 点到平面 ICP 配准 (两级粗精收敛，保证能拉回大偏差)
    auto t_start = std::chrono::high_resolution_clock::now();

    // 粗配准：阈值 1.0m，拉回大偏差
    auto icp_coarse = open3d::pipelines::registration::RegistrationICP(
        *scan_down, *map_down, 1.0, T_init,
        open3d::pipelines::registration::TransformationEstimationPointToPlane(),
        open3d::pipelines::registration::ICPConvergenceCriteria(1e-6, 1e-6, 30));

    // 精配准：阈值 0.25m，严密贴合墙体结构
    auto icp_result = open3d::pipelines::registration::RegistrationICP(
        *scan_down, *map_down, 0.25, icp_coarse.transformation_,
        open3d::pipelines::registration::TransformationEstimationPointToPlane(),
        open3d::pipelines::registration::ICPConvergenceCriteria(1e-6, 1e-6, 30));

    auto t_end = std::chrono::high_resolution_clock::now();
    double cost_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();

    // 评估配准后的重合度
    auto final_eval = open3d::pipelines::registration::EvaluateRegistration(
        *scan_down, *map_down, voxel_size * 3.0, icp_result.transformation_);

    // 保存配准对齐后的点云
    auto scan_aligned = std::make_shared<open3d::geometry::PointCloud>(*scan_down);
    scan_aligned->Transform(icp_result.transformation_);

    // 计算校正后的位姿 (真值附近应接近单位阵 [0, 0, 0])
    double est_x = icp_result.transformation_(0, 3);
    double est_y = icp_result.transformation_(1, 3);
    double est_yaw = std::atan2(icp_result.transformation_(1, 0), icp_result.transformation_(0, 0)) * 180.0 / M_PI;

    std::cout << "\n---------------- 配准结果 ----------------\n";
    std::cout << "  配准耗时       : " << cost_ms << " ms\n";
    std::cout << "  配准前重合度   : " << init_eval.fitness_ * 100.0 << " %\n";
    std::cout << "  配准后重合度   : " << final_eval.fitness_ * 100.0 << " %\n";
    std::cout << "  配准后点均误差 : " << final_eval.inlier_rmse_ * 100.0 << " cm\n";
    std::cout << "  收敛位姿校正量 : x=" << est_x << " m, y=" << est_y << " m, yaw=" << est_yaw << " deg\n";
    std::cout << "------------------------------------------\n";

    // 5. 保存结果供可视化渲染
    system(("mkdir -p " + out_dir).c_str());
    open3d::io::WritePointCloud(out_dir + "/scan_initial.pcd", *scan_initial);
    open3d::io::WritePointCloud(out_dir + "/scan_aligned.pcd", *scan_aligned);

    std::ofstream ofs(out_dir + "/result.txt");
    ofs << cost_ms << "\n"
        << init_eval.fitness_ << "\n"
        << final_eval.fitness_ << "\n"
        << final_eval.inlier_rmse_ << "\n"
        << offset_x << "\n" << offset_y << "\n" << offset_yaw << "\n"
        << est_x << "\n" << est_y << "\n" << est_yaw << "\n";
    ofs.close();

    std::cout << "✅ 结果已保存至 " << out_dir << "，准备生成灰度图可视化效果！\n";
    return 0;
}

