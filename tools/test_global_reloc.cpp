#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <open3d/Open3D.h>
#include <Eigen/Core>
#include <Eigen/Dense>

// 2D 距离场似然度特征图结构体 (支持二进制秒级持久化缓存)
struct MapDistanceField2D {
    float min_x = 0, max_x = 0, min_y = 0, max_y = 0;
    float resolution = 0.15f;
    int nx = 0, ny = 0;
    std::vector<float> lut; // 预计算的高斯似然度表: exp(-d^2 / (2 * sigma^2))

    bool Save(const std::string& filepath) const {
        std::ofstream ofs(filepath, std::ios::binary);
        if (!ofs.is_open()) return false;
        ofs.write(reinterpret_cast<const char*>(&min_x), sizeof(float));
        ofs.write(reinterpret_cast<const char*>(&max_x), sizeof(float));
        ofs.write(reinterpret_cast<const char*>(&min_y), sizeof(float));
        ofs.write(reinterpret_cast<const char*>(&max_y), sizeof(float));
        ofs.write(reinterpret_cast<const char*>(&resolution), sizeof(float));
        ofs.write(reinterpret_cast<const char*>(&nx), sizeof(int));
        ofs.write(reinterpret_cast<const char*>(&ny), sizeof(int));
        size_t sz = lut.size();
        ofs.write(reinterpret_cast<const char*>(&sz), sizeof(size_t));
        ofs.write(reinterpret_cast<const char*>(lut.data()), sz * sizeof(float));
        return true;
    }

    bool Load(const std::string& filepath) {
        std::ifstream ifs(filepath, std::ios::binary);
        if (!ifs.is_open()) return false;
        ifs.read(reinterpret_cast<char*>(&min_x), sizeof(float));
        ifs.read(reinterpret_cast<char*>(&max_x), sizeof(float));
        ifs.read(reinterpret_cast<char*>(&min_y), sizeof(float));
        ifs.read(reinterpret_cast<char*>(&max_y), sizeof(float));
        ifs.read(reinterpret_cast<char*>(&resolution), sizeof(float));
        ifs.read(reinterpret_cast<char*>(&nx), sizeof(int));
        ifs.read(reinterpret_cast<char*>(&ny), sizeof(int));
        size_t sz = 0;
        ifs.read(reinterpret_cast<char*>(&sz), sizeof(size_t));
        lut.resize(sz);
        ifs.read(reinterpret_cast<char*>(lut.data()), sz * sizeof(float));
        return true;
    }

    void BuildFromPointCloud(const open3d::geometry::PointCloud& cloud, float res, float sigma, float max_dist) {
        resolution = res;
        min_x = 1e9; max_x = -1e9;
        min_y = 1e9; max_y = -1e9;
        std::vector<Eigen::Vector2d> obstacle_pts;
        for (const auto& p : cloud.points_) {
            if (p.z() > -0.8 && p.z() < 0.8) {
                if (p.x() < min_x) min_x = p.x();
                if (p.x() > max_x) max_x = p.x();
                if (p.y() < min_y) min_y = p.y();
                if (p.y() > max_y) max_y = p.y();
                obstacle_pts.push_back(Eigen::Vector2d(p.x(), p.y()));
            }
        }
        min_x -= 1.0; max_x += 1.0;
        min_y -= 1.0; max_y += 1.0;
        nx = std::ceil((max_x - min_x) / res);
        ny = std::ceil((max_y - min_y) / res);
        lut.assign(nx * ny, 0.0f);

        auto obs_cloud = std::make_shared<open3d::geometry::PointCloud>();
        for (const auto& pt : obstacle_pts) {
            obs_cloud->points_.push_back(Eigen::Vector3d(pt.x(), pt.y(), 0.0));
        }
        open3d::geometry::KDTreeFlann kdtree(*obs_cloud);

        #pragma omp parallel for
        for (int iy = 0; iy < ny; ++iy) {
            float y = min_y + iy * res;
            std::vector<int> idx(1);
            std::vector<double> d(1);
            for (int ix = 0; ix < nx; ++ix) {
                float x = min_x + ix * res;
                kdtree.SearchKNN(Eigen::Vector3d(x, y, 0.0), 1, idx, d);
                float dist = std::sqrt(d[0]);
                if (dist < max_dist) {
                    lut[iy * nx + ix] = std::exp(- (dist * dist) / (2.0f * sigma * sigma));
                } else {
                    lut[iy * nx + ix] = 0.0f;
                }
            }
        }
    }
};

int main(int argc, char** argv) {
    std::string map_path = "/home/wgk/maps/map_1021_1050.pcd";
    std::string scan_path = "/home/wgk/maps/scan_1021_1050.pcd";
    
    // 模拟开机未知位姿 (Ground Truth)
    double sim_x = 0.0;
    double sim_y = 0.0;
    double sim_yaw = 0.0;
    bool has_sim_xy = false;
    bool has_sim_yaw = false;

    // 粗初值先验范围 (若指定，则只在先验附近搜索；不指定则 100% 全图盲开搜索)
    double prior_x = 0.0;
    double prior_y = 0.0;
    double prior_radius = -1.0; // < 0 表示全图盲搜

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
        else if ((arg == "--sim-yaw" || arg == "--yaw") && i + 1 < argc) {
            sim_yaw = std::stod(argv[++i]);
            has_sim_yaw = true;
        }
        else if (arg == "--prior-x" && i + 1 < argc) prior_x = std::stod(argv[++i]);
        else if (arg == "--prior-y" && i + 1 < argc) prior_y = std::stod(argv[++i]);
        else if (arg == "--prior-r" && i + 1 < argc) prior_radius = std::stod(argv[++i]);
    }

    std::cout << "==============================================================\n";
    std::cout << "  开机无先验初值全局重定位测试 (距离场似然度 + 点到面 ICP)\n";
    std::cout << "  先验地图: " << map_path << "\n";
    std::cout << "  地图建筑有效范围: X ∈ [-6.0, +2.2] m, Y ∈ [-1.5, +10.5] m\n";
    if (has_sim_xy) {
        std::cout << "  模拟开机未知坐标 (GT): X = " << sim_x << " m, Y = " << sim_y 
                  << " m, Yaw = " << sim_yaw << "°\n";
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

    // 1. 读取先验地图
    auto map_cloud = std::make_shared<open3d::geometry::PointCloud>();
    if (!open3d::io::ReadPointCloud(map_path, *map_cloud) || map_cloud->IsEmpty()) {
        std::cerr << "❌ 无法读取先验地图: " << map_path << std::endl;
        return 1;
    }

    // 2. 准备点云数据 (支持实测 PCD 或任意位置地图采样合成)
    auto scan_cloud = std::make_shared<open3d::geometry::PointCloud>();

    if (has_sim_xy) {
        // 用户指定了在地图中的任意未知位置 (sim_x, sim_y, sim_yaw)
        // 从地图中提取该点周围雷达视场范围内的点云并转换到机器人本体局部系
        double rad_sim = sim_yaw * M_PI / 180.0;
        double c_sim = std::cos(rad_sim);
        double s_sim = std::sin(rad_sim);

        for (const auto& p : map_cloud->points_) {
            double dx = p.x() - sim_x;
            double dy = p.y() - sim_y;
            double dz = p.z();
            // 模拟 16 线激光雷达视距 (~10米) 与高度截断
            if (std::hypot(dx, dy) < 10.0 && std::abs(dz) < 1.3) {
                double lx = c_sim * dx + s_sim * dy;
                double ly = -s_sim * dx + c_sim * dy;
                double lz = dz;
                scan_cloud->points_.push_back(Eigen::Vector3d(lx, ly, lz));
            }
        }

        if (scan_cloud->points_.size() < 10000) {
            std::cerr << "\n\033[1;31m❌ 【坐标超界警告】您输入的坐标 X=" << sim_x 
                      << "m, Y=" << sim_y << "m 位于建筑外部的空旷真空区域！\033[0m\n";
            std::cerr << "\033[1;33m   当前地图真实建筑/房间走廊范围为: X ∈ [-6.0, +2.2] m, Y ∈ [-1.5, +10.5] m\033[0m\n";
            std::cerr << "\033[1;33m   在此范围外雷达打不到任何墙面或地面 (周围无有效激光点)，机器狗无法在此开机！\033[0m\n";
            std::cerr << "\033[1;32m   👉 推荐测试有效走廊/房间坐标，例如: -x -2.0 -y 6.0 或 -x 0.0 -y 4.0\033[0m\n\n";
            return 1;
        }

        std::cout << "[1/4] 成功在模拟位置 (" << sim_x << ", " << sim_y 
                  << ", " << sim_yaw << "°) 截取雷达视场点云 (" << scan_cloud->points_.size() << " 点)" << std::endl;
    } else {
        if (!open3d::io::ReadPointCloud(scan_path, *scan_cloud) || scan_cloud->IsEmpty()) {
            std::cerr << "❌ 无法读取雷达扫描: " << scan_path << std::endl;
            return 1;
        }
        // 若只有偏航旋转注入
        if (has_sim_yaw) {
            double rad_rot = sim_yaw * M_PI / 180.0;
            Eigen::Matrix4d T_rot = Eigen::Matrix4d::Identity();
            T_rot(0, 0) = std::cos(rad_rot);
            T_rot(0, 1) = -std::sin(rad_rot);
            T_rot(1, 0) = std::sin(rad_rot);
            T_rot(1, 1) = std::cos(rad_rot);
            scan_cloud->Transform(T_rot.inverse());
        }
        std::cout << "[1/4] 成功加载雷达扫描 (" << scan_cloud->points_.size() << " 点)" << std::endl;
    }

    // 3. 加载或构建地图特征距离场缓存 (.df.bin)
    std::string cache_path = map_path + ".df.bin";
    MapDistanceField2D df;
    auto t_cache_s = std::chrono::high_resolution_clock::now();
    if (df.Load(cache_path)) {
        std::cout << "[2/4] 从二进制缓存快速加载地图特征索引: " << cache_path << std::endl;
    } else {
        std::cout << "[2/4] 首次运行：正在生成地图特征距离场索引 (自动保存为缓存)..." << std::endl;
        df.BuildFromPointCloud(*map_cloud, 0.15f, 0.20f, 0.80f);
        df.Save(cache_path);
        std::cout << "      特征索引已缓存至: " << cache_path << std::endl;
    }
    auto t_cache_e = std::chrono::high_resolution_clock::now();
    double cache_cost_ms = std::chrono::duration<double, std::milli>(t_cache_e - t_cache_s).count();
    std::cout << "      地图特征加载耗时: " << cache_cost_ms << " ms" << std::endl;

    // 4. 准备高精点到面 ICP 地图 (清理异常法向量并下采样)
    double voxel_size = 0.10;
    auto map_down = map_cloud->VoxelDownSample(voxel_size);
    map_down->normals_.clear();
    map_down->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(voxel_size * 2.0, 30));

    // ======================== 在线自定位核心流程 ========================
    auto t_online_start = std::chrono::high_resolution_clock::now();

    // 提取当前单帧雷达水平切片特征点
    std::vector<Eigen::Vector2d> scan_sub;
    int step = std::max(1, (int)scan_cloud->points_.size() / 150);
    for (size_t i = 0; i < scan_cloud->points_.size(); i += step) {
        if (scan_cloud->points_[i].z() > -0.8 && scan_cloud->points_[i].z() < 0.8) {
            scan_sub.push_back(Eigen::Vector2d(scan_cloud->points_[i].x(), scan_cloud->points_[i].y()));
        }
    }

    // 5. 并行 360° 全图似然度几何匹配 (OpenMP 并行搜索 72 个偏航角与全图网格)
    std::cout << "[3/4] 正在执行无初值全局特征似然度几何匹配 (360°全向搜索)..." << std::endl;
    int n_yaw = 72; // 5° 航向角高角分辨率
    struct Candidate {
        float score;
        float x, y, yaw;
    };
    std::vector<Candidate> best_per_yaw(n_yaw, {-1.0f, 0, 0, 0});

    int stride = 2; // 30cm 粗搜索步长
    const float inv_res = 1.0f / df.resolution;
    const float* lut_ptr = df.lut.data();
    const int nx = df.nx;
    const int ny = df.ny;
    const float min_x = df.min_x;
    const float min_y = df.min_y;

    #pragma omp parallel for
    for (int a = 0; a < n_yaw; ++a) {
        double yaw = a * (2.0 * M_PI / n_yaw);
        double c = std::cos(yaw);
        double s = std::sin(yaw);

        std::vector<Eigen::Vector2f> rot_pts(scan_sub.size());
        for (size_t i = 0; i < scan_sub.size(); ++i) {
            rot_pts[i] = Eigen::Vector2f(c * scan_sub[i].x() - s * scan_sub[i].y(),
                                         s * scan_sub[i].x() + c * scan_sub[i].y());
        }

        for (int iy = 0; iy < ny; iy += stride) {
            float y_world = min_y + iy * df.resolution;
            if (prior_radius > 0 && std::abs(y_world - prior_y) > prior_radius) continue;

            for (int ix = 0; ix < nx; ix += stride) {
                float x_world = min_x + ix * df.resolution;
                if (prior_radius > 0 && std::abs(x_world - prior_x) > prior_radius) continue;

                float score = 0;
                for (const auto& pt : rot_pts) {
                    int px = static_cast<int>((pt.x() + x_world - min_x) * inv_res);
                    int py = static_cast<int>((pt.y() + y_world - min_y) * inv_res);
                    if (px >= 0 && px < nx && py >= 0 && py < ny) {
                        score += lut_ptr[py * nx + px];
                    }
                }

                if (score > best_per_yaw[a].score) {
                    best_per_yaw[a] = {score, x_world, y_world, (float)(yaw * 180.0 / M_PI)};
                }
            }
        }
    }

    Candidate best_global = {-1.0f, 0, 0, 0};
    for (const auto& c : best_per_yaw) {
        if (c.score > best_global.score) {
            best_global = c;
        }
    }

    auto t_coarse_end = std::chrono::high_resolution_clock::now();
    double coarse_cost_ms = std::chrono::duration<double, std::milli>(t_coarse_end - t_online_start).count();
    std::cout << "      粗匹配锁定最佳候选: X=" << best_global.x << "m, Y=" << best_global.y 
              << "m, Yaw=" << best_global.yaw << "° (得分: " << best_global.score 
              << " / " << scan_sub.size() << ", 耗时: " << coarse_cost_ms << " ms)" << std::endl;

    // 6. 两级点到面 ICP 精修收敛至亚厘米级
    std::cout << "[4/4] 正在执行两级点到面 ICP 亚厘米级高精微调..." << std::endl;
    double rad_coarse = best_global.yaw * M_PI / 180.0;
    Eigen::Matrix4d T_coarse = Eigen::Matrix4d::Identity();
    T_coarse(0, 0) = std::cos(rad_coarse);
    T_coarse(0, 1) = -std::sin(rad_coarse);
    T_coarse(1, 0) = std::sin(rad_coarse);
    T_coarse(1, 1) = std::cos(rad_coarse);
    T_coarse(0, 3) = best_global.x;
    T_coarse(1, 3) = best_global.y;

    auto scan_down = scan_cloud->VoxelDownSample(voxel_size);

    // 第一级粗 ICP：阈值 1.5m，消除网格离散化偏差
    auto icp_coarse = open3d::pipelines::registration::RegistrationICP(
        *scan_down, *map_down, 1.5, T_coarse,
        open3d::pipelines::registration::TransformationEstimationPointToPlane(),
        open3d::pipelines::registration::ICPConvergenceCriteria(1e-6, 1e-6, 30));

    // 第二级精 ICP：阈值 0.25m，严格咬合墙面立柱结构
    auto icp_fine = open3d::pipelines::registration::RegistrationICP(
        *scan_down, *map_down, 0.25, icp_coarse.transformation_,
        open3d::pipelines::registration::TransformationEstimationPointToPlane(),
        open3d::pipelines::registration::ICPConvergenceCriteria(1e-6, 1e-6, 30));

    auto t_online_end = std::chrono::high_resolution_clock::now();
    double icp_cost_ms = std::chrono::duration<double, std::milli>(t_online_end - t_coarse_end).count();
    double total_online_ms = std::chrono::duration<double, std::milli>(t_online_end - t_online_start).count();

    // 7. 结果解析与量化报表
    Eigen::Matrix4d T_est = icp_fine.transformation_;
    double est_x = T_est(0, 3);
    double est_y = T_est(1, 3);
    double est_z = T_est(2, 3);
    double est_yaw = std::atan2(T_est(1, 0), T_est(0, 0)) * 180.0 / M_PI;

    std::cout << "\n======================== 全局自定位评估报表 ========================\n";
    if (has_sim_xy) {
        std::cout << "  开机真值坐标 (Ground Truth) : X = " << sim_x << " m, Y = " << sim_y 
                  << " m, Yaw = " << sim_yaw << "°\n";
    }
    std::cout << "  粗匹配候选位置 : X = " << best_global.x << " m, Y = " << best_global.y 
              << " m, Yaw = " << best_global.yaw << "°\n";
    std::cout << "  ★ 最终精准位姿 : X = " << est_x << " m, Y = " << est_y 
              << " m, Z = " << est_z << " m, Yaw = " << est_yaw << "°\n";
    std::cout << "--------------------------------------------------------------------\n";
    if (has_sim_xy) {
        double err_x = std::abs(est_x - sim_x);
        double err_y = std::abs(est_y - sim_y);
        double err_yaw = std::abs(est_yaw - sim_yaw);
        if (err_yaw > 180.0) err_yaw = 360.0 - err_yaw;
        std::cout << "  绝对平移定位误差 : dX = " << err_x * 100.0 << " cm, dY = " << err_y * 100.0 
                  << " cm (平面总误差: " << std::hypot(err_x, err_y) * 100.0 << " cm)\n";
        std::cout << "  绝对朝向航向误差 : dYaw = " << err_yaw << "°\n";
    }
    std::cout << "  点云咬合重合度 : " << icp_fine.fitness_ * 100.0 << " % (Fitness)\n";
    std::cout << "  点到面残差均方 : " << icp_fine.inlier_rmse_ * 100.0 << " cm (RMSE)\n";
    std::cout << "--------------------------------------------------------------------\n";
    std::cout << "  性能耗时明细:\n";
    std::cout << "    · 特征缓存加载耗时 : " << cache_cost_ms << " ms\n";
    std::cout << "    · 360°粗匹配耗时   : " << coarse_cost_ms << " ms\n";
    std::cout << "    · 两级点到面ICP耗时 : " << icp_cost_ms << " ms\n";
    std::cout << "    ★ 在线自定位总耗时 : " << total_online_ms << " ms (约 " 
              << total_online_ms / 1000.0 << " 秒)\n";
    std::cout << "====================================================================\n";

    // 8. 自动生成匹配全流程图像 (粗匹配候选扫描 + ICP精微调咬合)
    std::string eval_dir = "/tmp/reloc_eval";
    system(("mkdir -p " + eval_dir).c_str());

    auto scan_coarse_vis = std::make_shared<open3d::geometry::PointCloud>(*scan_down);
    scan_coarse_vis->Transform(T_coarse);
    open3d::io::WritePointCloud(eval_dir + "/scan_coarse.pcd", *scan_coarse_vis);

    auto scan_fine_vis = std::make_shared<open3d::geometry::PointCloud>(*scan_down);
    scan_fine_vis->Transform(T_est);
    open3d::io::WritePointCloud(eval_dir + "/scan_fine.pcd", *scan_fine_vis);

    std::ofstream ofs(eval_dir + "/info.txt");
    ofs << "gt_x=" << sim_x << "\n"
        << "gt_y=" << sim_y << "\n"
        << "gt_yaw=" << sim_yaw << "\n"
        << "coarse_x=" << best_global.x << "\n"
        << "coarse_y=" << best_global.y << "\n"
        << "coarse_yaw=" << best_global.yaw << "\n"
        << "fine_x=" << est_x << "\n"
        << "fine_y=" << est_y << "\n"
        << "fine_yaw=" << est_yaw << "\n"
        << "fitness=" << icp_fine.fitness_ * 100.0 << "\n"
        << "rmse=" << icp_fine.inlier_rmse_ << "\n"
        << "coarse_ms=" << coarse_cost_ms << "\n"
        << "fine_ms=" << icp_cost_ms << "\n";
    ofs.close();

    std::string plot_cmd = "PYTHONNOUSERSITE=1 python3 /home/wgk/fast_lio_ws/tools/plot_reloc_result.py " + map_path + 
                           " " + eval_dir + "/scan_coarse.pcd " + eval_dir + "/scan_fine.pcd " + 
                           eval_dir + "/info.txt /home/wgk/maps/reloc_process_result.png";
    system(plot_cmd.c_str());
    system("cp /home/wgk/maps/reloc_process_result.png /home/wgk/.gemini/antigravity/brain/7feabc36-0275-47da-90e0-5b6a575d001d/reloc_process_result.png 2>/dev/null || true");
    std::cout << "🖼️  匹配过程高清对比图已保存至: /home/wgk/maps/reloc_process_result.png\n";

    if (icp_fine.fitness_ > 0.85 && icp_fine.inlier_rmse_ < 0.08) {
        std::cout << "✅ 【重定位成功】开机特征匹配成功锁定真实全局坐标！\n";
        return 0;
    } else {
        std::cout << "⚠️ 定位收敛完成，但置信度较低 (Fitness=" << icp_fine.fitness_ * 100 << "%)\n";
        return 1;
    }
}
