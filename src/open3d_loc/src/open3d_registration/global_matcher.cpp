#include "open3d_registration/global_matcher.h"
#include <fstream>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <iostream>

namespace open3d_loc {

bool MapDistanceField2D::Save(const std::string& filepath) const {
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

bool MapDistanceField2D::Load(const std::string& filepath) {
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

void MapDistanceField2D::BuildFromPointCloud(const open3d::geometry::PointCloud& cloud, 
                                             float res, float sigma, float max_dist) {
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

bool GlobalMatcher::LoadMap(const std::string& map_path, double voxel_size) {
    map_path_ = map_path;
    voxel_size_ = voxel_size;
    map_cloud_ = std::make_shared<open3d::geometry::PointCloud>();

    if (!open3d::io::ReadPointCloud(map_path, *map_cloud_) || map_cloud_->IsEmpty()) {
        std::cerr << "❌ [GlobalMatcher] 无法读取先验地图: " << map_path << std::endl;
        return false;
    }

    std::string cache_path = map_path + ".df.bin";
    if (df_.Load(cache_path)) {
        std::cout << "✅ [GlobalMatcher] 从缓存快速加载距离场索引: " << cache_path << std::endl;
    } else {
        std::cout << "⏳ [GlobalMatcher] 首次生成先验地图距离场索引 (将自动缓存)..." << std::endl;
        df_.BuildFromPointCloud(*map_cloud_, 0.15f, 0.20f, 0.80f);
        df_.Save(cache_path);
        std::cout << "✅ [GlobalMatcher] 距离场索引已缓存至: " << cache_path << std::endl;
    }

    // 预处理降采样与法向量
    map_down_ = map_cloud_->VoxelDownSample(voxel_size_);
    map_down_->normals_.clear();
    map_down_->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(voxel_size_ * 2.0, 30));

    map_loaded_ = true;
    return true;
}

RelocResult GlobalMatcher::Relocalize(const std::shared_ptr<open3d::geometry::PointCloud>& scan) {
    RelocResult res;
    if (!map_loaded_ || !scan || scan->IsEmpty()) {
        std::cerr << "❌ [GlobalMatcher] 地图未就绪或激光帧为空！" << std::endl;
        return res;
    }

    auto t_online_start = std::chrono::high_resolution_clock::now();

    // 1. 提取当前单帧雷达水平切片特征点
    std::vector<Eigen::Vector2d> scan_sub;
    int step = std::max(1, (int)scan->points_.size() / 150);
    for (size_t i = 0; i < scan->points_.size(); i += step) {
        if (scan->points_[i].z() > -0.8 && scan->points_[i].z() < 0.8) {
            scan_sub.push_back(Eigen::Vector2d(scan->points_[i].x(), scan->points_[i].y()));
        }
    }

    // 2. 并行 360° 无先验全图似然度粗搜索
    int n_yaw = 72; // 5度航向分辨率
    struct Candidate {
        float score;
        float x, y, yaw;
    };
    std::vector<Candidate> best_per_yaw(n_yaw, {-1.0f, 0, 0, 0});

    int stride = 2; // 30cm 步长
    const float inv_res = 1.0f / df_.resolution;
    const float* lut_ptr = df_.lut.data();
    const int nx = df_.nx;
    const int ny = df_.ny;
    const float min_x = df_.min_x;
    const float min_y = df_.min_y;

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
            float y_world = min_y + iy * df_.resolution;
            for (int ix = 0; ix < nx; ix += stride) {
                float x_world = min_x + ix * df_.resolution;

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
    res.time_coarse_ms = std::chrono::duration<double, std::milli>(t_coarse_end - t_online_start).count();

    // 3. 两级点到面 ICP 精修
    double rad_coarse = best_global.yaw * M_PI / 180.0;
    Eigen::Matrix4d T_coarse = Eigen::Matrix4d::Identity();
    T_coarse(0, 0) = std::cos(rad_coarse);
    T_coarse(0, 1) = -std::sin(rad_coarse);
    T_coarse(1, 0) = std::sin(rad_coarse);
    T_coarse(1, 1) = std::cos(rad_coarse);
    T_coarse(0, 3) = best_global.x;
    T_coarse(1, 3) = best_global.y;

    auto scan_down = scan->VoxelDownSample(voxel_size_);

    // 粗 ICP：1.5m 阈值拉回
    auto icp_coarse = open3d::pipelines::registration::RegistrationICP(
        *scan_down, *map_down_, 1.5, T_coarse,
        open3d::pipelines::registration::TransformationEstimationPointToPlane(),
        open3d::pipelines::registration::ICPConvergenceCriteria(1e-6, 1e-6, 30));

    // 精 ICP：0.25m 阈值咬合
    auto icp_fine = open3d::pipelines::registration::RegistrationICP(
        *scan_down, *map_down_, 0.25, icp_coarse.transformation_,
        open3d::pipelines::registration::TransformationEstimationPointToPlane(),
        open3d::pipelines::registration::ICPConvergenceCriteria(1e-6, 1e-6, 30));

    auto t_online_end = std::chrono::high_resolution_clock::now();
    res.time_fine_ms = std::chrono::duration<double, std::milli>(t_online_end - t_coarse_end).count();
    res.time_total_ms = std::chrono::duration<double, std::milli>(t_online_end - t_online_start).count();

    res.transformation = icp_fine.transformation_;
    res.fitness = icp_fine.fitness_;
    res.rmse = icp_fine.inlier_rmse_;
    res.x = res.transformation(0, 3);
    res.y = res.transformation(1, 3);
    res.z = res.transformation(2, 3);
    res.yaw = std::atan2(res.transformation(1, 0), res.transformation(0, 0)) * 180.0 / M_PI;

    // 判定成功标准：重合度 >= 85% 且 RMSE <= 8cm
    res.success = (res.fitness >= 0.85 && res.rmse <= 0.08);

    return res;
}

} // namespace open3d_loc

