#include "open3d_registration/global_matcher.h"
#include <fstream>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <unordered_map>
#include <cstdlib>

namespace open3d_loc {

bool MapDistanceField2D::Save(const std::string& filepath) const {
    std::ofstream ofs(filepath, std::ios::binary);
    if (!ofs.is_open()) return false;
    uint32_t magic = MAGIC_V2 + 2; // DFV4: 保留全部室内实验台/障碍物立面 + 剔除地板
    ofs.write(reinterpret_cast<const char*>(&magic), sizeof(uint32_t));
    ofs.write(reinterpret_cast<const char*>(&min_x), sizeof(float));
    ofs.write(reinterpret_cast<const char*>(&max_x), sizeof(float));
    ofs.write(reinterpret_cast<const char*>(&min_y), sizeof(float));
    ofs.write(reinterpret_cast<const char*>(&max_y), sizeof(float));
    ofs.write(reinterpret_cast<const char*>(&resolution), sizeof(float));
    ofs.write(reinterpret_cast<const char*>(&z_floor), sizeof(float));
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
    uint32_t magic = 0;
    ifs.read(reinterpret_cast<char*>(&magic), sizeof(uint32_t));
    if (magic != MAGIC_V2 + 2) {
        return false; // 自动触发重建包含全部室内障碍物立面的距离场缓存
    }
    ifs.read(reinterpret_cast<char*>(&min_x), sizeof(float));
    ifs.read(reinterpret_cast<char*>(&max_x), sizeof(float));
    ifs.read(reinterpret_cast<char*>(&min_y), sizeof(float));
    ifs.read(reinterpret_cast<char*>(&max_y), sizeof(float));
    ifs.read(reinterpret_cast<char*>(&resolution), sizeof(float));
    ifs.read(reinterpret_cast<char*>(&z_floor), sizeof(float));
    ifs.read(reinterpret_cast<char*>(&nx), sizeof(int));
    ifs.read(reinterpret_cast<char*>(&ny), sizeof(int));
    size_t sz = 0;
    ifs.read(reinterpret_cast<char*>(&sz), sizeof(size_t));
    if (sz == 0 || sz > 50000000) return false;
    lut.resize(sz);
    ifs.read(reinterpret_cast<char*>(lut.data()), sz * sizeof(float));
    return ifs.good();
}

void MapDistanceField2D::BuildFromWallCloud(const open3d::geometry::PointCloud& wall_cloud,
                                            float floor_z,
                                            float res,
                                            float sigma,
                                            float max_pos_dist,
                                            float penalty_dist,
                                            float penalty_val) {
    resolution = res;
    z_floor = floor_z;
    min_x = 1e9f; max_x = -1e9f;
    min_y = 1e9f; max_y = -1e9f;

    // 使用 10cm 2D 网格统计每个柱体的高度跨度 (z_min, z_max) 与点数，剔除矮桌面/椅子/孤立噪点
    const float cell_sz = 0.10f;
    struct GridKey {
        int gx, gy;
        bool operator==(const GridKey& o) const { return gx == o.gx && gy == o.gy; }
    };
    struct GridHash {
        size_t operator()(const GridKey& k) const {
            return (static_cast<size_t>(static_cast<uint32_t>(k.gx)) << 32) ^
                   static_cast<uint32_t>(k.gy);
        }
    };
    struct CellAcc {
        int count = 0;
        double sum_x = 0.0;
        double sum_y = 0.0;
        double min_z = 1e9;
        double max_z = -1e9;
    };
    std::unordered_map<GridKey, CellAcc, GridHash> grid_acc;
    grid_acc.reserve(wall_cloud.points_.size() / 2 + 1);

    for (const auto& p : wall_cloud.points_) {
        if (!std::isfinite(p.x()) || !std::isfinite(p.y()) || !std::isfinite(p.z())) continue;
        int gx = static_cast<int>(std::floor(p.x() / cell_sz));
        int gy = static_cast<int>(std::floor(p.y() / cell_sz));
        auto& entry = grid_acc[{gx, gy}];
        entry.count += 1;
        entry.sum_x += p.x();
        entry.sum_y += p.y();
        if (p.z() < entry.min_z) entry.min_z = p.z();
        if (p.z() > entry.max_z) entry.max_z = p.z();
    }

    std::vector<Eigen::Vector2d> obstacle_pts;
    obstacle_pts.reserve(grid_acc.size());
    for (const auto& kv : grid_acc) {
        const auto& c = kv.second;
        double z_span = c.max_z - c.min_z;
        double h_max = c.max_z - floor_z;
        // 保留全部室内障碍物/实验台/机柜立面与墙壁 (h_max >= 0.30m) 且具备垂直延伸度或足够点密度
        if (c.count < 2 || h_max < 0.30 || (z_span < 0.14 && c.count < 3)) {
            continue;
        }
        Eigen::Vector2d pt(c.sum_x / static_cast<double>(c.count),
                           c.sum_y / static_cast<double>(c.count));
        if (pt.x() < min_x) min_x = static_cast<float>(pt.x());
        if (pt.x() > max_x) max_x = static_cast<float>(pt.x());
        if (pt.y() < min_y) min_y = static_cast<float>(pt.y());
        if (pt.y() > max_y) max_y = static_cast<float>(pt.y());
        obstacle_pts.push_back(pt);
    }

    min_x -= 1.0f; max_x += 1.0f;
    min_y -= 1.0f; max_y += 1.0f;
    nx = static_cast<int>(std::ceil((max_x - min_x) / res));
    ny = static_cast<int>(std::ceil((max_y - min_y) / res));
    lut.assign(nx * ny, penalty_val);

    if (obstacle_pts.empty()) return;

    auto obs_cloud = std::make_shared<open3d::geometry::PointCloud>();
    obs_cloud->points_.reserve(obstacle_pts.size());
    for (const auto& pt : obstacle_pts) {
        obs_cloud->points_.push_back(Eigen::Vector3d(pt.x(), pt.y(), 0.0));
    }
    open3d::geometry::KDTreeFlann kdtree(*obs_cloud);

    #pragma omp parallel for
    for (int iy = 0; iy < ny; ++iy) {
        float y = min_y + iy * res;
        std::vector<int> idx(1);
        std::vector<double> d2(1);
        for (int ix = 0; ix < nx; ++ix) {
            float x = min_x + ix * res;
            kdtree.SearchKNN(Eigen::Vector3d(x, y, 0.0), 1, idx, d2);
            float dist = std::sqrt(static_cast<float>(d2[0]));
            if (dist <= max_pos_dist) {
                lut[iy * nx + ix] = std::exp(-(dist * dist) / (2.0f * sigma * sigma));
            } else if (dist <= penalty_dist) {
                lut[iy * nx + ix] = 0.0f;
            } else {
                lut[iy * nx + ix] = penalty_val;
            }
        }
    }
}

void GlobalMatcher::EstimateFloorAndLeveling(const open3d::geometry::PointCloud& cloud,
                                             double& z_floor_out,
                                             Eigen::Matrix3d& R_level_out) {
    R_level_out = Eigen::Matrix3d::Identity();
    z_floor_out = -0.45;
    if (cloud.points_.empty()) return;

    // 1. 按 25cm x 25cm XY 网格提取每个柱体的最低点 Z (彻底消除 VLP-16 0° 水平激光环在 Z=0 处的数量偏置！)
    struct CellKey {
        int gx, gy;
        bool operator==(const CellKey& o) const { return gx == o.gx && gy == o.gy; }
    };
    struct CellHash {
        size_t operator()(const CellKey& k) const {
            return (static_cast<size_t>(static_cast<uint32_t>(k.gx)) << 32) ^
                   static_cast<uint32_t>(k.gy);
        }
    };
    std::unordered_map<CellKey, double, CellHash> cell_min_z;
    cell_min_z.reserve(2048);

    const double cell_w = 0.25;
    for (const auto& p : cloud.points_) {
        if (!std::isfinite(p.x()) || !std::isfinite(p.y()) || !std::isfinite(p.z())) continue;
        double r2 = p.x() * p.x() + p.y() * p.y();
        if (r2 > 0.40 * 0.40 && r2 < 7.0 * 7.0 && p.z() >= -1.60 && p.z() <= 0.15) {
            int gx = static_cast<int>(std::floor(p.x() / cell_w));
            int gy = static_cast<int>(std::floor(p.y() / cell_w));
            auto it = cell_min_z.find({gx, gy});
            if (it == cell_min_z.end() || p.z() < it->second) {
                cell_min_z[{gx, gy}] = p.z();
            }
        }
    }

    const double z_min = -1.60, z_max = 0.15, bin_w = 0.04;
    const int n_bins = static_cast<int>((z_max - z_min) / bin_w);
    std::vector<int> hist(n_bins, 0);

    for (const auto& kv : cell_min_z) {
        int b = static_cast<int>((kv.second - z_min) / bin_w);
        if (b >= 0 && b < n_bins) hist[b]++;
    }

    int max_count = 0;
    int best_bin = n_bins / 2;
    for (int i = 1; i < n_bins - 1; ++i) {
        int s = hist[i - 1] + 2 * hist[i] + hist[i + 1];
        if (s > max_count) {
            max_count = s;
            best_bin = i;
        }
    }

    double z_rough = z_min + (best_bin + 0.5) * bin_w;

    // 2. 提取地板附近内点并用 PCA 估计地板法向量与均值高度
    std::vector<Eigen::Vector3d> floor_pts;
    floor_pts.reserve(4000);
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    for (const auto& p : cloud.points_) {
        if (!std::isfinite(p.x()) || !std::isfinite(p.y()) || !std::isfinite(p.z())) continue;
        double r2 = p.x() * p.x() + p.y() * p.y();
        if (r2 > 0.40 * 0.40 && r2 < 6.0 * 6.0 && std::abs(p.z() - z_rough) < 0.08) {
            floor_pts.push_back(p);
            mean += p;
        }
    }

    if (floor_pts.size() < 80) {
        z_floor_out = z_rough;
        return;
    }

    mean /= static_cast<double>(floor_pts.size());
    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    for (const auto& p : floor_pts) {
        Eigen::Vector3d d = p - mean;
        cov += d * d.transpose();
    }
    cov /= static_cast<double>(floor_pts.size());

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);
    Eigen::Vector3d normal = es.eigenvectors().col(0);
    if (normal.z() < 0) normal = -normal;

    if (normal.z() > std::cos(12.0 * M_PI / 180.0)) {
        R_level_out = Eigen::Quaterniond::FromTwoVectors(normal, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        double z_sum = 0.0;
        for (const auto& p : floor_pts) {
            z_sum += (R_level_out * p).z();
        }
        z_floor_out = z_sum / static_cast<double>(floor_pts.size());
    } else {
        z_floor_out = mean.z();
    }
}

std::shared_ptr<open3d::geometry::PointCloud> GlobalMatcher::ExtractVerticalWallCloud(
    const open3d::geometry::PointCloud& cloud_with_normals,
    double z_floor,
    double min_height_above_floor,
    double max_height_above_floor,
    double max_abs_normal_z) {
    auto walls = std::make_shared<open3d::geometry::PointCloud>();
    if (!cloud_with_normals.HasNormals()) return walls;

    walls->points_.reserve(cloud_with_normals.points_.size() / 2);
    walls->normals_.reserve(cloud_with_normals.points_.size() / 2);

    for (size_t i = 0; i < cloud_with_normals.points_.size(); ++i) {
        const auto& p = cloud_with_normals.points_[i];
        const auto& n = cloud_with_normals.normals_[i];
        if (!std::isfinite(p.x()) || !std::isfinite(p.y()) || !std::isfinite(p.z())) continue;
        double h = p.z() - z_floor;
        if (h >= min_height_above_floor && h <= max_height_above_floor &&
            std::isfinite(n.z()) && std::abs(n.z()) < max_abs_normal_z) {
            Eigen::Vector3d n_horiz(n.x(), n.y(), 0.0);
            double norm = n_horiz.norm();
            if (norm > 1e-3) {
                walls->points_.push_back(p);
                walls->normals_.push_back(n_horiz / norm);
            }
        }
    }
    return walls;
}

bool GlobalMatcher::LoadMap(const std::string& map_path, double voxel_size) {
    map_path_ = map_path;
    voxel_size_ = voxel_size;
    map_cloud_ = std::make_shared<open3d::geometry::PointCloud>();

    if (!open3d::io::ReadPointCloud(map_path, *map_cloud_) || map_cloud_->IsEmpty()) {
        std::cerr << "❌ [GlobalMatcher] 无法读取先验地图: " << map_path << std::endl;
        return false;
    }

    // 1. 预处理降采样与法向量估计
    map_down_ = map_cloud_->VoxelDownSample(voxel_size_);
    map_down_->normals_.clear();
    map_down_->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(voxel_size_ * 2.5, 35));

    // 2. 自动检测先验地图基准地面高度并提取纯垂直墙面点云
    double map_floor_z = -0.49;
    Eigen::Matrix3d R_map_level = Eigen::Matrix3d::Identity();
    EstimateFloorAndLeveling(*map_down_, map_floor_z, R_map_level);

    // 提取 [0.22m, 1.85m] 垂直立面点云（剔除水平地面，完整保留实验台、机柜、立柱与墙壁）
    map_walls_ = ExtractVerticalWallCloud(*map_down_, map_floor_z, 0.22, 1.85, 0.50);

    // 3. 加载或构建 V2 纯墙面正负距离场缓存 (.df_v2.bin)
    std::string cache_path = map_path + ".df_v2.bin";
    if (df_.Load(cache_path)) {
        std::cout << "✅ [GlobalMatcher V2] 从缓存快速加载纯墙面距离场索引: " << cache_path
                  << " (地面基准 Z=" << df_.z_floor << "m, 墙面点=" << map_walls_->points_.size() << ")" << std::endl;
    } else {
        std::cout << "⏳ [GlobalMatcher V2] 正在构建纯建筑墙面正负距离场索引 (彻底剔除地板与矮桌面干扰)..." << std::endl;
        df_.BuildFromWallCloud(*map_walls_, static_cast<float>(map_floor_z), 0.10f, 0.14f, 0.32f, 0.45f, -0.25f);
        df_.Save(cache_path);
        std::cout << "✅ [GlobalMatcher V2] 纯墙面距离场已缓存至: " << cache_path
                  << " (网格: " << df_.nx << "x" << df_.ny << ", 地面 Z=" << df_.z_floor << "m)" << std::endl;
    }

    map_loaded_ = true;
    return true;
}

RelocResult GlobalMatcher::Relocalize(const std::shared_ptr<open3d::geometry::PointCloud>& scan,
                                      double prior_x,
                                      double prior_y,
                                      double prior_radius) {
    RelocResult res;
    if (!map_loaded_ || !scan || scan->IsEmpty()) {
        std::cerr << "❌ [GlobalMatcher] 地图未就绪或激光帧为空！" << std::endl;
        return res;
    }

    auto t_online_start = std::chrono::high_resolution_clock::now();

    // 1. 自动估计当前雷达帧地面高度与水平倾角 (基于 25cm 栅格最低点直方图，彻底免疫 0° 水平光束偏置)
    double scan_floor_z = -0.45;
    Eigen::Matrix3d R_scan_level = Eigen::Matrix3d::Identity();
    EstimateFloorAndLeveling(*scan, scan_floor_z, R_scan_level);

    Eigen::Matrix4d T_level = Eigen::Matrix4d::Identity();
    T_level.block<3, 3>(0, 0) = R_scan_level;
    T_level(2, 3) = static_cast<double>(df_.z_floor) - scan_floor_z;

    auto scan_leveled = std::make_shared<open3d::geometry::PointCloud>(*scan);
    scan_leveled->Transform(T_level);

    // 2. 下采样并估计当前帧法向量，提取纯垂直墙面 3D 点云
    auto scan_down = scan_leveled->VoxelDownSample(voxel_size_);
    scan_down->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(voxel_size_ * 2.5, 30));

    // 用于 3D 障碍物与墙面 ICP 的立面点云 (距地面 [0.22m, 1.85m])
    auto scan_walls_3d = ExtractVerticalWallCloud(*scan_down, df_.z_floor, 0.22, 1.85, 0.50);
    if (scan_walls_3d->points_.size() < 40) {
        for (const auto& p : scan_down->points_) {
            double h = p.z() - df_.z_floor;
            if (h >= 0.22 && h <= 1.85) {
                scan_walls_3d->points_.push_back(p);
                scan_walls_3d->normals_.push_back(Eigen::Vector3d(1, 0, 0));
            }
        }
    }

    // 3. 提取障碍物与墙面切片 (距地面 [0.22m, 1.85m]，包含实验台、机柜与外墙) 并做 0.12m 2D 均匀体素采样
    const float voxel_2d = 0.12f;
    struct Cell2D {
        int x, y;
        bool operator==(const Cell2D& o) const { return x == o.x && y == o.y; }
    };
    struct Cell2DHash {
        size_t operator()(const Cell2D& k) const {
            return (static_cast<size_t>(static_cast<uint32_t>(k.x)) << 32) ^
                   static_cast<uint32_t>(k.y);
        }
    };
    struct Cell2DAcc {
        int count = 0;
        double sum_x = 0.0;
        double sum_y = 0.0;
    };
    std::unordered_map<Cell2D, Cell2DAcc, Cell2DHash> cells_2d;
    cells_2d.reserve(scan_walls_3d->points_.size());

    // 使用高于地面 0.22m 的全部障碍物立面与墙壁构建 2D 扫描轮廓
    double min_h_2d = 0.22;
    for (int pass = 0; pass < 1; ++pass) {
        cells_2d.clear();
        for (const auto& p : scan_walls_3d->points_) {
            if (!std::isfinite(p.x()) || !std::isfinite(p.y()) || !std::isfinite(p.z())) continue;
            double h = p.z() - df_.z_floor;
            if (h < min_h_2d || h > 1.85) continue;
            double r2 = p.x() * p.x() + p.y() * p.y();
            if (r2 < 0.35 * 0.35 || r2 > 14.0 * 14.0) continue;
            int cx = static_cast<int>(std::floor(p.x() / voxel_2d));
            int cy = static_cast<int>(std::floor(p.y() / voxel_2d));
            auto& c = cells_2d[{cx, cy}];
            c.count += 1;
            c.sum_x += p.x();
            c.sum_y += p.y();
        }
    }

    std::vector<Eigen::Vector2f> scan_sub;
    scan_sub.reserve(cells_2d.size());
    for (const auto& kv : cells_2d) {
        double avg_x = kv.second.sum_x / static_cast<double>(kv.second.count);
        double avg_y = kv.second.sum_y / static_cast<double>(kv.second.count);
        scan_sub.push_back(Eigen::Vector2f(static_cast<float>(avg_x), static_cast<float>(avg_y)));
    }

    // 4. 并行 360° (3° 步长, 120 航向) 全图正负似然度匹配 + 多峰候选收集
    const int n_yaw = 120;
    struct Candidate {
        float score;
        float x, y, yaw;
    };
    std::vector<std::vector<Candidate>> cands_per_yaw(n_yaw);

    const int stride = (prior_radius > 0 && prior_radius <= 4.0) ? 1 : 2;
    const float inv_res = 1.0f / df_.resolution;
    const float* lut_ptr = df_.lut.data();
    const int nx = df_.nx;
    const int ny = df_.ny;
    const float min_x = df_.min_x;
    const float min_y = df_.min_y;
    const float out_of_bounds_penalty = -0.25f;

    #pragma omp parallel for schedule(dynamic)
    for (int a = 0; a < n_yaw; ++a) {
        double yaw = a * (2.0 * M_PI / n_yaw);
        float c = static_cast<float>(std::cos(yaw));
        float s = static_cast<float>(std::sin(yaw));

        std::vector<Eigen::Vector2f> rot_pts(scan_sub.size());
        for (size_t i = 0; i < scan_sub.size(); ++i) {
            rot_pts[i] = Eigen::Vector2f(c * scan_sub[i].x() - s * scan_sub[i].y(),
                                         s * scan_sub[i].x() + c * scan_sub[i].y());
        }

        std::vector<Candidate> local_top;
        local_top.reserve(8);

        for (int iy = 0; iy < ny; iy += stride) {
            float y_world = min_y + iy * df_.resolution;
            if (prior_radius > 0 && std::abs(y_world - prior_y) > prior_radius) continue;

            for (int ix = 0; ix < nx; ix += stride) {
                float x_world = min_x + ix * df_.resolution;
                if (prior_radius > 0 && std::abs(x_world - prior_x) > prior_radius) continue;

                float score = 0.0f;
                for (const auto& pt : rot_pts) {
                    int px = static_cast<int>((pt.x() + x_world - min_x) * inv_res);
                    int py = static_cast<int>((pt.y() + y_world - min_y) * inv_res);
                    if (px >= 0 && px < nx && py >= 0 && py < ny) {
                        score += lut_ptr[py * nx + px];
                    } else {
                        score += out_of_bounds_penalty;
                    }
                }

                if (local_top.size() < 3 || score > local_top.back().score) {
                    Candidate cand{score, x_world, y_world, static_cast<float>(yaw * 180.0 / M_PI)};
                    bool merged = false;
                    for (auto& ex : local_top) {
                        if (std::hypot(ex.x - cand.x, ex.y - cand.y) < 0.8f) {
                            if (cand.score > ex.score) ex = cand;
                            merged = true;
                            break;
                        }
                    }
                    if (!merged) {
                        local_top.push_back(cand);
                    }
                    std::sort(local_top.begin(), local_top.end(),
                              [](const Candidate& lhs, const Candidate& rhs) { return lhs.score > rhs.score; });
                    if (local_top.size() > 3) local_top.pop_back();
                }
            }
        }
        cands_per_yaw[a] = std::move(local_top);
    }

    // 汇总所有候选并执行 3D (X, Y, Yaw) 非极大值抑制 (NMS)，选出 Top-K (K=12) 独立假设
    std::vector<Candidate> all_cands;
    all_cands.reserve(n_yaw * 3);
    for (const auto& vec : cands_per_yaw) {
        for (const auto& c : vec) all_cands.push_back(c);
    }
    std::sort(all_cands.begin(), all_cands.end(),
              [](const Candidate& a, const Candidate& b) { return a.score > b.score; });

    const size_t TOP_K = 12;
    std::vector<Candidate> top_k_cands;
    top_k_cands.reserve(TOP_K);
    for (const auto& c : all_cands) {
        bool suppressed = false;
        for (const auto& kept : top_k_cands) {
            float dist_xy = std::hypot(c.x - kept.x, c.y - kept.y);
            float diff_yaw = std::abs(c.yaw - kept.yaw);
            if (diff_yaw > 180.0f) diff_yaw = 360.0f - diff_yaw;
            if (dist_xy < 0.80f && diff_yaw < 25.0f) {
                suppressed = true;
                break;
            }
        }
        if (!suppressed) {
            top_k_cands.push_back(c);
            if (top_k_cands.size() >= TOP_K) break;
        }
    }

    auto t_coarse_end = std::chrono::high_resolution_clock::now();
    res.time_coarse_ms = std::chrono::duration<double, std::milli>(t_coarse_end - t_online_start).count();

    if (top_k_cands.empty()) return res;

    // 5. 对 Top-K 独立候选并行执行 [第一级: 纯墙面 ICP] + [第二级: 全自由度精修 ICP] + [纯墙面几何防伪校验]
    struct RefinedHypothesis {
        double composite_score = -1e9;
        double wall_fitness = 0.0;
        double wall_rmse = 1e9;
        double full_fitness = 0.0;
        double full_rmse = 1e9;
        Candidate coarse_cand;
        Eigen::Matrix4d T_coarse_full = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d T_final_full = Eigen::Matrix4d::Identity();
    };

    std::vector<RefinedHypothesis> refined_list(top_k_cands.size());

    #pragma omp parallel for schedule(dynamic)
    for (size_t k = 0; k < top_k_cands.size(); ++k) {
        const auto& cand = top_k_cands[k];
        double rad = cand.yaw * M_PI / 180.0;
        Eigen::Matrix4d T_xy_yaw = Eigen::Matrix4d::Identity();
        T_xy_yaw(0, 0) = std::cos(rad);
        T_xy_yaw(0, 1) = -std::sin(rad);
        T_xy_yaw(1, 0) = std::sin(rad);
        T_xy_yaw(1, 1) = std::cos(rad);
        T_xy_yaw(0, 3) = cand.x;
        T_xy_yaw(1, 3) = cand.y;

        // 阶段 A：纯墙面 Point-to-Plane ICP (完全不受地面干扰，精准拉回 X, Y, Yaw)
        auto icp_wall_coarse = open3d::pipelines::registration::RegistrationICP(
            *scan_walls_3d, *map_walls_, 0.75, T_xy_yaw,
            open3d::pipelines::registration::TransformationEstimationPointToPlane(),
            open3d::pipelines::registration::ICPConvergenceCriteria(1e-5, 1e-5, 25));

        auto icp_wall_fine = open3d::pipelines::registration::RegistrationICP(
            *scan_walls_3d, *map_walls_, 0.25, icp_wall_coarse.transformation_,
            open3d::pipelines::registration::TransformationEstimationPointToPlane(),
            open3d::pipelines::registration::ICPConvergenceCriteria(1e-6, 1e-6, 25));

        // 阶段 B：全点云 6-DoF 精修 ICP (利用地面锁定毫米级 Z/Roll/Pitch，利用墙面锁定 X/Y/Yaw)
        auto icp_full = open3d::pipelines::registration::RegistrationICP(
            *scan_down, *map_down_, 0.20, icp_wall_fine.transformation_,
            open3d::pipelines::registration::TransformationEstimationPointToPlane(),
            open3d::pipelines::registration::ICPConvergenceCriteria(1e-6, 1e-6, 25));

        // 阶段 C：严格评估纯垂直墙面的真实重合度 (阈值 0.20m)
        auto eval_wall = open3d::pipelines::registration::EvaluateRegistration(
            *scan_walls_3d, *map_walls_, 0.20, icp_full.transformation_);

        RefinedHypothesis hyp;
        hyp.coarse_cand = cand;
        hyp.T_coarse_full = T_xy_yaw * T_level;
        hyp.T_final_full = icp_full.transformation_ * T_level;
        hyp.wall_fitness = eval_wall.fitness_;
        hyp.wall_rmse = (eval_wall.fitness_ > 0.05) ? eval_wall.inlier_rmse_ : 0.50;
        hyp.full_fitness = icp_full.fitness_;
        hyp.full_rmse = (icp_full.fitness_ > 0.05) ? icp_full.inlier_rmse_ : 0.50;

        // 综合评分：以纯墙面重合率、墙面残差与 2D 粗匹配似然度共同约束
        double norm_coarse = static_cast<double>(cand.score) / std::max<size_t>(1, scan_sub.size());
        hyp.composite_score = hyp.wall_fitness - 1.5 * hyp.wall_rmse + 0.25 * norm_coarse + 0.10 * hyp.full_fitness;
        refined_list[k] = hyp;
    }

    const RefinedHypothesis* best_hyp = &refined_list[0];
    for (size_t k = 1; k < refined_list.size(); ++k) {
        if (refined_list[k].composite_score > best_hyp->composite_score) {
            best_hyp = &refined_list[k];
        }
    }

    auto t_online_end = std::chrono::high_resolution_clock::now();
    res.time_fine_ms = std::chrono::duration<double, std::milli>(t_online_end - t_coarse_end).count();
    res.time_total_ms = std::chrono::duration<double, std::milli>(t_online_end - t_online_start).count();

    res.coarse_x = best_hyp->coarse_cand.x;
    res.coarse_y = best_hyp->coarse_cand.y;
    res.coarse_yaw = best_hyp->coarse_cand.yaw;
    res.coarse_transformation = best_hyp->T_coarse_full;

    res.transformation = best_hyp->T_final_full;
    res.fitness = best_hyp->full_fitness;
    res.rmse = best_hyp->full_rmse;
    res.wall_fitness = best_hyp->wall_fitness;
    res.wall_rmse = best_hyp->wall_rmse;

    res.x = res.transformation(0, 3);
    res.y = res.transformation(1, 3);
    res.z = res.transformation(2, 3);
    res.yaw = std::atan2(res.transformation(1, 0), res.transformation(0, 0)) * 180.0 / M_PI;
    res.pitch = std::asin(std::clamp(-res.transformation(2, 0), -1.0, 1.0)) * 180.0 / M_PI;
    res.roll = std::atan2(res.transformation(2, 1), res.transformation(2, 2)) * 180.0 / M_PI;

    // 真实墙面防伪判定标准：纯墙面重合度 >= 55% 且墙面 RMSE <= 10cm
    res.success = (res.wall_fitness >= 0.55 && res.wall_rmse <= 0.10);

    return res;
}

bool GlobalMatcher::SaveEvalAndPlot(const std::shared_ptr<open3d::geometry::PointCloud>& raw_scan,
                                    const RelocResult& res,
                                    const std::string& out_png_path,
                                    bool has_gt,
                                    double gt_x,
                                    double gt_y,
                                    double gt_yaw) const {
    if (!raw_scan || raw_scan->IsEmpty()) return false;

    std::string eval_dir = "/tmp/reloc_eval";
    int ret_mkdir = std::system(("mkdir -p " + eval_dir).c_str());
    (void)ret_mkdir;

    auto scan_down = raw_scan->VoxelDownSample(voxel_size_);

    auto scan_coarse_vis = std::make_shared<open3d::geometry::PointCloud>(*scan_down);
    scan_coarse_vis->Transform(res.coarse_transformation);
    open3d::io::WritePointCloud(eval_dir + "/scan_coarse.pcd", *scan_coarse_vis);

    auto scan_fine_vis = std::make_shared<open3d::geometry::PointCloud>(*scan_down);
    scan_fine_vis->Transform(res.transformation);
    open3d::io::WritePointCloud(eval_dir + "/scan_fine.pcd", *scan_fine_vis);

    std::ofstream ofs(eval_dir + "/info.txt");
    ofs << "has_gt=" << (has_gt ? 1 : 0) << "\n"
        << "gt_x=" << gt_x << "\n"
        << "gt_y=" << gt_y << "\n"
        << "gt_yaw=" << gt_yaw << "\n"
        << "z_floor=" << df_.z_floor << "\n"
        << "coarse_x=" << res.coarse_x << "\n"
        << "coarse_y=" << res.coarse_y << "\n"
        << "coarse_yaw=" << res.coarse_yaw << "\n"
        << "fine_x=" << res.x << "\n"
        << "fine_y=" << res.y << "\n"
        << "fine_z=" << res.z << "\n"
        << "fine_yaw=" << res.yaw << "\n"
        << "fitness=" << res.fitness * 100.0 << "\n"
        << "rmse=" << res.rmse << "\n"
        << "wall_fitness=" << res.wall_fitness * 100.0 << "\n"
        << "wall_rmse=" << res.wall_rmse << "\n"
        << "coarse_ms=" << res.time_coarse_ms << "\n"
        << "fine_ms=" << res.time_fine_ms << "\n";
    ofs.close();

    const char* home_env = std::getenv("HOME");
    std::string home_str = home_env ? std::string(home_env) : "";
    std::string script_path = home_str + "/fast_lio_ws/tools/plot_reloc_result.py";

    std::string plot_cmd = "PYTHONNOUSERSITE=1 python3 " + script_path + " " +
                           map_path_ + " " + eval_dir + "/scan_coarse.pcd " +
                           eval_dir + "/scan_fine.pcd " + eval_dir + "/info.txt " + out_png_path;
    int ret = std::system(plot_cmd.c_str());
    return (ret == 0);
}

} // namespace open3d_loc
