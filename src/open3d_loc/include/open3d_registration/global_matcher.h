#pragma once

#include <string>
#include <vector>
#include <memory>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <open3d/Open3D.h>

namespace open3d_loc {

// 2D 距离场似然度特征网格（支持二进制秒级持久化缓存）
struct MapDistanceField2D {
    float min_x = 0, max_x = 0, min_y = 0, max_y = 0;
    float resolution = 0.15f;
    int nx = 0, ny = 0;
    std::vector<float> lut; // 预计算的高斯似然度表: exp(-d^2 / (2 * sigma^2))

    bool Save(const std::string& filepath) const;
    bool Load(const std::string& filepath);
    void BuildFromPointCloud(const open3d::geometry::PointCloud& cloud, 
                             float res = 0.15f, float sigma = 0.20f, float max_dist = 0.80f);
};

// 重定位解算输出结构体
struct RelocResult {
    bool success = false;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0; // 偏航角 (单位: 度)
    Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();
    double fitness = 0.0;    // 点云重合度 (0.0 ~ 1.0)
    double rmse = 0.0;       // 点到面均方根误差 (米)
    double time_cache_ms = 0.0;
    double time_coarse_ms = 0.0;
    double time_fine_ms = 0.0;
    double time_total_ms = 0.0;
};

// 独立开机全局重定位匹配器核心类
class GlobalMatcher {
public:
    GlobalMatcher() = default;
    ~GlobalMatcher() = default;

    /// @brief 加载先验点云地图并建立/加载特征距离场缓存
    /// @param map_path 先验 PCD 地图绝对路径
    /// @param voxel_size 精配准体素分辨率 (默认 0.10m)
    /// @return 是否成功
    bool LoadMap(const std::string& map_path, double voxel_size = 0.10);

    /// @brief 开机单帧盲开重定位解算 (零先验初值)
    /// @param scan 单帧雷达点云 (自身局部系)
    /// @return 重定位结果
    RelocResult Relocalize(const std::shared_ptr<open3d::geometry::PointCloud>& scan);

private:
    std::string map_path_;
    double voxel_size_ = 0.10;
    std::shared_ptr<open3d::geometry::PointCloud> map_cloud_;
    std::shared_ptr<open3d::geometry::PointCloud> map_down_;
    MapDistanceField2D df_;
    bool map_loaded_ = false;
};

} // namespace open3d_loc

