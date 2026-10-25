#pragma once

#include <string>
#include <vector>
#include <memory>
#include <cstdint>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <open3d/Open3D.h>

namespace open3d_loc {

// 2D 纯墙面正负距离场似然度特征网格（支持二进制秒级持久化缓存 V2）
struct MapDistanceField2D {
    static constexpr uint32_t MAGIC_V2 = 0x44465632; // "DFV2"
    float min_x = 0, max_x = 0, min_y = 0, max_y = 0;
    float resolution = 0.10f;
    float z_floor = 0.0f;
    int nx = 0, ny = 0;
    std::vector<float> lut; // 正负似然度表: 墙面区 exp(-d^2/(2*sigma^2)) ∈ (0,1], 空白悬空区 -0.25f

    bool Save(const std::string& filepath) const;
    bool Load(const std::string& filepath);
    void BuildFromWallCloud(const open3d::geometry::PointCloud& wall_cloud,
                            float floor_z,
                            float res = 0.10f,
                            float sigma = 0.14f,
                            float max_pos_dist = 0.35f,
                            float penalty_dist = 0.50f,
                            float penalty_val = -0.25f);
};

// 重定位解算输出结构体
struct RelocResult {
    bool success = false;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;        // 最终偏航角 (单位: 度)
    double coarse_x = 0.0;
    double coarse_y = 0.0;
    double coarse_yaw = 0.0; // 最佳候选粗匹配位姿
    Eigen::Matrix4d coarse_transformation = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();
    double fitness = 0.0;      // 全点云重合度 (0.0 ~ 1.0)
    double rmse = 0.0;         // 全点云均方根误差 (米)
    double wall_fitness = 0.0; // 纯垂直墙面重合度 (0.0 ~ 1.0) —— 真实几何防伪指标
    double wall_rmse = 0.0;    // 纯垂直墙面均方根误差 (米)
    double time_cache_ms = 0.0;
    double time_coarse_ms = 0.0;
    double time_fine_ms = 0.0;
    double time_total_ms = 0.0;
};

// 独立开机全局重定位匹配器核心类 (V2: 地面解耦 + 纯墙面正负距离场 + Top-K 多假设 ICP)
class GlobalMatcher {
public:
    GlobalMatcher() = default;
    ~GlobalMatcher() = default;

    /// @brief 加载先验点云地图并建立/加载 V2 纯墙面距离场缓存
    /// @param map_path 先验 PCD 地图绝对路径
    /// @param voxel_size 精配准体素分辨率 (默认 0.10m)
    /// @return 是否成功
    bool LoadMap(const std::string& map_path, double voxel_size = 0.10);

    /// @brief 开机盲开重定位解算 (支持全图零先验或局部半径搜索)
    /// @param scan 雷达点云 (自身局部系)
    /// @param prior_x 可选先验中心 X
    /// @param prior_y 可选先验中心 Y
    /// @param prior_radius 先验搜索半径 (< 0 表示 100% 全图盲搜)
    /// @return 重定位结果
    RelocResult Relocalize(const std::shared_ptr<open3d::geometry::PointCloud>& scan,
                           double prior_x = 0.0,
                           double prior_y = 0.0,
                           double prior_radius = -1.0);

    /// @brief 将配准结果导出至 /tmp/reloc_eval 并调用绘图脚本生成对比图
    bool SaveEvalAndPlot(const std::shared_ptr<open3d::geometry::PointCloud>& raw_scan,
                         const RelocResult& res,
                         const std::string& out_png_path,
                         bool has_gt = false,
                         double gt_x = 0.0,
                         double gt_y = 0.0,
                         double gt_yaw = 0.0) const;

    /// @brief 静态辅助工具：估计点云地面高度 z_floor 及水平对齐旋转矩阵 R_level
    static void EstimateFloorAndLeveling(const open3d::geometry::PointCloud& cloud,
                                         double& z_floor_out,
                                         Eigen::Matrix3d& R_level_out);

    /// @brief 静态辅助工具：从已带法向量的水平点云中提取纯垂直墙面 3D 点云
    static std::shared_ptr<open3d::geometry::PointCloud> ExtractVerticalWallCloud(
        const open3d::geometry::PointCloud& cloud_with_normals,
        double z_floor,
        double min_height_above_floor = 0.25,
        double max_height_above_floor = 1.45,
        double max_abs_normal_z = 0.45);

    double GetMapFloorZ() const { return df_.z_floor; }
    std::shared_ptr<open3d::geometry::PointCloud> GetMapDownCloud() const { return map_down_; }
    std::shared_ptr<open3d::geometry::PointCloud> GetMapWallCloud() const { return map_walls_; }

private:
    std::string map_path_;
    double voxel_size_ = 0.10;
    std::shared_ptr<open3d::geometry::PointCloud> map_cloud_;
    std::shared_ptr<open3d::geometry::PointCloud> map_down_;
    std::shared_ptr<open3d::geometry::PointCloud> map_walls_;
    MapDistanceField2D df_;
    bool map_loaded_ = false;
};

} // namespace open3d_loc
