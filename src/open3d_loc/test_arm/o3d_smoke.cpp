// Open3D aarch64 无头冒烟测试
// 目的：验证 libOpen3D.a 在香橙派上能链接、能跑我们要用的三个函数
//   1) VoxelDownSample
//   2) RegistrationICP
//   3) EvaluateRegistration
#include <cstdio>

#include <Eigen/Geometry>
#include <open3d/Open3D.h>

int main() {
    printf("=== Open3D 无头冒烟测试 ===\n");

    // 合成一个 5m x 5m 的平面点云
    auto src = std::make_shared<open3d::geometry::PointCloud>();
    for (int i = 0; i < 100; ++i)
        for (int j = 0; j < 100; ++j)
            src->points_.emplace_back(i * 0.05, j * 0.05, 0.0);
    printf("  源点云: %zu 点\n", src->points_.size());

    // 施加已知变换（平移 0.5/0.3，绕 z 转 0.2 rad）
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    T(0, 3) = 0.5;
    T(1, 3) = 0.3;

    auto tgt = std::make_shared<open3d::geometry::PointCloud>();
    *tgt = *src;
    tgt->Transform(T);

    // 1) 体素下采样
    auto ds = src->VoxelDownSample(0.1);
    auto dt = tgt->VoxelDownSample(0.1);
    printf("  VoxelDownSample: %zu / %zu 点\n", ds->points_.size(), dt->points_.size());
    if (ds->points_.empty() || dt->points_.empty()) {
        printf("  ❌ 下采样结果为空\n");
        return 1;
    }

    // 2) ICP
    auto res = open3d::pipelines::registration::RegistrationICP(
        *ds, *dt, 0.2, Eigen::Matrix4d::Identity(),
        open3d::pipelines::registration::TransformationEstimationPointToPoint());
    printf("  RegistrationICP: fitness=%.4f inlier_rmse=%.4f\n", res.fitness_, res.inlier_rmse_);
    printf("    估计平移 = (%.3f, %.3f, %.3f)   真值 = (0.500, 0.300, 0.000)\n",
           res.transformation_(0, 3), res.transformation_(1, 3), res.transformation_(2, 3));

    // 3) 配准评估
    auto ev = open3d::pipelines::registration::EvaluateRegistration(
        *ds, *dt, 0.2, res.transformation_);
    printf("  EvaluateRegistration: fitness=%.4f\n", ev.fitness_);

    // 4) 法向估计（open3d_loc 里也用到）
    auto withn = std::make_shared<open3d::geometry::PointCloud>(*ds);
    withn->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.2, 30));
    printf("  EstimateNormals: has_normals=%d\n", (int)withn->HasNormals());

    double err = std::abs(res.transformation_(0, 3) - 0.5) + std::abs(res.transformation_(1, 3) - 0.3);
    if (err < 0.05)
        printf("\n✅ Open3D 在 aarch64 上工作正常（平移恢复误差 %.4f m）\n", err);
    else {
        printf("\n⚠️ 能跑但配准结果偏差 %.4f m\n", err);
        return 2;
    }
    return 0;
}
