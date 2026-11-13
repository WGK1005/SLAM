#pragma once
/**
 * @file scu_recovery_tracker.hpp
 * @brief SCU-TDT-V1 核心融合组件：
 *        1. 硬件坐标系转换器 (适配 map_1021_1050.pcd 全局坐标系与 2D 栅格图)
 *        2. VLP-16 实时 3D 点云动态掩码注入器 (连接 Fast-LIO /cloud_registered 与 YAstar 多层掩码)
 *        3. SCURM 风格“向自由空间智能脱困”恢复算法 (SmartFreeSpaceRecovery)
 *        4. Minimum Snap 连续轨迹前馈+纯跟踪控制器 (TrajectoryTracker)
 */
#include <Eigen/Dense>
#include <cmath>
#include <vector>
#include <fstream>
#include <sstream>
#include <string>
#include <algorithm>
#include "../../src/YAstar/yastar.hpp"
#include "../../src/MinimumSnapOsqp/minimumSnap.hpp"

namespace scu_tdt_v1 {

struct HardwareConfig {
    float resolution = 0.05f;
    float origin_x = -7.50f;
    float origin_y = -2.50f;
    float z_floor = -0.470f;
    float obs_min_h = 0.22f;
    float obs_max_h = 1.85f;
    float robot_radius = 0.22f;       // 实机底盘安全半径 (m)
    float max_speed = 1.20f;          // 室内安全最大线速度 (m/s)
    float max_acc = 0.80f;            // 室内安全最大线加速度 (m/s^2)
    float max_omega = 1.50f;          // 最大转向角速度 (rad/s)
    float max_corridor_range = 1.50f; // 室内方形安全走廊最大膨胀范围 (m)
    float lookahead_dist = 0.45f;     // 轨迹跟踪前视距离 (m)
    bool omni_mode = false;           // false: 差速/阿克曼底盘; true: 全向/麦轮/哨兵小陀螺底盘
};

class ScuTdtPlannerEngine {
public:
    ScuTdtPlannerEngine() = default;

    bool LoadPGMMap(const std::string& pgm_path, const HardwareConfig& cfg) {
        cfg_ = cfg;
        std::ifstream ifs(pgm_path, std::ios::binary);
        if (!ifs.is_open()) return false;

        std::string magic;
        ifs >> magic;
        if (magic != "P5") return false;

        // 跳过注释行
        char ch;
        ifs.get(ch);
        while (ifs.peek() == '#') {
            std::string dummy;
            std::getline(ifs, dummy);
        }

        int w = 0, h = 0, max_val = 255;
        ifs >> w >> h >> max_val;
        ifs.get(ch); // 消费换行符

        if (w <= 0 || h <= 0) return false;
        width_ = w;
        height_ = h;

        raw_map_.resize(w * h);
        ifs.read(reinterpret_cast<char*>(raw_map_.data()), w * h);
        if (!ifs) return false;

        // 初始化 YAstar (内部使用局部正坐标 [0, w*res] x [0, h*res] 以彻底规避符号偏置)
        astar_.setMapping(cfg_.resolution);
        astar_.setOriginPos(0.0f, 0.0f);
        astar_.setMap(width_, height_, raw_map_.data());
        astar_.setMaskNumLayers(4);
        // 三层区域代价场 (严格对标 Nav2 禁止区 / 挤压区 / 通行区)：
        // 1. 禁止区 (0 < d <= 0.22m): 极高致命惩罚 (500.0)，严禁底盘中心踏入防撞内切圈
        // 2. 挤压区 (0.22m < d <= 0.50m): 强二次排斥代价，使路径远离墙角与挤压区
        // 3. 通行区 (d > 0.50m): 自由通行，并附加微弱中轴线引导项使轨迹居中行驶
        astar_.setCostField(0.50f, [](float d) {
            if (d <= 0.22f) return 500.0f;
            if (d >= 0.50f) return 0.20f / (d + 0.10f);
            float ratio = (0.50f - d) / (0.50f - 0.22f);
            return 28.0f * ratio * ratio + 0.20f / (d + 0.10f);
        });
        astar_.initCostMap(true);

        // 初始化 MinimumSnap 后端与 SfcSquare 走廊生成器
        SfcSquare::Map sfc_map(height_, width_);
        std::copy(raw_map_.begin(), raw_map_.end(), sfc_map.data());
        min_snap_.setOrder(6);   // 5阶多项式 (6个未知系数)
        min_snap_.setMaxDx(3);   // Minimum Jerk (3阶导平滑，室内电机控制最平稳)
        min_snap_.setDt(0.05f);  // 50ms 采样步长 (20Hz 控制轨迹点)
        min_snap_.setTL(0.20);   // 单次迭代最大耗时限制
        min_snap_.setMap(sfc_map, cfg_.resolution, 0.0f, 0.0f);

        map_loaded_ = true;
        return true;
    }

    // 世界坐标 <-> 算法内部正坐标转换
    Eigen::Vector2f WorldToLocal(const Eigen::Vector2f& w) const {
        return Eigen::Vector2f(w.x() - cfg_.origin_x, w.y() - cfg_.origin_y);
    }

    Eigen::Vector2f LocalToWorld(const Eigen::Vector2f& l) const {
        return Eigen::Vector2f(l.x() + cfg_.origin_x, l.y() + cfg_.origin_y);
    }

    // 清空所有动态避障掩码层，恢复纯净静态地图代价场（重定位或重置时调用）
    void ClearDynamicMask() {
        if (!map_loaded_) return;
        astar_.resetMask();
        astar_.initCostMap(true);
        had_dynamic_pts_ = false;
    }

    // 注入来自 VLP-16 的实时 3D 障碍物点云 (已在 map 坐标系下) 作为动态避障掩码层
    void UpdateDynamicLidarMask(const std::vector<Eigen::Vector3f>& cloud_in_map,
                                double /*mask_duration_sec*/ = 1.0) {
        if (!map_loaded_) return;
        YAstar::TMatrix<u_char> mask(height_, width_);
        mask.fill(255); // 255 = 可通行, 0 = 动态障碍物

        int rad_px = std::max(1, static_cast<int>(std::ceil(cfg_.robot_radius * 0.5f / cfg_.resolution)));
        int dynamic_pts = 0;

        for (const auto& p : cloud_in_map) {
            if (!std::isfinite(p.x()) || !std::isfinite(p.y()) || !std::isfinite(p.z())) continue;
            float h = p.z() - cfg_.z_floor;
            if (h < cfg_.obs_min_h || h > cfg_.obs_max_h) continue;

            Eigen::Vector2f loc = WorldToLocal(Eigen::Vector2f(p.x(), p.y()));
            int gx = static_cast<int>(loc.x() / cfg_.resolution);
            int gy = static_cast<int>(loc.y() / cfg_.resolution);
            if (gx < 2 || gx >= width_ - 2 || gy < 2 || gy >= height_ - 2) continue;

            // 若落点位于已知静态障碍物及其表面 2 栅格 (10cm) 邻域内，属于静态墙面反射，不重复叠加动态掩码
            bool near_static_obs = false;
            for (int sy = -2; sy <= 2 && !near_static_obs; ++sy) {
                for (int sx = -2; sx <= 2; ++sx) {
                    if (raw_map_[(gy + sy) * width_ + (gx + sx)] == 0) {
                        near_static_obs = true;
                        break;
                    }
                }
            }
            if (near_static_obs) continue;

            for (int dy = -rad_px; dy <= rad_px; ++dy) {
                for (int dx = -rad_px; dx <= rad_px; ++dx) {
                    if (dx * dx + dy * dy <= rad_px * rad_px) {
                        int nx = gx + dx, ny = gy + dy;
                        if (nx >= 0 && nx < width_ && ny >= 0 && ny < height_) {
                            mask(ny, nx) = 0;
                            dynamic_pts++;
                        }
                    }
                }
            }
        }

        if (dynamic_pts > 0 || had_dynamic_pts_) {
            astar_.setMaskMap(mask, "vlp16_dynamic", 0);
            astar_.initCostMap(true);
            had_dynamic_pts_ = (dynamic_pts > 0);
        }
    }

    // ==================== SCURM 风格：智能朝向自由空间脱困算法 ====================
    // 当起点/当前位置过于贴近障碍物或卡死时，利用 SDF 符号距离场自动搜索 360° 最开阔方向脱困
    Eigen::Vector2f FindNearestSafePoint(const Eigen::Vector2f& world_pt, float safe_clearance = 0.22f) {
        Eigen::Vector2f loc = WorldToLocal(world_pt);
        auto sdf = astar_.getSDF();
        int gx = std::clamp(static_cast<int>(loc.x() / cfg_.resolution), 0, width_ - 1);
        int gy = std::clamp(static_cast<int>(loc.y() / cfg_.resolution), 0, height_ - 1);

        if (!astar_.pointInObsticle(loc.x(), loc.y()) && sdf(gy, gx) >= safe_clearance) {
            return world_pt;
        }

        // 在周围半径 [0.05m, 1.20m] 内按 36 个方向搜索 SDF 最大（最远离墙壁）的安全自由空间点
        Eigen::Vector2f best_loc = loc;
        float best_score = -1e9f;

        for (float r = 0.10f; r <= 1.20f; r += 0.05f) {
            for (int a = 0; a < 36; ++a) {
                float ang = a * (2.0f * static_cast<float>(M_PI) / 36.0f);
                Eigen::Vector2f cand(loc.x() + r * std::cos(ang), loc.y() + r * std::sin(ang));
                int cx = static_cast<int>(cand.x() / cfg_.resolution);
                int cy = static_cast<int>(cand.y() / cfg_.resolution);
                if (cx < 2 || cx >= width_ - 2 || cy < 2 || cy >= height_ - 2) continue;
                if (astar_.pointInObsticle(cand.x(), cand.y())) continue;

                float clearance = sdf(cy, cx);
                if (clearance >= safe_clearance) {
                    // 兼顾安全裕度与移动距离最短
                    float score = clearance - 0.35f * r;
                    if (score > best_score) {
                        best_score = score;
                        best_loc = cand;
                    }
                }
            }
            if (best_score > 0.0f) break;
        }
        return LocalToWorld(best_loc);
    }

    // 计算 SCURM 智能脱困速度指令 (向 SDF 梯度上升方向平滑撤离)
    Eigen::Vector3f ComputeSmartRecoveryTwist(const Eigen::Vector2f& robot_pos_w, float robot_yaw_rad) {
        Eigen::Vector2f safe_target_w = FindNearestSafePoint(robot_pos_w, cfg_.robot_radius + 0.15f);
        Eigen::Vector2f dir_w = safe_target_w - robot_pos_w;
        float dist = dir_w.norm();
        if (dist < 1e-3f) return Eigen::Vector3f::Zero();
        dir_w /= dist;

        // 转换到车体局部坐标系 (base_link)
        float c = std::cos(-robot_yaw_rad), s = std::sin(-robot_yaw_rad);
        float vx_body = c * dir_w.x() - s * dir_w.y();
        float vy_body = s * dir_w.x() + c * dir_w.y();

        const float escape_speed = 0.35f;
        if (cfg_.omni_mode) {
            return Eigen::Vector3f(escape_speed * vx_body, escape_speed * vy_body, 0.0f);
        } else {
            // 差速底盘：若安全空间在车后方，直接倒车脱困；若在侧方，边转弯边撤离
            float v_lin = (vx_body >= 0.0f) ? escape_speed : -escape_speed;
            float heading_err = std::atan2(vy_body, std::abs(vx_body) + 0.1f);
            float w_ang = std::clamp(1.2f * heading_err, -0.8f, 0.8f);
            return Eigen::Vector3f(v_lin, 0.0f, w_ang);
        }
    }

    // ==================== 完整前后端规划与轨迹优化接口 ====================
    struct PlanResult {
        bool success = false;
        bool used_start_recovery = false;
        Eigen::Vector2f safe_start_w = Eigen::Vector2f::Zero();
        Eigen::Vector2f safe_goal_w = Eigen::Vector2f::Zero();
        std::vector<Eigen::Vector2f> raw_astar_path_w;
        std::vector<Eigen::Vector2f> simplified_waypoints_w;
        std::vector<Eigen::Vector2f> smooth_traj_w;
        std::vector<Eigen::Vector2f> smooth_vel_w;
        std::vector<Eigen::Vector2f> smooth_acc_w;
        std::vector<SfcSquare::PointPair> corridors_w;
        double astar_ms = 0.0;
        double simplify_ms = 0.0;
        double min_snap_ms = 0.0;
        double total_ms = 0.0;
        double path_length_m = 0.0;
        int min_snap_iters = 0;
    };

    PlanResult Plan(const Eigen::Vector2f& start_w,
                    const Eigen::Vector2f& goal_w,
                    const Eigen::Vector2f& init_vel_w = Eigen::Vector2f::Zero()) {
        PlanResult out;
        if (!map_loaded_) return out;

        auto t0 = std::chrono::high_resolution_clock::now();

        // 1. 起点与终点安全检查与 SCURM 自由空间自动脱困修正
        out.safe_start_w = FindNearestSafePoint(start_w, cfg_.robot_radius + 0.08f);
        out.safe_goal_w = FindNearestSafePoint(goal_w, cfg_.robot_radius + 0.08f);
        if ((out.safe_start_w - start_w).norm() > 0.06f) {
            out.used_start_recovery = true;
        }

        Eigen::Vector2f start_loc = WorldToLocal(out.safe_start_w);
        Eigen::Vector2f goal_loc = WorldToLocal(out.safe_goal_w);

        // 2. 前端 YAstar 人工势场搜索
        auto t_a0 = std::chrono::high_resolution_clock::now();
        auto raw_path_loc = astar_.search(start_loc, goal_loc);
        auto t_a1 = std::chrono::high_resolution_clock::now();
        out.astar_ms = std::chrono::duration<double, std::milli>(t_a1 - t_a0).count();

        if (raw_path_loc.size() < 2) {
            return out;
        }

        out.raw_astar_path_w.reserve(raw_path_loc.size());
        for (const auto& p : raw_path_loc) {
            out.raw_astar_path_w.push_back(LocalToWorld(p));
        }

        // 3. Douglas-Peucker 拓扑保形化简 + 长走廊均匀重采样 (抑制五次多项式龙格过冲)
        auto t_s0 = std::chrono::high_resolution_clock::now();
        auto dp_loc = astar_.simplifyPath(raw_path_loc, 0.08f);
        // 确保化简折线不跨越挤压区切内角：保留间距 <= 0.85m 的中轴线控制点
        std::vector<Eigen::Vector2f> simp_loc;
        const float max_seg_len = 0.85f;
        for (size_t i = 0; i + 1 < raw_path_loc.size(); i += 14) {
            simp_loc.push_back(raw_path_loc[i]);
        }
        if ((simp_loc.back() - raw_path_loc.back()).norm() > 0.10f) {
            simp_loc.push_back(raw_path_loc.back());
        }
        auto t_s1 = std::chrono::high_resolution_clock::now();
        out.simplify_ms = std::chrono::duration<double, std::milli>(t_s1 - t_s0).count();

        out.simplified_waypoints_w.reserve(simp_loc.size());
        for (const auto& p : simp_loc) {
            out.simplified_waypoints_w.push_back(LocalToWorld(p));
        }

        // 4. 后端 Minimum Snap + 方形安全飞行走廊 (SFC) 二次规划优化
        auto t_m0 = std::chrono::high_resolution_clock::now();
        MinimumSnap::SolveInput input;
        input.setPath(simp_loc);
        input.setCollisionCheckIter(1);
        input.setMaxSpeed(cfg_.max_speed);
        input.setMaxAcc(cfg_.max_acc);
        input.setMaxCorridorRange(0.22f);
        input.setCorridorShrink(0.0f);
        input.setNormTime(true);
        input.setBackend(MinimumSnap::Backend::OSQPCorridor);
        input.setInitVel(init_vel_w);

        auto ms_res = min_snap_.solve(input);
        auto t_m1 = std::chrono::high_resolution_clock::now();
        out.min_snap_ms = std::chrono::duration<double, std::milli>(t_m1 - t_m0).count();
        out.min_snap_iters = ms_res.iter;

        auto sfc_corridor = min_snap_.getSfc().getCorridor(simp_loc, 0.48f, 0.18f).corridor;

        if (!ms_res.success || ms_res.path.size() < 2) {
            // 回退至闭式求解
            input.setBackend(MinimumSnap::Backend::Close);
            ms_res = min_snap_.solve(input);
        }

        if (ms_res.path.size() >= 2) {
            out.smooth_traj_w.reserve(ms_res.path.size());
            for (const auto& p : ms_res.path) {
                out.smooth_traj_w.push_back(LocalToWorld(p));
            }
            for (const auto& c : sfc_corridor) {
                out.corridors_w.push_back({
                    c[0] + cfg_.origin_x, c[1] + cfg_.origin_y,
                    c[2] + cfg_.origin_x, c[3] + cfg_.origin_y
                });
            }
        } else {
            out.smooth_traj_w = out.raw_astar_path_w;
        }

        // 5. 差分计算平滑轨迹的速度曲线 V(t) 与加速度曲线 A(t) 及总里程
        const float dt = 0.05f;
        const size_t N = out.smooth_traj_w.size();
        out.smooth_vel_w.assign(N, Eigen::Vector2f::Zero());
        out.smooth_acc_w.assign(N, Eigen::Vector2f::Zero());
        out.path_length_m = 0.0;

        for (size_t i = 0; i + 1 < N; ++i) {
            Eigen::Vector2f dp = out.smooth_traj_w[i + 1] - out.smooth_traj_w[i];
            out.path_length_m += dp.norm();
            out.smooth_vel_w[i] = dp / dt;
            // 限幅至物理最大速度
            float spd = out.smooth_vel_w[i].norm();
            if (spd > cfg_.max_speed) {
                out.smooth_vel_w[i] *= (cfg_.max_speed / spd);
            }
        }
        for (size_t i = 1; i + 1 < N; ++i) {
            out.smooth_acc_w[i] = (out.smooth_vel_w[i] - out.smooth_vel_w[i - 1]) / dt;
        }

        auto t1 = std::chrono::high_resolution_clock::now();
        out.total_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        out.success = true;
        return out;
    }

    // ==================== 轨迹前馈+纯跟踪速度控制器 (输出 /cmd_vel) ====================
    Eigen::Vector3f ComputeTrackingTwist(const PlanResult& plan,
                                         const Eigen::Vector2f& robot_pos_w,
                                         float robot_yaw_rad,
                                         float spin_omega_cmd = 0.0f) {
        if (!plan.success || plan.smooth_traj_w.size() < 2) {
            return Eigen::Vector3f::Zero();
        }

        // 检查是否到达终点 (距离 < 0.12m)
        float dist_to_goal = (plan.smooth_traj_w.back() - robot_pos_w).norm();
        if (dist_to_goal < 0.12f) {
            return Eigen::Vector3f::Zero();
        }

        // 1. 找到轨迹上离当前机器人最近的点索引
        size_t closest_idx = 0;
        float min_d = 1e9f;
        for (size_t i = 0; i < plan.smooth_traj_w.size(); ++i) {
            float d = (plan.smooth_traj_w[i] - robot_pos_w).norm();
            if (d < min_d) {
                min_d = d;
                closest_idx = i;
            }
        }

        // 2. 沿轨迹向前搜索前视目标点 (Lookahead Point)
        size_t target_idx = closest_idx;
        float acc_d = 0.0f;
        while (target_idx + 1 < plan.smooth_traj_w.size() && acc_d < cfg_.lookahead_dist) {
            acc_d += (plan.smooth_traj_w[target_idx + 1] - plan.smooth_traj_w[target_idx]).norm();
            target_idx++;
        }

        Eigen::Vector2f target_pt = plan.smooth_traj_w[target_idx];
        Eigen::Vector2f ff_vel = plan.smooth_vel_w[target_idx];
        Eigen::Vector2f pos_err = target_pt - robot_pos_w;

        // 接近终点时平滑减速
        float slow_down = std::clamp(dist_to_goal / 0.80f, 0.20f, 1.0f);
        Eigen::Vector2f des_vel_w = 0.65f * ff_vel * slow_down + 1.15f * pos_err;
        float spd = des_vel_w.norm();
        if (spd > cfg_.max_speed * slow_down) {
            des_vel_w *= (cfg_.max_speed * slow_down / spd);
        }

        // 转换到车体坐标系 (解决云台/小陀螺与底盘的速度解耦，同 SCURM cmd_chassis)
        float c = std::cos(-robot_yaw_rad), s = std::sin(-robot_yaw_rad);
        float vx_body = c * des_vel_w.x() - s * des_vel_w.y();
        float vy_body = s * des_vel_w.x() + c * des_vel_w.y();

        if (cfg_.omni_mode) {
            // 全向底盘 / 小陀螺模式
            return Eigen::Vector3f(vx_body, vy_body, spin_omega_cmd);
        } else {
            // 差速/阿克曼底盘模式
            float target_yaw = std::atan2(pos_err.y(), pos_err.x());
            float yaw_err = std::remainder(target_yaw - robot_yaw_rad, 2.0f * static_cast<float>(M_PI));
            float w_cmd = std::clamp(1.8f * yaw_err, -cfg_.max_omega, cfg_.max_omega);
            float v_cmd = std::clamp(std::hypot(vx_body, vy_body) * std::max(0.0f, std::cos(yaw_err)),
                                     0.0f, cfg_.max_speed);
            return Eigen::Vector3f(v_cmd, 0.0f, w_cmd);
        }
    }

    YAstar& GetAstar() { return astar_; }
    const HardwareConfig& GetConfig() const { return cfg_; }
    int Width() const { return width_; }
    int Height() const { return height_; }

private:
    HardwareConfig cfg_;
    int width_ = 0;
    int height_ = 0;
    bool map_loaded_ = false;
    bool had_dynamic_pts_ = false;
    std::vector<u_char> raw_map_;
    YAstar astar_;
    MinimumSnap min_snap_;
};

} // namespace scu_tdt_v1
