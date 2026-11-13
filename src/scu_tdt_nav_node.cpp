/**
 * @file scu_tdt_nav_node.cpp
 * @brief SCU-TDT-V1 ROS 2 实时决策规划与轨迹跟踪节点
 *        同时支持：
 *          1. Gazebo 11 RMUC 2025 3D 赛场仿真 (VLP-16 + HI12 + 全向/差速底盘)
 *          2. 真实实验室 (VLP-16 + Hipnuc HI12 + Fast-LIO + GlobalMatcher V2)
 *
 * 实时四步点云处理与膨胀发布：
 *   Step 1: 地面分割滤除 (z - z_floor < 0.15m) 与天花板滤除
 *   Step 2: 车体自身点云滤除 (r < 0.38m) -> 发布 /points_no_ground_and_self
 *   Step 3: 拍平投影至 2D 栅格并融合静态地图 -> 发布 /scu_tdt/occupancy_map
 *   Step 4: 生成 Nav2 标准三层膨胀代价图 -> 发布 /scu_tdt/inflated_costmap
 *           (100=障碍物黑, 99=内切禁止区深蓝/洋红, 1..98=挤压区青蓝/粉红, 0=自由通行区白)
 *           同时发布方形安全飞行走廊 /scu_tdt/sfc_corridors (MarkerArray)
 */
#include "../include/scu_tdt_v1/scu_recovery_tracker.hpp"
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

using namespace scu_tdt_v1;

class ScuTdtNavNode : public rclcpp::Node {
public:
    ScuTdtNavNode() : Node("scu_tdt_nav_node") {
        this->declare_parameter<std::string>("pgm_map_path",
            "/home/wgk/decision_planning/SCU-TDT-V1/maps/rmuc2025.pgm");
        this->declare_parameter<double>("origin_x", 0.0);
        this->declare_parameter<double>("origin_y", 0.0);
        this->declare_parameter<double>("z_floor", 0.0);
        this->declare_parameter<double>("init_x", 2.50);
        this->declare_parameter<double>("init_y", 2.00);
        this->declare_parameter<double>("init_yaw", 0.0);
        this->declare_parameter<double>("auto_goal_x", 25.20);
        this->declare_parameter<double>("auto_goal_y", 13.00);
        this->declare_parameter<bool>("enable_auto_goal", true);
        this->declare_parameter<double>("max_speed", 2.20);
        this->declare_parameter<double>("max_acc", 1.50);
        this->declare_parameter<double>("robot_radius", 0.25);
        this->declare_parameter<double>("squeeze_radius", 0.55);
        this->declare_parameter<bool>("omni_mode", true);
        this->declare_parameter<std::string>("odom_topic", "/odom");
        this->declare_parameter<std::string>("cloud_topic", "/velodyne_points");

        std::string pgm_path = this->get_parameter("pgm_map_path").as_string();
        cfg_.origin_x = static_cast<float>(this->get_parameter("origin_x").as_double());
        cfg_.origin_y = static_cast<float>(this->get_parameter("origin_y").as_double());
        cfg_.z_floor = static_cast<float>(this->get_parameter("z_floor").as_double());
        cfg_.max_speed = static_cast<float>(this->get_parameter("max_speed").as_double());
        cfg_.max_acc = static_cast<float>(this->get_parameter("max_acc").as_double());
        cfg_.robot_radius = static_cast<float>(this->get_parameter("robot_radius").as_double());
        squeeze_radius_ = static_cast<float>(this->get_parameter("squeeze_radius").as_double());
        cfg_.omni_mode = this->get_parameter("omni_mode").as_bool();

        robot_x_ = static_cast<float>(this->get_parameter("init_x").as_double());
        robot_y_ = static_cast<float>(this->get_parameter("init_y").as_double());
        robot_yaw_ = static_cast<float>(this->get_parameter("init_yaw").as_double());
        reloc_offset_x_ = 0.0f;
        reloc_offset_y_ = 0.0f;
        reloc_offset_yaw_ = 0.0f;

        if (!engine_.LoadPGMMap(pgm_path, cfg_)) {
            RCLCPP_ERROR(this->get_logger(), "无法加载 2D 导航栅格图: %s", pgm_path.c_str());
            return;
        }
        RCLCPP_INFO(this->get_logger(),
                    "✅ [SCU-TDT-V1] 导航规划引擎已就绪 (地图: %dx%d, origin=[%.2f, %.2f], v_max=%.2f m/s)",
                    engine_.Width(), engine_.Height(), cfg_.origin_x, cfg_.origin_y, cfg_.max_speed);

        // 发布器
        auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
        pub_map_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/scu_tdt/occupancy_map", map_qos);
        pub_costmap_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/scu_tdt/inflated_costmap", map_qos);
        pub_filtered_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/points_no_ground_and_self", 10);
        pub_astar_ = this->create_publisher<nav_msgs::msg::Path>("/scu_tdt/global_path", 10);
        pub_smooth_ = this->create_publisher<nav_msgs::msg::Path>("/scu_tdt/smooth_trajectory", 10);
        pub_corridors_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/scu_tdt/sfc_corridors", 10);
        pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        PublishOccupancyAndCostmap();

        std::string odom_topic = this->get_parameter("odom_topic").as_string();
        std::string cloud_topic = this->get_parameter("cloud_topic").as_string();

        // 订阅器
        sub_reloc_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10,
            std::bind(&ScuTdtNavNode::OnInitialPose, this, std::placeholders::_1));

        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, 20,
            std::bind(&ScuTdtNavNode::OnOdometry, this, std::placeholders::_1));

        sub_goal_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10,
            std::bind(&ScuTdtNavNode::OnGoalPose, this, std::placeholders::_1));

        sub_scan_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            cloud_topic, rclcpp::SensorDataQoS(),
            std::bind(&ScuTdtNavNode::OnLidarCloud, this, std::placeholders::_1));

        // 20Hz 轨迹跟踪控制定时器
        timer_ctrl_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&ScuTdtNavNode::ControlLoop, this));

        // 1Hz 周期重发三层膨胀代价图与当前轨迹，确保 RViz2 随时打开均立即可见
        timer_vis_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            [this]() {
                PublishOccupancyAndCostmap();
                if (current_plan_.success) {
                    PublishPaths();
                }
            });

        // 启动后若开启自动演示目标点，自动规划一条跨赛场完整示范轨迹
        if (this->get_parameter("enable_auto_goal").as_bool()) {
            goal_w_ = Eigen::Vector2f(
                static_cast<float>(this->get_parameter("auto_goal_x").as_double()),
                static_cast<float>(this->get_parameter("auto_goal_y").as_double()));
            has_goal_ = true;
            current_plan_ = engine_.Plan(Eigen::Vector2f(robot_x_, robot_y_), goal_w_);
            if (current_plan_.success) {
                RCLCPP_INFO(this->get_logger(),
                            "🚀 自动演示规划就绪: (%.2f, %.2f) -> (%.2f, %.2f) | 耗时=%.2f ms",
                            robot_x_, robot_y_, goal_w_.x(), goal_w_.y(), current_plan_.total_ms);
                PublishPaths();
            }
        }
    }

private:
    static float QuatToYaw(double x, double y, double z, double w) {
        return static_cast<float>(std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)));
    }

    void OnInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        const auto& p = msg->pose.pose.position;
        const auto& q = msg->pose.pose.orientation;
        float target_x = static_cast<float>(p.x);
        float target_y = static_cast<float>(p.y);
        float target_yaw = QuatToYaw(q.x, q.y, q.z, q.w);

        // 1. 立即停止当前跟踪运动并清空旧轨迹与旧动态避障掩码
        has_goal_ = false;
        current_plan_ = ScuTdtPlannerEngine::PlanResult{};
        geometry_msgs::msg::Twist stop_cmd;
        pub_cmd_vel_->publish(stop_cmd);
        engine_.ClearDynamicMask();
        PublishOccupancyAndCostmap();
        PublishPaths();

        // 2. 区分 Gazebo 仿真模式与实机模式：
        //    - Gazebo 仿真下 (use_sim_time=true)：直接通过 gz model 将物理机器人瞬移至重定位点，
        //      使 Gazebo 实体、VLP-16 激光扫描与 RViz2 TF 保持 100% 物理同位，偏移量归零；
        //    - 实机模式下 (use_sim_time=false)：按严格 SE(2) 刚体变换求取 map<-odom 旋转平移偏置。
        bool is_sim = false;
        this->get_parameter("use_sim_time", is_sim);
        if (is_sim) {
            reloc_offset_x_ = 0.0f;
            reloc_offset_y_ = 0.0f;
            reloc_offset_yaw_ = 0.0f;
            odom_x_ = target_x;
            odom_y_ = target_y;
            odom_yaw_ = target_yaw;
            // 暂停激光动态掩码注入 0.4s，等待 Gazebo 完成实体瞬移
            reloc_time_ = std::chrono::steady_clock::now();

            char cmd[512];
            std::snprintf(cmd, sizeof(cmd),
                          "(gz model -m unitree_go2_vlp16 -x %.3f -y %.3f -z 0.05 -R 0 -P 0 -Y %.4f >/dev/null 2>&1 || "
                          "gz model -m scu_sentry_vlp16 -x %.3f -y %.3f -z 0.05 -R 0 -P 0 -Y %.4f >/dev/null 2>&1) &",
                          target_x, target_y, target_yaw, target_x, target_y, target_yaw);
            int ret = std::system(cmd);
            (void)ret;
        } else {
            reloc_offset_yaw_ = target_yaw - odom_yaw_;
            float c = std::cos(reloc_offset_yaw_), s = std::sin(reloc_offset_yaw_);
            reloc_offset_x_ = target_x - (c * odom_x_ - s * odom_y_);
            reloc_offset_y_ = target_y - (s * odom_x_ + c * odom_y_);
        }

        robot_x_ = target_x;
        robot_y_ = target_y;
        robot_yaw_ = target_yaw;
        has_pose_ = true;
        RCLCPP_INFO(this->get_logger(), "📍 重定位完成: (%.3f, %.3f, Yaw=%.1f°)%s",
                    robot_x_, robot_y_, robot_yaw_ * 180.0f / static_cast<float>(M_PI),
                    is_sim ? " [已同步瞬移 Gazebo 机器人]" : " [已更新 SE(2) 里程计偏置]");
    }

    void OnOdometry(const nav_msgs::msg::Odometry::SharedPtr msg) {
        const auto& p = msg->pose.pose.position;
        const auto& q = msg->pose.pose.orientation;
        odom_x_ = static_cast<float>(p.x);
        odom_y_ = static_cast<float>(p.y);
        odom_yaw_ = QuatToYaw(q.x, q.y, q.z, q.w);

        float c = std::cos(reloc_offset_yaw_), s = std::sin(reloc_offset_yaw_);
        robot_x_ = c * odom_x_ - s * odom_y_ + reloc_offset_x_;
        robot_y_ = s * odom_x_ + c * odom_y_ + reloc_offset_y_;
        robot_yaw_ = odom_yaw_ + reloc_offset_yaw_;
        robot_vx_ = static_cast<float>(msg->twist.twist.linear.x);
        robot_vy_ = static_cast<float>(msg->twist.twist.linear.y);
        has_pose_ = true;
    }

    void OnLidarCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        if (!has_pose_) return;
        // 重定位瞬移过渡期内 (0.45s) 跳过点云投影，防止瞬移前的旧视角点云污染地图
        double since_reloc = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - reloc_time_).count();
        if (since_reloc < 0.45) return;

        std::vector<Eigen::Vector3f> pts_in_map;
        pts_in_map.reserve(2000);

        float c = std::cos(robot_yaw_), s = std::sin(robot_yaw_);
        sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> it_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> it_z(*msg, "z");

        for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z) {
            float lx = *it_x, ly = *it_y, lz = *it_z;
            if (!std::isfinite(lx) || !std::isfinite(ly) || !std::isfinite(lz)) continue;

            // Step 2: 去除车体自身点云 (r < 0.38m) 与超远杂波 (r > 12.0m)
            float r2 = lx * lx + ly * ly;
            if (r2 < 0.38f * 0.38f || r2 > 12.0f * 12.0f) continue;

            // 转换到世界系 (velodyne 安装高度约 +0.42m)
            float mx = c * lx - s * ly + robot_x_;
            float my = s * lx + c * ly + robot_y_;
            float mz = lz + 0.42f;

            // Step 1: 地面分割去除与天花板过滤
            if (mz - cfg_.z_floor < 0.15f || mz - cfg_.z_floor > 1.85f) continue;

            pts_in_map.emplace_back(mx, my, mz);
        }

        // 发布去除地面与车体自身后的纯净障碍物点云 /points_no_ground_and_self
        if (!pts_in_map.empty()) {
            sensor_msgs::msg::PointCloud2 out_cloud;
            out_cloud.header.stamp = msg->header.stamp;
            out_cloud.header.frame_id = "map";
            out_cloud.height = 1;
            out_cloud.width = static_cast<uint32_t>(pts_in_map.size());
            sensor_msgs::PointCloud2Modifier mod(out_cloud);
            mod.setPointCloud2FieldsByString(1, "xyz");
            mod.resize(pts_in_map.size());

            sensor_msgs::PointCloud2Iterator<float> ox(out_cloud, "x");
            sensor_msgs::PointCloud2Iterator<float> oy(out_cloud, "y");
            sensor_msgs::PointCloud2Iterator<float> oz(out_cloud, "z");
            for (const auto& pt : pts_in_map) {
                *ox = pt.x(); *oy = pt.y(); *oz = pt.z();
                ++ox; ++oy; ++oz;
            }
            pub_filtered_cloud_->publish(out_cloud);
        }

        engine_.UpdateDynamicLidarMask(pts_in_map, 1.0);
    }

    void OnGoalPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        goal_w_ = Eigen::Vector2f(static_cast<float>(msg->pose.position.x),
                                  static_cast<float>(msg->pose.position.y));
        has_goal_ = true;
        Eigen::Vector2f start_w(robot_x_, robot_y_);
        Eigen::Vector2f init_vel(robot_vx_, robot_vy_);

        current_plan_ = engine_.Plan(start_w, goal_w_, init_vel);
        if (current_plan_.success) {
            RCLCPP_INFO(this->get_logger(),
                        "🎯 规划成功: (%.2f, %.2f) -> (%.2f, %.2f) | 总耗时=%.2f ms (A*=%.2fms, MinSnap=%.2fms)",
                        start_w.x(), start_w.y(), goal_w_.x(), goal_w_.y(),
                        current_plan_.total_ms, current_plan_.astar_ms, current_plan_.min_snap_ms);
            PublishPaths();
        } else {
            RCLCPP_WARN(this->get_logger(), "⚠️ 目标点不可达，启动 SCURM 智能向自由空间脱困...");
        }
    }

    void ControlLoop() {
        if (!has_goal_) return;
        Eigen::Vector2f pos_w(robot_x_, robot_y_);
        Eigen::Vector3f cmd = Eigen::Vector3f::Zero();

        if (current_plan_.success) {
            cmd = engine_.ComputeTrackingTwist(current_plan_, pos_w, robot_yaw_);
        } else {
            cmd = engine_.ComputeSmartRecoveryTwist(pos_w, robot_yaw_);
        }

        geometry_msgs::msg::Twist twist;
        twist.linear.x = cmd.x();
        twist.linear.y = cmd.y();
        twist.angular.z = cmd.z();
        pub_cmd_vel_->publish(twist);
    }

    void PublishOccupancyAndCostmap() {
        auto now = this->now();
        int W = engine_.Width();
        int H = engine_.Height();

        // 1. 纯净二值障碍物底图 (/scu_tdt/occupancy_map)
        nav_msgs::msg::OccupancyGrid grid_msg;
        grid_msg.header.stamp = now;
        grid_msg.header.frame_id = "map";
        grid_msg.info.resolution = cfg_.resolution;
        grid_msg.info.width = W;
        grid_msg.info.height = H;
        grid_msg.info.origin.position.x = cfg_.origin_x;
        grid_msg.info.origin.position.y = cfg_.origin_y;
        grid_msg.info.origin.orientation.w = 1.0;

        auto map_mat = engine_.GetAstar().getMap();
        auto sdf_mat = engine_.GetAstar().getSDF();
        grid_msg.data.resize(W * H);

        // 2. Nav2 标准三层膨胀区域图 (/scu_tdt/inflated_costmap)
        // 在 RViz2 Map Display 中选择 Color Scheme = costmap 时：
        //   - 100    : 致命障碍物本体 (黑色)
        //   - 99     : 内切禁止区 Forbidden Zone (深蓝 + 洋红/红色内切边界)
        //   - 1..98  : 缓冲挤压区 Squeeze Zone (青蓝色渐变 + 粉红边缘)
        //   - 0      : 自由通行区 Free Zone (白色/透明)
        nav_msgs::msg::OccupancyGrid cost_msg = grid_msg;
        cost_msg.data.resize(W * H);

        for (int i = 0; i < W * H; ++i) {
            bool is_obs = (map_mat.data()[i] == 0);
            grid_msg.data[i] = is_obs ? 100 : 0;

            float d = sdf_mat.data()[i];
            if (is_obs || d <= 1e-4f) {
                cost_msg.data[i] = 100;
            } else if (d <= cfg_.robot_radius) {
                cost_msg.data[i] = 99;
            } else if (d <= squeeze_radius_) {
                float ratio = (squeeze_radius_ - d) / std::max(0.05f, squeeze_radius_ - cfg_.robot_radius);
                int cval = std::clamp(static_cast<int>(1.0f + 97.0f * ratio), 1, 98);
                cost_msg.data[i] = static_cast<int8_t>(cval);
            } else {
                cost_msg.data[i] = 0;
            }
        }
        pub_map_->publish(grid_msg);
        pub_costmap_->publish(cost_msg);
    }

    void PublishPaths() {
        auto now = this->now();
        nav_msgs::msg::Path astar_msg;
        astar_msg.header.stamp = now;
        astar_msg.header.frame_id = "map";
        for (const auto& p : current_plan_.simplified_waypoints_w) {
            geometry_msgs::msg::PoseStamped ps;
            ps.header = astar_msg.header;
            ps.pose.position.x = p.x();
            ps.pose.position.y = p.y();
            ps.pose.position.z = 0.05;
            ps.pose.orientation.w = 1.0;
            astar_msg.poses.push_back(ps);
        }
        pub_astar_->publish(astar_msg);

        nav_msgs::msg::Path smooth_msg;
        smooth_msg.header.stamp = now;
        smooth_msg.header.frame_id = "map";
        for (const auto& p : current_plan_.smooth_traj_w) {
            geometry_msgs::msg::PoseStamped ps;
            ps.header = smooth_msg.header;
            ps.pose.position.x = p.x();
            ps.pose.position.y = p.y();
            ps.pose.position.z = 0.08;
            ps.pose.orientation.w = 1.0;
            smooth_msg.poses.push_back(ps);
        }
        pub_smooth_->publish(smooth_msg);

        // 发布蓝色方形安全飞行走廊 (Square SFC) 到 RViz2
        visualization_msgs::msg::MarkerArray ma;
        visualization_msgs::msg::Marker clear_m;
        clear_m.action = visualization_msgs::msg::Marker::DELETEALL;
        ma.markers.push_back(clear_m);

        for (size_t i = 0; i < current_plan_.corridors_w.size(); ++i) {
            const auto& c = current_plan_.corridors_w[i];
            float x0 = std::min(c[0], c[2]), x1 = std::max(c[0], c[2]);
            float y0 = std::min(c[1], c[3]), y1 = std::max(c[1], c[3]);

            visualization_msgs::msg::Marker box;
            box.header.stamp = now;
            box.header.frame_id = "map";
            box.ns = "sfc_boxes";
            box.id = static_cast<int>(i);
            box.type = visualization_msgs::msg::Marker::CUBE;
            box.action = visualization_msgs::msg::Marker::ADD;
            box.pose.position.x = 0.5f * (x0 + x1);
            box.pose.position.y = 0.5f * (y0 + y1);
            box.pose.position.z = 0.03f;
            box.pose.orientation.w = 1.0;
            box.scale.x = std::max(0.05f, x1 - x0);
            box.scale.y = std::max(0.05f, y1 - y0);
            box.scale.z = 0.02f;
            box.color.r = 0.15f;
            box.color.g = 0.55f;
            box.color.b = 0.95f;
            box.color.a = 0.28f;
            ma.markers.push_back(box);
        }
        pub_corridors_->publish(ma);
    }

    HardwareConfig cfg_;
    float squeeze_radius_ = 0.55f;
    ScuTdtPlannerEngine engine_;
    ScuTdtPlannerEngine::PlanResult current_plan_;

    float odom_x_ = 0.0f, odom_y_ = 0.0f, odom_yaw_ = 0.0f;
    float reloc_offset_x_ = 0.0f, reloc_offset_y_ = 0.0f, reloc_offset_yaw_ = 0.0f;
    float robot_x_ = 2.50f, robot_y_ = 2.00f, robot_yaw_ = 0.0f;
    float robot_vx_ = 0.0f, robot_vy_ = 0.0f;
    bool has_pose_ = true;
    bool has_goal_ = false;
    Eigen::Vector2f goal_w_ = Eigen::Vector2f::Zero();
    std::chrono::steady_clock::time_point reloc_time_{};

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_map_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_costmap_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_filtered_cloud_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_astar_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_smooth_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_corridors_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_reloc_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_scan_;
    rclcpp::TimerBase::SharedPtr timer_ctrl_;
    rclcpp::TimerBase::SharedPtr timer_vis_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScuTdtNavNode>());
    rclcpp::shutdown();
    return 0;
}

