#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "open3d_registration/global_matcher.h"
#include "open3d_conversions/open3d_conversions.h"

class AutoGlobalRelocNode : public rclcpp::Node {
public:
    AutoGlobalRelocNode() : Node("auto_global_reloc_node") {
        // 声明参数
        this->declare_parameter<std::string>("map_path", "/home/wgk/maps/map_1021_1050.pcd");
        this->declare_parameter<std::string>("scan_topic", "/velodyne_points");
        this->declare_parameter<std::string>("initialpose_topic", "/initialpose");
        this->declare_parameter<bool>("auto_exit", true);
        this->declare_parameter<bool>("publish_map", true);

        map_path_ = this->get_parameter("map_path").as_string();
        scan_topic_ = this->get_parameter("scan_topic").as_string();
        initialpose_topic_ = this->get_parameter("initialpose_topic").as_string();
        auto_exit_ = this->get_parameter("auto_exit").as_bool();
        publish_map_ = this->get_parameter("publish_map").as_bool();

        RCLCPP_INFO(this->get_logger(), "========================================================");
        RCLCPP_INFO(this->get_logger(), "  【开机无初值自定位节点 auto_global_reloc 启动】");
        RCLCPP_INFO(this->get_logger(), "  先验地图: %s", map_path_.c_str());
        RCLCPP_INFO(this->get_logger(), "  雷达话题: %s", scan_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "  初值话题: %s", initialpose_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "  ★ 状态: 等待第一帧点云到达即可全自动完成全局定位！");
        RCLCPP_INFO(this->get_logger(), "========================================================");

        // 初始化匹配器并加载地图
        if (!matcher_.LoadMap(map_path_)) {
            RCLCPP_ERROR(this->get_logger(), "❌ 无法加载先验地图: %s", map_path_.c_str());
            return;
        }

        // 创建 TF 广播器
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // 创建 /initialpose 发布者
        pub_initialpose_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            initialpose_topic_, rclcpp::QoS(10).reliable());

        // 创建 RViz 可视化发布者: /map 与 /cloud_registered_map
        if (publish_map_) {
            pub_map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                "/map", rclcpp::QoS(1).transient_local().reliable());
            pub_aligned_scan_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                "/cloud_registered_map", rclcpp::QoS(10).reliable());

            // 广播先验地图至 RViz
            PublishMapCloud();
        }

        // 订阅雷达话题 (同时兼容 /velodyne_points 和 /cloud_registered)
        sub_scan_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            scan_topic_, rclcpp::SensorDataQoS(),
            std::bind(&AutoGlobalRelocNode::CallbackScan, this, std::placeholders::_1));

        if (scan_topic_ != "/cloud_registered") {
            sub_cloud_reg_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "/cloud_registered", rclcpp::SensorDataQoS(),
                std::bind(&AutoGlobalRelocNode::CallbackScan, this, std::placeholders::_1));
        }
    }

private:
    void PublishMapCloud() {
        auto map_cloud = std::make_shared<open3d::geometry::PointCloud>();
        if (open3d::io::ReadPointCloud(map_path_, *map_cloud) && !map_cloud->IsEmpty()) {
            auto down = map_cloud->VoxelDownSample(0.08);
            sensor_msgs::msg::PointCloud2 map_msg;
            open3d_conversions::open3dToRos(*down, map_msg);
            map_msg.header.stamp = this->now();
            map_msg.header.frame_id = "map";
            pub_map_->publish(map_msg);
            RCLCPP_INFO(this->get_logger(), "🗺️ 先验点云地图已广播至 RViz 话题: /map (%lu 点)", down->points_.size());
        }
    }

    void CallbackScan(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        if (reloc_completed_) return;

        // 转换 ROS PointCloud2 -> Open3D PointCloud
        auto o3d_scan = std::make_shared<open3d::geometry::PointCloud>();
        open3d_conversions::rosToOpen3d(msg, *o3d_scan);

        if (o3d_scan->points_.size() < 100) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                                "雷达点云点数过少 (%lu 点)，跳过...", o3d_scan->points_.size());
            return;
        }

        RCLCPP_INFO(this->get_logger(), "📡 接收到开机雷达帧 (%lu 点)，正在进行全图无先验特征匹配...", 
                    o3d_scan->points_.size());

        // 执行无初值全局匹配
        auto res = matcher_.Relocalize(o3d_scan);

        if (!res.success) {
            RCLCPP_WARN(this->get_logger(), "⚠️ 本帧置信度较低 (Fitness=%.1f%%, RMSE=%.2f cm)，等待下一帧重试...", 
                        res.fitness * 100.0, res.rmse * 100.0);
            return;
        }

        reloc_completed_ = true;

        // 构造四元数
        Eigen::Matrix3d R = res.transformation.block<3, 3>(0, 0);
        Eigen::Quaterniond q(R);

        // 构造 PoseWithCovarianceStamped
        geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
        pose_msg.header.stamp = this->now();
        pose_msg.header.frame_id = "map";

        pose_msg.pose.pose.position.x = res.x;
        pose_msg.pose.pose.position.y = res.y;
        pose_msg.pose.pose.position.z = res.z;
        pose_msg.pose.pose.orientation.x = q.x();
        pose_msg.pose.pose.orientation.y = q.y();
        pose_msg.pose.pose.orientation.z = q.z();
        pose_msg.pose.pose.orientation.w = q.w();

        // 协方差赋小值表示高置信度
        for (int i = 0; i < 36; ++i) pose_msg.pose.covariance[i] = 0.0;
        pose_msg.pose.covariance[0] = 0.01;  // x
        pose_msg.pose.covariance[7] = 0.01;  // y
        pose_msg.pose.covariance[14] = 0.01; // z
        pose_msg.pose.covariance[35] = 0.02; // yaw

        // 连续发送 3 次以防 QoS 丢包
        for (int i = 0; i < 3; ++i) {
            pub_initialpose_->publish(pose_msg);
            rclcpp::sleep_for(std::chrono::milliseconds(50));
        }

        // 广播配准对齐后的点云至 RViz (/cloud_registered_map)
        if (pub_aligned_scan_) {
            auto aligned_scan = std::make_shared<open3d::geometry::PointCloud>(*o3d_scan);
            aligned_scan->Transform(res.transformation);
            sensor_msgs::msg::PointCloud2 aligned_msg;
            open3d_conversions::open3dToRos(*aligned_scan, aligned_msg);
            aligned_msg.header.stamp = this->now();
            aligned_msg.header.frame_id = "map";
            pub_aligned_scan_->publish(aligned_msg);
        }

        // 广播 TF: map -> base_link
        latest_tf_.header.stamp = this->now();
        latest_tf_.header.frame_id = "map";
        latest_tf_.child_frame_id = "base_link";
        latest_tf_.transform.translation.x = res.x;
        latest_tf_.transform.translation.y = res.y;
        latest_tf_.transform.translation.z = res.z;
        latest_tf_.transform.rotation.x = q.x();
        latest_tf_.transform.rotation.y = q.y();
        latest_tf_.transform.rotation.z = q.z();
        latest_tf_.transform.rotation.w = q.w();
        tf_broadcaster_->sendTransform(latest_tf_);

        // 终端打印显眼的大字报绿框输出
        std::cout << "\n\033[1;32m====================================================================\033[0m\n";
        std::cout << "\033[1;32m  🎉 【auto_global_reloc】开机无先验全局自定位成功！\033[0m\n";
        std::cout << "  · 锁定全局坐标 : \033[1;36mX = " << res.x << " m, Y = " << res.y 
                  << " m, Z = " << res.z << " m\033[0m\n";
        std::cout << "  · 锁定航向朝向 : \033[1;36mYaw = " << res.yaw << "°\033[0m\n";
        std::cout << "  · 点云咬合匹配 : \033[1;32m" << res.fitness * 100.0 << " %\033[0m (Fitness)\n";
        std::cout << "  · 空间残差均方 : \033[1;32m" << res.rmse * 100.0 << " cm\033[0m (RMSE)\n";
        std::cout << "  · 粗匹配耗时   : " << res.time_coarse_ms << " ms\n";
        std::cout << "  · ICP精修耗时  : " << res.time_fine_ms << " ms\n";
        std::cout << "  · 在线总计算量 : \033[1;33m" << res.time_total_ms << " ms\033[0m\n";
        std::cout << "  ★ RViz 可视化已同步推送到: \033[1;35m/initialpose, /map, /cloud_registered_map\033[0m\n";
        std::cout << "\033[1;32m====================================================================\033[0m\n\n";

        if (auto_exit_) {
            RCLCPP_INFO(this->get_logger(), "✅ 自定位完成，使命结束，节点将在 2 秒后自动退出并交接给定位系统...");
            timer_exit_ = this->create_wall_timer(
                std::chrono::seconds(2), [this]() {
                    rclcpp::shutdown();
                });
        } else {
            // 若开启常驻模式（如专门给 RViz 观察），启动 10Hz 定时器持续广播 TF 和点云
            RCLCPP_INFO(this->get_logger(), "👀 RViz 观察模式保持运行中 (持续广播 TF 与重定位对齐点云)... 按 Ctrl+C 退出");
            timer_tf_ = this->create_wall_timer(
                std::chrono::milliseconds(100), [this]() {
                    latest_tf_.header.stamp = this->now();
                    tf_broadcaster_->sendTransform(latest_tf_);
                });
        }
    }

    std::string map_path_;
    std::string scan_topic_;
    std::string initialpose_topic_;
    bool auto_exit_ = true;
    bool publish_map_ = true;
    bool reloc_completed_ = false;

    geometry_msgs::msg::TransformStamped latest_tf_;
    open3d_loc::GlobalMatcher matcher_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_initialpose_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_map_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_aligned_scan_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_scan_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_reg_;
    rclcpp::TimerBase::SharedPtr timer_exit_;
    rclcpp::TimerBase::SharedPtr timer_tf_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AutoGlobalRelocNode>();
    rclcpp::spin(node);
    return 0;
}
