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
        // 声明参数 (动态适配不同用户根目录)
        const char* home = std::getenv("HOME");
        home_dir_ = home ? std::string(home) : "";
        std::string default_map = home_dir_ + "/maps/map_1021_1050.pcd";
        this->declare_parameter<std::string>("map_path", default_map);
        this->declare_parameter<std::string>("scan_topic", "/velodyne_points");
        this->declare_parameter<std::string>("initialpose_topic", "/initialpose");
        this->declare_parameter<bool>("auto_exit", true);
        this->declare_parameter<bool>("publish_map", true);
        this->declare_parameter<int>("accumulate_frames", 3);

        map_path_ = this->get_parameter("map_path").as_string();
        scan_topic_ = this->get_parameter("scan_topic").as_string();
        initialpose_topic_ = this->get_parameter("initialpose_topic").as_string();
        auto_exit_ = this->get_parameter("auto_exit").as_bool();
        publish_map_ = this->get_parameter("publish_map").as_bool();
        accumulate_frames_ = std::max(1, static_cast<int>(this->get_parameter("accumulate_frames").as_int()));

        accumulated_scan_ = std::make_shared<open3d::geometry::PointCloud>();

        RCLCPP_INFO(this->get_logger(), "========================================================");
        RCLCPP_INFO(this->get_logger(), "  【开机无初值自定位节点 auto_global_reloc V2 启动】");
        RCLCPP_INFO(this->get_logger(), "  先验地图: %s", map_path_.c_str());
        RCLCPP_INFO(this->get_logger(), "  雷达话题: %s (叠加 %d 帧增强墙面轮廓)", scan_topic_.c_str(), accumulate_frames_);
        RCLCPP_INFO(this->get_logger(), "  初值话题: %s", initialpose_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "  ★ 状态: 等待点云到达，执行地面解耦+纯墙面多假设全局定位！");
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

        // 创建 RViz 可视化点云发布者 (Transient Local 让后打开的 RViz 也能收到地图)
        auto latch_qos = rclcpp::QoS(1).transient_local().reliable();
        pub_map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/map", latch_qos);
        pub_aligned_scan_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered_map", latch_qos);

        // 发布先验地图至 RViz
        if (publish_map_) {
            PublishPriorMap();
        }

        // 订阅实时激光雷达点云 (兼容 SensorDataQoS 与 Reliable)
        sub_scan_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            scan_topic_, rclcpp::SensorDataQoS(),
            std::bind(&AutoGlobalRelocNode::CallbackScan, this, std::placeholders::_1));
    }

private:
    void PublishPriorMap() {
        auto map_down = matcher_.GetMapDownCloud();
        if (map_down && !map_down->IsEmpty()) {
            sensor_msgs::msg::PointCloud2 map_msg;
            open3d_conversions::open3dToRos(*map_down, map_msg);
            map_msg.header.stamp = this->now();
            map_msg.header.frame_id = "map";
            pub_map_->publish(map_msg);
            RCLCPP_INFO(this->get_logger(), "🗺️ 先验点云地图已广播至 RViz 话题: /map (%lu 点)", map_down->points_.size());
        }
    }

    void CallbackScan(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        if (reloc_completed_) return;

        // 转换 ROS PointCloud2 -> Open3D PointCloud
        auto o3d_scan = std::make_shared<open3d::geometry::PointCloud>();
        open3d_conversions::rosToOpen3d(msg, *o3d_scan);

        if (o3d_scan->points_.size() < 200) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "雷达点云点数过少 (%lu 点)，跳过...", o3d_scan->points_.size());
            return;
        }

        *accumulated_scan_ += *o3d_scan;
        frame_count_++;

        if (frame_count_ < accumulate_frames_) {
            RCLCPP_INFO(this->get_logger(), "📥 正在累积静态开机雷达帧 (%d/%d, 当前共 %lu 点)...",
                        frame_count_, accumulate_frames_, accumulated_scan_->points_.size());
            return;
        }

        RCLCPP_INFO(this->get_logger(), "📡 完成 %d 帧点云累积 (%lu 点)，正在进行地面解耦与纯墙面多假设全局匹配...",
                    frame_count_, accumulated_scan_->points_.size());

        // 执行 V2 无初值全局匹配
        auto res = matcher_.Relocalize(accumulated_scan_);

        if (!res.success) {
            RCLCPP_WARN(this->get_logger(),
                        "⚠️ 本轮纯墙面重合度不足 (Wall Fitness=%.1f%%, Wall RMSE=%.2f cm)，清空缓存重新采样...",
                        res.wall_fitness * 100.0, res.wall_rmse * 100.0);
            accumulated_scan_->Clear();
            frame_count_ = 0;
            return;
        }

        reloc_completed_ = true;

        // 保存当次实机点云快照并自动渲染最新可视化图片 (供 fetch_pi_maps.sh 拉取)
        std::string latest_scan_path = home_dir_ + "/maps/latest_real_scan.pcd";
        open3d::io::WritePointCloud(latest_scan_path, *accumulated_scan_);
        std::string out_png = home_dir_ + "/maps/reloc_process_result.png";
        matcher_.SaveEvalAndPlot(accumulated_scan_, res, out_png, false);

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
            auto aligned_scan = std::make_shared<open3d::geometry::PointCloud>(*accumulated_scan_);
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
        std::cout << "\033[1;32m  🎉 【auto_global_reloc V2】开机无先验全局自定位成功！\033[0m\n";
        std::cout << "  · 锁定全局坐标 : \033[1;36mX = " << res.x << " m, Y = " << res.y
                  << " m, Z = " << res.z << " m\033[0m\n";
        std::cout << "  · 锁定航向朝向 : \033[1;36mYaw = " << res.yaw << "°\033[0m (Roll="
                  << res.roll << "°, Pitch=" << res.pitch << "°)\n";
        std::cout << "  · 粗配准候选点 : X = " << res.coarse_x << " m, Y = " << res.coarse_y
                  << " m, Yaw = " << res.coarse_yaw << "°\n";
        std::cout << "  · 纯墙面咬合度 : \033[1;32m" << res.wall_fitness * 100.0
                  << " %\033[0m (Wall RMSE = " << res.wall_rmse * 100.0 << " cm)\n";
        std::cout << "  · 全点云咬合度 : " << res.fitness * 100.0
                  << " % (Overall RMSE = " << res.rmse * 100.0 << " cm)\n";
        std::cout << "  · 粗匹配耗时   : " << res.time_coarse_ms << " ms\n";
        std::cout << "  · 多假设ICP耗时: " << res.time_fine_ms << " ms\n";
        std::cout << "  · 在线总计算量 : \033[1;33m" << res.time_total_ms << " ms\033[0m\n";
        std::cout << "  · 实时可视化图 : \033[1;35m" << out_png << " (已同步更新)\033[0m\n";
        std::cout << "\033[1;32m====================================================================\033[0m\n\n";

        if (auto_exit_) {
            RCLCPP_INFO(this->get_logger(), "✅ 自定位完成，图像已生成，节点将在 2 秒后自动退出并交接给定位系统...");
            timer_exit_ = this->create_wall_timer(
                std::chrono::seconds(2), [this]() {
                    rclcpp::shutdown();
                });
        } else {
            RCLCPP_INFO(this->get_logger(), "👀 RViz 观察模式保持运行中 (持续广播 TF 与重定位对齐点云)... 按 Ctrl+C 退出");
            timer_tf_ = this->create_wall_timer(
                std::chrono::milliseconds(100), [this]() {
                    latest_tf_.header.stamp = this->now();
                    tf_broadcaster_->sendTransform(latest_tf_);
                });
        }
    }

    std::string home_dir_;
    std::string map_path_;
    std::string scan_topic_;
    std::string initialpose_topic_;
    bool auto_exit_ = true;
    bool publish_map_ = true;
    int accumulate_frames_ = 3;
    int frame_count_ = 0;
    bool reloc_completed_ = false;

    std::shared_ptr<open3d::geometry::PointCloud> accumulated_scan_;
    open3d_loc::GlobalMatcher matcher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_scan_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_initialpose_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_map_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_aligned_scan_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    geometry_msgs::msg::TransformStamped latest_tf_;
    rclcpp::TimerBase::SharedPtr timer_exit_;
    rclcpp::TimerBase::SharedPtr timer_tf_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AutoGlobalRelocNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
