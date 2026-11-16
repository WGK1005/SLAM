#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "rolling_map_updater/rolling_voxel_map.h"

namespace rolling_map_updater {

namespace {

void RosToOpen3D(const sensor_msgs::msg::PointCloud2& msg,
                 open3d::geometry::PointCloud& out) {
  out.Clear();
  const size_t n = static_cast<size_t>(msg.width) * static_cast<size_t>(msg.height);
  if (n == 0) return;

  out.points_.reserve(n);
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(msg, "z");

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    const float x = *iter_x;
    const float y = *iter_y;
    const float z = *iter_z;
    if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
      out.points_.emplace_back(x, y, z);
    }
  }
}

void Open3DToRos(const open3d::geometry::PointCloud& cloud,
                 const std::string& frame_id,
                 const rclcpp::Time& stamp,
                 sensor_msgs::msg::PointCloud2& msg) {
  msg.header.frame_id = frame_id;
  msg.header.stamp = stamp;
  msg.height = 1;
  msg.width = static_cast<uint32_t>(cloud.points_.size());
  msg.is_bigendian = false;
  msg.is_dense = true;

  sensor_msgs::PointCloud2Modifier modifier(msg);
  if (cloud.HasColors()) {
    modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  } else {
    modifier.setPointCloud2FieldsByString(1, "xyz");
  }
  modifier.resize(cloud.points_.size());

  sensor_msgs::PointCloud2Iterator<float> it_x(msg, "x");
  sensor_msgs::PointCloud2Iterator<float> it_y(msg, "y");
  sensor_msgs::PointCloud2Iterator<float> it_z(msg, "z");

  if (cloud.HasColors()) {
    sensor_msgs::PointCloud2Iterator<uint8_t> it_r(msg, "r");
    sensor_msgs::PointCloud2Iterator<uint8_t> it_g(msg, "g");
    sensor_msgs::PointCloud2Iterator<uint8_t> it_b(msg, "b");
    for (size_t i = 0; i < cloud.points_.size();
         ++i, ++it_x, ++it_y, ++it_z, ++it_r, ++it_g, ++it_b) {
      *it_x = static_cast<float>(cloud.points_[i].x());
      *it_y = static_cast<float>(cloud.points_[i].y());
      *it_z = static_cast<float>(cloud.points_[i].z());
      *it_r = static_cast<uint8_t>(std::clamp(cloud.colors_[i].x() * 255.0, 0.0, 255.0));
      *it_g = static_cast<uint8_t>(std::clamp(cloud.colors_[i].y() * 255.0, 0.0, 255.0));
      *it_b = static_cast<uint8_t>(std::clamp(cloud.colors_[i].z() * 255.0, 0.0, 255.0));
    }
  } else {
    for (size_t i = 0; i < cloud.points_.size(); ++i, ++it_x, ++it_y, ++it_z) {
      *it_x = static_cast<float>(cloud.points_[i].x());
      *it_y = static_cast<float>(cloud.points_[i].y());
      *it_z = static_cast<float>(cloud.points_[i].z());
    }
  }
}

Eigen::Matrix4d PoseMsgToMatrix(const geometry_msgs::msg::Pose& p) {
  Eigen::Quaterniond q(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
  if (q.norm() < 1e-8) {
    q = Eigen::Quaterniond::Identity();
  } else {
    q.normalize();
  }
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3, 3>(0, 0) = q.toRotationMatrix();
  T.block<3, 1>(0, 3) = Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
  return T;
}

Eigen::Matrix3d OrthonormalizeRot(const Eigen::Matrix3d& R_in) {
  if (!R_in.allFinite()) {
    return Eigen::Matrix3d::Identity();
  }
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(
      R_in, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d U = svd.matrixU();
  Eigen::Matrix3d V = svd.matrixV();
  if (U.determinant() * V.determinant() < 0.0) {
    U.col(2) *= -1.0;
  }
  return U * V.transpose();
}

}  // namespace

class RollingMapNode : public rclcpp::Node {
 public:
  RollingMapNode() : Node("rolling_map_node") {
    const char* home = std::getenv("HOME");
    std::string default_map = home ? (std::string(home) + "/maps/map_1021_1050.pcd") : "";

    this->declare_parameter<std::string>("prior_map_path", default_map);
    this->declare_parameter<std::string>("snapshot_export_path", "/tmp/rolling_map_snapshot.pcd");
    this->declare_parameter<std::string>("metrics_csv_path", "/tmp/rolling_match_metrics.csv");
    this->declare_parameter<std::string>("odom_topic", "/Odometry");
    this->declare_parameter<std::string>("scan_topic", "/cloud_registered_body");
    this->declare_parameter<std::string>("initialpose_topic", "/initialpose");
    this->declare_parameter<std::string>("map_frame", "map");
    this->declare_parameter<std::string>("odom_frame", "camera_init");
    this->declare_parameter<std::string>("body_frame", "body");
    this->declare_parameter<bool>("wait_for_initialpose", true);
    this->declare_parameter<bool>("enable_map_update", true);
    this->declare_parameter<bool>("log_every_frame", false);
    this->declare_parameter<double>("match_min_period_s", 0.2);
    this->declare_parameter<double>("max_update_omega_rad_s", 1.0);
    this->declare_parameter<double>("pose_smooth_alpha", 0.50);
    this->declare_parameter<int>("scan_stack_count", 3);
    this->declare_parameter<double>("accept_min_fit25", 0.52);
    this->declare_parameter<double>("accept_min_fit10", 0.45);
    this->declare_parameter<double>("accept_min_inlier5", 40.0);
    this->declare_parameter<double>("accept_max_p2pl_med_mm", 28.0);
    this->declare_parameter<double>("accept_max_dpos_m", 0.30);
    this->declare_parameter<double>("accept_max_dangle_deg", 10.0);
    this->declare_parameter<double>("recover_max_dpos_m", 0.85);
    this->declare_parameter<double>("recover_max_dangle_deg", 20.0);
    this->declare_parameter<double>("slip_min_fit25", 0.70);
    this->declare_parameter<double>("slip_min_fit10", 0.62);
    this->declare_parameter<double>("slip_min_inlier5", 45.0);
    this->declare_parameter<double>("slip_max_p2pl_med_mm", 21.0);
    this->declare_parameter<double>("slip_max_dpos_m", 2.00);
    this->declare_parameter<double>("slip_max_dangle_deg", 35.0);
    this->declare_parameter<double>("slip_min_improve", 0.14);
    this->declare_parameter<double>("max_corr_step_m", 0.12);
    this->declare_parameter<double>("max_corr_step_deg", 3.0);
    this->declare_parameter<int>("reloc_lost_trigger", 3);

    // RollingVoxelMap parameters
    RollingMapConfig cfg;
    this->declare_parameter<double>("chunk_size", cfg.chunk_size);
    this->declare_parameter<double>("voxel_size", cfg.voxel_size);
    this->declare_parameter<double>("active_radius_xy", cfg.active_radius_xy);
    this->declare_parameter<double>("active_z_down", cfg.active_z_down);
    this->declare_parameter<double>("active_z_up", cfg.active_z_up);
    this->declare_parameter<double>("roll_trigger_dist", cfg.roll_trigger_dist);
    this->declare_parameter<int>("min_roll_prior_voxels", static_cast<int>(cfg.min_roll_prior_voxels));
    this->declare_parameter<double>("scan_voxel_size", cfg.scan_voxel_size);
    this->declare_parameter<double>("target_icp_voxel_size", cfg.target_icp_voxel_size);
    this->declare_parameter<int>("max_target_icp_pts", static_cast<int>(cfg.max_target_icp_pts));
    this->declare_parameter<double>("icp_max_corr_dist", cfg.icp_max_corr_dist);
    this->declare_parameter<double>("coarse_corr_dist", cfg.coarse_corr_dist);
    this->declare_parameter<double>("coarse_trigger_fitness", cfg.coarse_trigger_fitness);
    this->declare_parameter<double>("coarse_trigger_fitness_10cm", cfg.coarse_trigger_fitness_10cm);
    this->declare_parameter<double>("fine_corr_dist", cfg.fine_corr_dist);
    this->declare_parameter<double>("icp_huber_k", cfg.icp_huber_k);
    this->declare_parameter<int>("icp_max_iterations", cfg.icp_max_iterations);
    this->declare_parameter<double>("eval_corr_dist", cfg.eval_corr_dist);
    this->declare_parameter<double>("degen_ratio_thresh", cfg.degen_ratio_thresh);
    this->declare_parameter<double>("degen_ratio_floor", cfg.degen_ratio_floor);
    this->declare_parameter<double>("degen_min_eigenvalue", cfg.degen_min_eigenvalue);
    this->declare_parameter<double>("scan_match_color_dist", cfg.scan_match_color_dist);
    this->declare_parameter<double>("error_eval_max_dist", cfg.error_eval_max_dist);
    this->declare_parameter<double>("min_update_fitness", cfg.min_update_fitness);
    this->declare_parameter<double>("max_update_rmse", cfg.max_update_rmse);
    this->declare_parameter<int>("min_hits_to_add", cfg.min_hits_to_add);
    this->declare_parameter<double>("new_point_min_dist", cfg.new_point_min_dist);
    this->declare_parameter<double>("ray_max_range", cfg.ray_max_range);
    this->declare_parameter<double>("ray_clearance_m", cfg.ray_clearance_m);
    this->declare_parameter<double>("live_keep_radius_m", cfg.live_keep_radius_m);
    this->declare_parameter<double>("ray_tube_radius_m", cfg.ray_tube_radius_m);

    prior_map_path_ = this->get_parameter("prior_map_path").as_string();
    snapshot_export_path_ = this->get_parameter("snapshot_export_path").as_string();
    metrics_csv_path_ = this->get_parameter("metrics_csv_path").as_string();
    odom_topic_ = this->get_parameter("odom_topic").as_string();
    scan_topic_ = this->get_parameter("scan_topic").as_string();
    initialpose_topic_ = this->get_parameter("initialpose_topic").as_string();
    map_frame_ = this->get_parameter("map_frame").as_string();
    odom_frame_ = this->get_parameter("odom_frame").as_string();
    body_frame_ = this->get_parameter("body_frame").as_string();
    wait_for_initialpose_ = this->get_parameter("wait_for_initialpose").as_bool();
    enable_map_update_ = this->get_parameter("enable_map_update").as_bool();
    log_every_frame_ = this->get_parameter("log_every_frame").as_bool();
    match_min_period_s_ = this->get_parameter("match_min_period_s").as_double();
    max_update_omega_ = this->get_parameter("max_update_omega_rad_s").as_double();
    pose_smooth_alpha_ = std::clamp(this->get_parameter("pose_smooth_alpha").as_double(), 0.05, 1.0);
    scan_stack_count_ = std::max<size_t>(1, static_cast<size_t>(this->get_parameter("scan_stack_count").as_int()));
    accept_min_fit25_ = this->get_parameter("accept_min_fit25").as_double();
    accept_min_fit10_ = this->get_parameter("accept_min_fit10").as_double();
    accept_min_inlier5_ = this->get_parameter("accept_min_inlier5").as_double();
    accept_max_p2pl_med_mm_ = this->get_parameter("accept_max_p2pl_med_mm").as_double();
    accept_max_dpos_m_ = this->get_parameter("accept_max_dpos_m").as_double();
    accept_max_dangle_deg_ = this->get_parameter("accept_max_dangle_deg").as_double();
    recover_max_dpos_m_ = this->get_parameter("recover_max_dpos_m").as_double();
    recover_max_dangle_deg_ = this->get_parameter("recover_max_dangle_deg").as_double();
    slip_min_fit25_ = this->get_parameter("slip_min_fit25").as_double();
    slip_min_fit10_ = this->get_parameter("slip_min_fit10").as_double();
    slip_min_inlier5_ = this->get_parameter("slip_min_inlier5").as_double();
    slip_max_p2pl_med_mm_ = this->get_parameter("slip_max_p2pl_med_mm").as_double();
    slip_max_dpos_m_ = this->get_parameter("slip_max_dpos_m").as_double();
    slip_max_dangle_deg_ = this->get_parameter("slip_max_dangle_deg").as_double();
    slip_min_improve_ = this->get_parameter("slip_min_improve").as_double();
    max_corr_step_m_ = this->get_parameter("max_corr_step_m").as_double();
    max_corr_step_deg_ = this->get_parameter("max_corr_step_deg").as_double();
    reloc_lost_trigger_ = std::max<int>(1, static_cast<int>(this->get_parameter("reloc_lost_trigger").as_int()));

    cfg.chunk_size = this->get_parameter("chunk_size").as_double();
    cfg.voxel_size = this->get_parameter("voxel_size").as_double();
    cfg.active_radius_xy = this->get_parameter("active_radius_xy").as_double();
    cfg.active_z_down = this->get_parameter("active_z_down").as_double();
    cfg.active_z_up = this->get_parameter("active_z_up").as_double();
    cfg.roll_trigger_dist = this->get_parameter("roll_trigger_dist").as_double();
    cfg.min_roll_prior_voxels = static_cast<size_t>(std::max<int64_t>(0, this->get_parameter("min_roll_prior_voxels").as_int()));
    cfg.scan_voxel_size = this->get_parameter("scan_voxel_size").as_double();
    cfg.target_icp_voxel_size = this->get_parameter("target_icp_voxel_size").as_double();
    cfg.max_target_icp_pts = static_cast<size_t>(std::max<int64_t>(0, this->get_parameter("max_target_icp_pts").as_int()));
    cfg.icp_max_corr_dist = this->get_parameter("icp_max_corr_dist").as_double();
    cfg.coarse_corr_dist = this->get_parameter("coarse_corr_dist").as_double();
    cfg.coarse_trigger_fitness = this->get_parameter("coarse_trigger_fitness").as_double();
    cfg.coarse_trigger_fitness_10cm = this->get_parameter("coarse_trigger_fitness_10cm").as_double();
    cfg.fine_corr_dist = this->get_parameter("fine_corr_dist").as_double();
    cfg.icp_huber_k = this->get_parameter("icp_huber_k").as_double();
    cfg.icp_max_iterations = static_cast<int>(this->get_parameter("icp_max_iterations").as_int());
    cfg.eval_corr_dist = this->get_parameter("eval_corr_dist").as_double();
    cfg.degen_ratio_thresh = this->get_parameter("degen_ratio_thresh").as_double();
    cfg.degen_ratio_floor = this->get_parameter("degen_ratio_floor").as_double();
    cfg.degen_min_eigenvalue = this->get_parameter("degen_min_eigenvalue").as_double();
    cfg.scan_match_color_dist = this->get_parameter("scan_match_color_dist").as_double();
    cfg.error_eval_max_dist = this->get_parameter("error_eval_max_dist").as_double();
    cfg.min_update_fitness = this->get_parameter("min_update_fitness").as_double();
    cfg.max_update_rmse = this->get_parameter("max_update_rmse").as_double();
    cfg.min_hits_to_add = static_cast<int>(this->get_parameter("min_hits_to_add").as_int());
    cfg.new_point_min_dist = this->get_parameter("new_point_min_dist").as_double();
    cfg.ray_max_range = this->get_parameter("ray_max_range").as_double();
    cfg.ray_clearance_m = this->get_parameter("ray_clearance_m").as_double();
    cfg.live_keep_radius_m = this->get_parameter("live_keep_radius_m").as_double();
    cfg.ray_tube_radius_m = this->get_parameter("ray_tube_radius_m").as_double();

    voxel_map_ = std::make_unique<RollingVoxelMap>(cfg);

    RCLCPP_INFO(this->get_logger(), "==========================================================");
    RCLCPP_INFO(this->get_logger(), "  [rolling_map_node] Standalone Rolling Map & Updater");
    RCLCPP_INFO(this->get_logger(), "  Prior Map (READ-ONLY): %s", prior_map_path_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Rolling Radius XY: %.1fm | Voxel: %.2fm | Chunk: %.1fm",
                cfg.active_radius_xy, cfg.voxel_size, cfg.chunk_size);
    RCLCPP_INFO(this->get_logger(), "  Two-Stage ICP: Coarse=%.2fm Fine=%.2fm | Stack=%zu",
                cfg.coarse_corr_dist, cfg.fine_corr_dist, scan_stack_count_);
    RCLCPP_INFO(this->get_logger(), "  Keep New & Delete Old: %s (RayClear=%.2fm)",
                enable_map_update_ ? "ENABLED" : "DISABLED", cfg.ray_clearance_m);
    RCLCPP_INFO(this->get_logger(), "  Per-Frame Metrics CSV: %s | Topic: /rolling_map/match_metrics",
                metrics_csv_path_.c_str());
    RCLCPP_INFO(this->get_logger(), "==========================================================");

    if (!voxel_map_->LoadPriorMapReadOnly(prior_map_path_)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to read prior map (read-only): %s", prior_map_path_.c_str());
      throw std::runtime_error("Cannot load prior map");
    }

    if (!metrics_csv_path_.empty()) {
      csv_stream_.open(metrics_csv_path_, std::ios::out | std::ios::trunc);
      if (csv_stream_.is_open()) {
        csv_stream_
            << "seq,stamp_sec,status,align_ms,fit_before,fitness_25cm,fitness_10cm,"
               "inlier_pct_5cm,inlier_pct_10cm,"
               "p2p_rmse_mm,p2p_mean_mm,p2p_median_mm,p2p_p95_mm,"
               "p2plane_rmse_mm,p2plane_mean_mm,p2plane_median_mm,p2plane_p95_mm,"
               "min_eig_ratio,degen_axes,"
               "corr_dx_mm,corr_dy_mm,corr_dz_mm,corr_dist_mm,corr_dyaw_deg,"
               "pose_x_m,pose_y_m,pose_z_m,pose_yaw_deg,added_new,deleted_old,active_pts\n";
        csv_stream_.flush();
      } else {
        RCLCPP_WARN(this->get_logger(), "Failed to open metrics_csv_path: %s",
                    metrics_csv_path_.c_str());
      }
    }

    if (!wait_for_initialpose_) {
      voxel_map_->RollWindow(Eigen::Vector3d::Zero(), true);
      pose_initialized_ = true;
      last_good_map_pos_ = Eigen::Vector3d::Zero();
      RCLCPP_INFO(this->get_logger(), "wait_for_initialpose=false -> initialized at map origin (0,0,0)");
    } else {
      RCLCPP_INFO(this->get_logger(), "Waiting for %s (e.g. from auto_global_reloc_node)...",
                  initialpose_topic_.c_str());
    }

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    auto latch_qos = rclcpp::QoS(1).transient_local().reliable();
    pub_prior_map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/rolling_map/prior_map", latch_qos);
    pub_active_submap_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/rolling_map/active_submap", 2);
    pub_new_points_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/rolling_map/new_points", 2);
    pub_removed_points_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/rolling_map/removed_points", 2);
    pub_aligned_scan_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/rolling_map/aligned_scan", 2);
    pub_rolling_pose_ = this->create_publisher<nav_msgs::msg::Odometry>("/rolling_pose", 20);
    pub_fitness_ = this->create_publisher<std_msgs::msg::Float32>("/rolling_map/fitness", 10);
    pub_rmse_mm_ = this->create_publisher<std_msgs::msg::Float32>("/rolling_map/rmse_mm", 10);
    pub_p2pl_med_mm_ = this->create_publisher<std_msgs::msg::Float32>("/rolling_map/p2plane_median_mm", 10);
    pub_match_metrics_ = this->create_publisher<std_msgs::msg::String>("/rolling_map/match_metrics", 20);

    // Publish initial read-only prior map once for RViz context
    {
      auto prior_cloud = voxel_map_->GetWorkingGlobalCloud();
      sensor_msgs::msg::PointCloud2 msg;
      Open3DToRos(*prior_cloud, map_frame_, this->now(), msg);
      pub_prior_map_->publish(msg);
      RCLCPP_INFO(this->get_logger(), "Loaded %zu read-only prior voxels into %zu chunks.",
                  prior_cloud->points_.size(), voxel_map_->GetStats().total_chunk_count);
    }

    cb_group_odom_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    cb_group_scan_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    cb_group_viz_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions odom_opts;
    odom_opts.callback_group = cb_group_odom_;
    rclcpp::SubscriptionOptions scan_opts;
    scan_opts.callback_group = cb_group_scan_;

    srv_export_ = this->create_service<std_srvs::srv::Trigger>(
        "/rolling_map/export_snapshot",
        std::bind(&RollingMapNode::CallbackExportSnapshot, this,
                  std::placeholders::_1, std::placeholders::_2));

    sub_initialpose_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        initialpose_topic_, 10,
        std::bind(&RollingMapNode::CallbackInitialPose, this, std::placeholders::_1), scan_opts);

    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, rclcpp::SensorDataQoS(),
        std::bind(&RollingMapNode::CallbackOdom, this, std::placeholders::_1), odom_opts);

    sub_scan_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        scan_topic_, rclcpp::SensorDataQoS(),
        std::bind(&RollingMapNode::CallbackScan, this, std::placeholders::_1), scan_opts);

    timer_viz_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&RollingMapNode::PublishVisualizations, this), cb_group_viz_);

    match_thread_ = std::thread(&RollingMapNode::MatchWorkerLoop, this);
  }

  ~RollingMapNode() override {
    {
      std::lock_guard<std::mutex> lock(scan_mutex_);
      stop_worker_ = true;
    }
    match_cv_.notify_all();
    if (match_thread_.joinable()) {
      match_thread_.join();
    }
    if (csv_stream_.is_open()) {
      csv_stream_.flush();
      csv_stream_.close();
    }
  }

 private:
  struct StampedOdom {
    double t_sec = 0.0;
    Eigen::Matrix4d T_odom_body = Eigen::Matrix4d::Identity();
  };

  struct PendingScan {
    double t_sec = 0.0;
    rclcpp::Time stamp_ros;
    std::string frame_id;
    open3d::geometry::PointCloud cloud;
    uint64_t seq = 0;
  };

  struct ScanHistEntry {
    double t_sec = 0.0;
    Eigen::Matrix4d T_odom_body = Eigen::Matrix4d::Identity();
    open3d::geometry::PointCloud cloud_body;
  };

  bool InterpolateOdomPose(double target_t_sec, Eigen::Matrix4d& out_T, double& out_omega) {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    if (odom_buffer_.empty()) {
      return false;
    }
    out_omega = current_omega_;
    if (target_t_sec <= odom_buffer_.front().t_sec) {
      out_T = odom_buffer_.front().T_odom_body;
      return true;
    }
    if (target_t_sec >= odom_buffer_.back().t_sec) {
      out_T = odom_buffer_.back().T_odom_body;
      return true;
    }
    auto it = std::lower_bound(
        odom_buffer_.begin(), odom_buffer_.end(), target_t_sec,
        [](const StampedOdom& elem, double t) { return elem.t_sec < t; });
    if (it == odom_buffer_.begin()) {
      out_T = it->T_odom_body;
      return true;
    }
    const auto it_prev = std::prev(it);
    const double dt = it->t_sec - it_prev->t_sec;
    if (dt < 1e-6) {
      out_T = it->T_odom_body;
      return true;
    }
    const double a = std::clamp((target_t_sec - it_prev->t_sec) / dt, 0.0, 1.0);
    const Eigen::Vector3d t0 = it_prev->T_odom_body.block<3, 1>(0, 3);
    const Eigen::Vector3d t1 = it->T_odom_body.block<3, 1>(0, 3);
    Eigen::Quaterniond q0(it_prev->T_odom_body.block<3, 3>(0, 0));
    Eigen::Quaterniond q1(it->T_odom_body.block<3, 3>(0, 0));
    q0.normalize();
    q1.normalize();
    if (q0.dot(q1) < 0.0) {
      q1.coeffs() = -q1.coeffs();
    }
    Eigen::Quaterniond q_interp = q0.slerp(a, q1);
    q_interp.normalize();
    out_T = Eigen::Matrix4d::Identity();
    out_T.block<3, 1>(0, 3) = (1.0 - a) * t0 + a * t1;
    out_T.block<3, 3>(0, 0) = q_interp.toRotationMatrix();
    return true;
  }

  void CallbackInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    const Eigen::Matrix4d T_map_base_init = PoseMsgToMatrix(msg->pose.pose);
    Eigen::Matrix4d T_odom_latest = Eigen::Matrix4d::Identity();
    {
      std::lock_guard<std::mutex> lock(odom_mutex_);
      if (!odom_buffer_.empty()) {
        T_odom_latest = odom_buffer_.back().T_odom_body;
      }
    }
    {
      std::lock_guard<std::mutex> lock(pose_mutex_);
      // T_map_odom = T_map_body * T_odom_body^-1
      T_map_odom_ = T_map_base_init * T_odom_latest.inverse();
      T_map_odom_.block<3, 3>(0, 0) = OrthonormalizeRot(T_map_odom_.block<3, 3>(0, 0));
      last_good_map_pos_ = T_map_base_init.block<3, 1>(0, 3);
      last_good_map_T_ = T_map_base_init;
      last_good_odom_T_ = T_odom_latest;
      lost_streak_ = 0;
      pose_initialized_ = true;
    }
    const Eigen::Vector3d pos = T_map_base_init.block<3, 1>(0, 3);
    voxel_map_->RollWindow(pos, true);
    viz_active_dirty_.store(true);
    RCLCPP_INFO(this->get_logger(),
                "Received /initialpose at (%.2f, %.2f, %.2f) -> Rolling submap anchored.",
                pos.x(), pos.y(), pos.z());
  }

  void CallbackOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
    const Eigen::Matrix4d T_curr = PoseMsgToMatrix(msg->pose.pose);
    const double t_sec = rclcpp::Time(msg->header.stamp).seconds();

    {
      std::lock_guard<std::mutex> lock(odom_mutex_);
      if (!odom_buffer_.empty() && t_sec > odom_buffer_.back().t_sec + 1e-4) {
        const double dt = t_sec - odom_buffer_.back().t_sec;
        const Eigen::Matrix3d dR =
            odom_buffer_.back().T_odom_body.block<3, 3>(0, 0).transpose() *
            T_curr.block<3, 3>(0, 0);
        const Eigen::AngleAxisd aa(dR);
        current_omega_ = std::abs(aa.angle()) / dt;
      }
      if (odom_buffer_.empty() || t_sec > odom_buffer_.back().t_sec + 1e-6) {
        odom_buffer_.push_back(StampedOdom{t_sec, T_curr});
        while (odom_buffer_.size() > 400) {
          odom_buffer_.pop_front();
        }
      } else {
        odom_buffer_.back().T_odom_body = T_curr;
      }
    }

    Eigen::Matrix4d T_map_odom_copy;
    bool initialized = false;
    {
      std::lock_guard<std::mutex> lock(pose_mutex_);
      T_map_odom_copy = T_map_odom_;
      initialized = pose_initialized_;
    }

    // Broadcast map -> camera_init (odom_frame_) immediately without waiting for ICP
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = msg->header.stamp;
    tf_msg.header.frame_id = map_frame_;
    tf_msg.child_frame_id = odom_frame_;
    Eigen::Isometry3d iso_mo;
    iso_mo.matrix() = T_map_odom_copy;
    tf_msg.transform = tf2::eigenToTransform(iso_mo).transform;
    tf_broadcaster_->sendTransform(tf_msg);

    if (initialized) {
      Eigen::Isometry3d iso_mb;
      iso_mb.matrix() = T_map_odom_copy * T_curr;
      nav_msgs::msg::Odometry pose_out;
      pose_out.header.stamp = msg->header.stamp;
      pose_out.header.frame_id = map_frame_;
      pose_out.child_frame_id = body_frame_;
      pose_out.pose.pose = tf2::toMsg(iso_mb);
      pub_rolling_pose_->publish(pose_out);
    }
  }

  void CallbackScan(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    open3d::geometry::PointCloud scan_raw;
    RosToOpen3D(*msg, scan_raw);
    if (scan_raw.points_.size() < 50) {
      return;
    }
    const double t_sec = rclcpp::Time(msg->header.stamp).seconds();

    {
      std::lock_guard<std::mutex> lock(scan_mutex_);
      pending_scan_.t_sec = t_sec;
      pending_scan_.stamp_ros = msg->header.stamp;
      pending_scan_.frame_id = msg->header.frame_id;
      pending_scan_.cloud = std::move(scan_raw);
      pending_scan_.seq = ++scan_seq_;
      has_pending_scan_ = true;
    }
    match_cv_.notify_one();
  }

  void MatchWorkerLoop() {
    std::deque<ScanHistEntry> scan_history;
    auto last_process_wall = std::chrono::steady_clock::time_point::min();

    while (rclcpp::ok()) {
      PendingScan cur;
      {
        std::unique_lock<std::mutex> lock(scan_mutex_);
        match_cv_.wait(lock, [this]() { return stop_worker_ || has_pending_scan_; });
        if (stop_worker_) break;

        if (last_process_wall != std::chrono::steady_clock::time_point::min()) {
          const auto target_wall =
              last_process_wall + std::chrono::duration<double>(match_min_period_s_);
          if (std::chrono::steady_clock::now() < target_wall) {
            match_cv_.wait_until(lock, target_wall, [this]() { return stop_worker_; });
            if (stop_worker_) break;
          }
        }
        cur = std::move(pending_scan_);
        has_pending_scan_ = false;
      }

      bool initialized = false;
      Eigen::Matrix4d T_map_odom_copy;
      Eigen::Vector3d last_good_pos;
      Eigen::Matrix4d last_good_T;
      Eigen::Matrix4d last_good_odom_T;
      size_t cur_lost_streak = 0;
      {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        initialized = pose_initialized_;
        T_map_odom_copy = T_map_odom_;
        last_good_pos = last_good_map_pos_;
        last_good_T = last_good_map_T_;
        last_good_odom_T = last_good_odom_T_;
        cur_lost_streak = lost_streak_;
      }
      if (!initialized) {
        continue;
      }

      // Wait briefly if scan timestamp slightly leads latest odom message
      for (int w = 0; w < 10; ++w) {
        double latest_t = 0.0;
        {
          std::lock_guard<std::mutex> lock(odom_mutex_);
          if (!odom_buffer_.empty()) latest_t = odom_buffer_.back().t_sec;
        }
        if (latest_t >= cur.t_sec - 1e-4) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }

      Eigen::Matrix4d T_odom_body_cur = Eigen::Matrix4d::Identity();
      double omega_copy = 0.0;
      if (!InterpolateOdomPose(cur.t_sec, T_odom_body_cur, omega_copy)) {
        continue;
      }
      last_process_wall = std::chrono::steady_clock::now();

      open3d::geometry::PointCloud scan_single = std::move(cur.cloud);
      if (cur.frame_id == odom_frame_) {
        scan_single.Transform(T_odom_body_cur.inverse());
      }

      // Build odom-compensated multi-frame local scan stack for robust ICP during turns/slips
      scan_history.push_back(ScanHistEntry{cur.t_sec, T_odom_body_cur, scan_single});
      while (scan_history.size() > scan_stack_count_ ||
             (!scan_history.empty() && (cur.t_sec - scan_history.front().t_sec) > 0.55)) {
        scan_history.pop_front();
      }

      open3d::geometry::PointCloud scan_stacked = scan_single;
      if (scan_history.size() > 1) {
        const Eigen::Matrix4d T_body_cur_odom = T_odom_body_cur.inverse();
        for (size_t i = 0; i + 1 < scan_history.size(); ++i) {
          open3d::geometry::PointCloud prev_in_cur = scan_history[i].cloud_body;
          prev_in_cur.Transform(T_body_cur_odom * scan_history[i].T_odom_body);
          scan_stacked += prev_in_cur;
        }
      }

      const Eigen::Matrix4d T_map_body_pred = T_map_odom_copy * T_odom_body_cur;
      const Eigen::Vector3d pred_pos = T_map_body_pred.block<3, 1>(0, 3);
      const double active_r = voxel_map_->config().active_radius_xy;
      const bool near_prior = voxel_map_->HasPriorNearby(pred_pos, active_r);

      // Anti-escape guard: only roll submap window when locked and near/inside prior map
      bool rolled = false;
      if (cur_lost_streak == 0 && near_prior) {
        rolled = voxel_map_->RollWindow(pred_pos, false);
      }

      AlignmentResult align =
          voxel_map_->AlignScanPointToPlane(scan_stacked, T_map_body_pred, &scan_single);
      Eigen::Matrix4d T_map_body_used = T_map_body_pred;
      std::string status = "LOST";
      bool pose_accepted = false;
      bool big_recovery = false;

      const double dpos =
          (align.T_map_body.block<3, 1>(0, 3) - pred_pos).norm();
      const Eigen::Matrix3d dR = OrthonormalizeRot(
          T_map_body_pred.block<3, 3>(0, 0).transpose() * align.T_map_body.block<3, 3>(0, 0));
      const double dangle_deg = Eigen::AngleAxisd(dR).angle() * 180.0 / M_PI;

      const bool valid_mm = (align.inlier_points_15cm >= 20);
      const bool high_overlap_p2pl_ok =
          (align.p2plane_median_mm <= accept_max_p2pl_med_mm_) ||
          (align.fitness >= 0.88 && align.fitness_10cm >= 0.75 &&
           align.inlier_pct_5cm >= 45.0 && align.p2plane_median_mm <= 32.0 &&
           dpos <= 0.20 && dangle_deg <= 6.0);
      if (align.success && !align.used_coarse_stage && valid_mm &&
          align.fitness >= accept_min_fit25_ &&
          align.fitness_10cm >= accept_min_fit10_ &&
          align.inlier_pct_5cm >= accept_min_inlier5_ &&
          high_overlap_p2pl_ok &&
          dpos <= accept_max_dpos_m_ && dangle_deg <= accept_max_dangle_deg_) {
        pose_accepted = true;
        status = "TRACKED";
      } else if (align.success && align.used_coarse_stage && valid_mm &&
                 align.fitness >= std::max(0.58, accept_min_fit25_) &&
                 align.fitness_10cm >= std::max(0.52, accept_min_fit10_) &&
                 align.inlier_pct_5cm >= accept_min_inlier5_ &&
                 align.p2plane_median_mm <= std::min(25.0, accept_max_p2pl_med_mm_) &&
                 align.fitness_10cm >= align.fitness_10cm_before + 0.03 &&
                 dpos <= recover_max_dpos_m_ && dangle_deg <= recover_max_dangle_deg_) {
        pose_accepted = true;
        status = "RECOVERED";
      } else if (align.success && valid_mm &&
                 align.fitness >= slip_min_fit25_ &&
                 align.fitness_10cm >= slip_min_fit10_ &&
                 align.inlier_pct_5cm >= slip_min_inlier5_ &&
                 align.p2plane_median_mm <= slip_max_p2pl_med_mm_ &&
                 align.fitness >= align.fitness_before + slip_min_improve_ &&
                 align.fitness_10cm >= align.fitness_10cm_before + slip_min_improve_ &&
                 dpos <= slip_max_dpos_m_ && dangle_deg <= slip_max_dangle_deg_) {
        pose_accepted = true;
        big_recovery = true;
        status = "SLIP_RECOVERED";
      } else if (align.success && align.fitness >= 0.45) {
        status = "REJECTED_JUMP";
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "Rejected ICP jump: dpos=%.2fm dangle=%.1fdeg Fit25=%.2f(10:%.2f,5cm:%.0f%%) p2plMed=%.1fmm",
            dpos, dangle_deg, align.fitness, align.fitness_10cm,
            align.inlier_pct_5cm, align.p2plane_median_mm);
      }

      auto valid_reloc_result = [this](const AlignmentResult& r) -> bool {
        return r.success && r.inlier_points_15cm >= 20 &&
               r.fitness >= std::max(0.58, accept_min_fit25_) &&
               r.fitness_10cm >= std::max(0.52, accept_min_fit10_) &&
               r.inlier_pct_5cm >= accept_min_inlier5_ &&
               r.p2plane_median_mm <= std::min(25.0, accept_max_p2pl_med_mm_);
      };

      // If tracking has been lost >= reloc_lost_trigger_ frames, run local recovery anchored around last good pose
      if (!pose_accepted && static_cast<int>(cur_lost_streak + 1) >= reloc_lost_trigger_) {
        AlignmentResult rec_align;
        bool recovered = false;
        if (near_prior &&
            voxel_map_->TryRecoverLocalRelocalization(
                scan_stacked, T_map_body_pred, rec_align, &scan_single) &&
            valid_reloc_result(rec_align)) {
          recovered = true;
        }
        if (!recovered) {
          const Eigen::Matrix4d T_hint_odom =
              last_good_T * last_good_odom_T.inverse() * T_odom_body_cur;
          const Eigen::Vector3d odom_hint_pos = T_hint_odom.block<3, 1>(0, 3);
          if (voxel_map_->HasPriorNearby(odom_hint_pos, active_r)) {
            rolled = voxel_map_->RollWindow(odom_hint_pos, false) || rolled;
            if (voxel_map_->TryRecoverLocalRelocalization(
                    scan_stacked, T_hint_odom, rec_align, &scan_single) &&
                valid_reloc_result(rec_align)) {
              recovered = true;
            }
          }
          if (!recovered && voxel_map_->HasPriorNearby(last_good_pos, active_r)) {
            rolled = voxel_map_->RollWindow(last_good_pos, false) || rolled;
            if (voxel_map_->TryRecoverLocalRelocalization(
                    scan_stacked, last_good_T, rec_align, &scan_single) &&
                valid_reloc_result(rec_align)) {
              recovered = true;
            }
          }
        }
        if (recovered) {
          align = rec_align;
          pose_accepted = true;
          big_recovery = true;
          status = "RELOC_RECOVERED";
        }
      }

      if (pose_accepted && align.T_map_body.allFinite()) {
        const Eigen::Matrix4d T_map_odom_raw = align.T_map_body * T_odom_body_cur.inverse();
        const double alpha = (status == "RELOC_RECOVERED") ? 1.0 : (big_recovery ? 0.90 : pose_smooth_alpha_);
        if (T_map_odom_raw.allFinite()) {
          std::lock_guard<std::mutex> lock(pose_mutex_);
          const Eigen::Vector3d t_old = T_map_odom_.block<3, 1>(0, 3);
          const Eigen::Vector3d t_target = T_map_odom_raw.block<3, 1>(0, 3);
          const Eigen::Matrix3d R_old = OrthonormalizeRot(T_map_odom_.block<3, 3>(0, 0));
          const Eigen::Matrix3d R_target = OrthonormalizeRot(T_map_odom_raw.block<3, 3>(0, 0));

          Eigen::Quaterniond q_old(R_old);
          Eigen::Quaterniond q_target(R_target);
          q_old.normalize();
          q_target.normalize();
          if (q_old.dot(q_target) < 0.0) {
            q_target.coeffs() = -q_target.coeffs();
          }

          Eigen::Vector3d dt = alpha * (t_target - t_old);
          Eigen::Quaterniond q_blend = q_old.slerp(alpha, q_target);
          q_blend.normalize();

          Eigen::Quaterniond q_new = q_blend;
          if (!big_recovery) {
            const double dt_norm = dt.norm();
            if (dt_norm > max_corr_step_m_ && dt_norm > 1e-6) {
              dt *= (max_corr_step_m_ / dt_norm);
            }
            Eigen::Quaterniond dq = q_old.conjugate() * q_blend;
            dq.normalize();
            if (dq.w() < 0.0) {
              dq.coeffs() = -dq.coeffs();
            }
            Eigen::AngleAxisd aa(dq);
            const double max_rad = max_corr_step_deg_ * M_PI / 180.0;
            if (std::abs(aa.angle()) > max_rad) {
              const double clamped_ang = std::copysign(max_rad, aa.angle());
              Eigen::Quaterniond dq_clamped(Eigen::AngleAxisd(clamped_ang, aa.axis()));
              q_new = (q_old * dq_clamped).normalized();
            }
          }

          T_map_odom_.block<3, 1>(0, 3) = t_old + dt;
          T_map_odom_.block<3, 3>(0, 0) = q_new.toRotationMatrix();
          T_map_body_used = T_map_odom_ * T_odom_body_cur;

          const bool high_conf_tracked =
              (!big_recovery && valid_mm &&
               align.inlier_pct_5cm >= 52.0 &&
               align.p2plane_median_mm <= 22.0 &&
               dpos <= 0.25);
          const bool high_conf_reloc =
              (status == "RELOC_RECOVERED" && valid_mm &&
               align.fitness >= 0.85 &&
               align.fitness_10cm >= 0.75 &&
               align.inlier_pct_5cm >= 52.0 &&
               align.p2plane_median_mm <= 22.0);
          if (high_conf_tracked || high_conf_reloc) {
            last_good_map_pos_ = T_map_body_used.block<3, 1>(0, 3);
            last_good_map_T_ = T_map_body_used;
            last_good_odom_T_ = T_odom_body_cur;
          }
          lost_streak_ = 0;
        }
        if (!rolled && voxel_map_->HasPriorNearby(T_map_body_used.block<3, 1>(0, 3), active_r)) {
          rolled = voxel_map_->RollWindow(T_map_body_used.block<3, 1>(0, 3), false);
        }
      } else {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        lost_streak_++;
      }

      // Online Keep-New & Delete-Old update (only when well-aligned inside prior map region)
      size_t added = 0;
      size_t deleted = 0;
      const Eigen::Vector3d used_pos = T_map_body_used.block<3, 1>(0, 3);
      const bool used_near_prior = voxel_map_->HasPriorNearby(used_pos, active_r * 0.75);
      const bool settling_stable =
          (status == "TRACKED" && !rolled && valid_mm &&
           align.inlier_pct_5cm >= 58.0 &&
           align.p2plane_median_mm <= 20.0 &&
           align.corr_dist_mm <= 60.0 &&
           std::abs(align.corr_dyaw_deg) <= 1.5);
      if (enable_map_update_ && pose_accepted && !big_recovery &&
          used_near_prior && settling_stable && omega_copy <= max_update_omega_) {
        auto upd = voxel_map_->UpdateKeepNewDeleteOld(
            scan_single, T_map_body_used, align.fitness, align.inlier_rmse);
        added = upd.first;
        deleted = upd.second;
        if (added > 0 || deleted > 0) {
          voxel_map_->RefreshActiveTargetCloud(false);
        }
      }
      if (rolled || added > 0 || deleted > 0) {
        viz_active_dirty_.store(true);
      }

      // Publish colored aligned scan (Green=matched, Red=unmatched/lost)
      if (pub_aligned_scan_->get_subscription_count() > 0) {
        open3d::geometry::PointCloud aligned = scan_single;
        aligned.Transform(T_map_body_used);
        auto colored_scan = voxel_map_->ColorizeAlignedScanByMatch(aligned, pose_accepted);
        sensor_msgs::msg::PointCloud2 scan_msg;
        Open3DToRos(*colored_scan, map_frame_, cur.stamp_ros, scan_msg);
        pub_aligned_scan_->publish(scan_msg);
      }

      PublishAndRecordMetrics(cur.seq, cur.t_sec, status, align, T_map_body_used, added, deleted, rolled);
    }
  }

  void PublishAndRecordMetrics(uint64_t seq,
                               double t_sec,
                               const std::string& status,
                               const AlignmentResult& align,
                               const Eigen::Matrix4d& T_used,
                               size_t added,
                               size_t deleted,
                               bool rolled) {
    std_msgs::msg::Float32 fit_msg;
    fit_msg.data = static_cast<float>(align.fitness);
    pub_fitness_->publish(fit_msg);

    std_msgs::msg::Float32 rmse_msg;
    rmse_msg.data = static_cast<float>(align.p2p_rmse_mm);
    pub_rmse_mm_->publish(rmse_msg);

    std_msgs::msg::Float32 p2pl_msg;
    p2pl_msg.data = static_cast<float>(align.p2plane_median_mm);
    pub_p2pl_med_mm_->publish(p2pl_msg);

    const auto st = voxel_map_->GetStats();
    const Eigen::Vector3d p = T_used.block<3, 1>(0, 3);
    const Eigen::Matrix3d R = T_used.block<3, 3>(0, 0);
    const double yaw_deg = std::atan2(R(1, 0), R(0, 0)) * 180.0 / M_PI;

    std::ostringstream js;
    js << std::fixed
       << "{\"seq\":" << seq
       << ",\"stamp\":" << std::setprecision(4) << t_sec
       << ",\"status\":\"" << status << "\""
       << ",\"align_ms\":" << std::setprecision(2) << align.align_time_ms
       << ",\"fit_before\":" << std::setprecision(4) << align.fitness_before
       << ",\"fitness_25cm\":" << align.fitness
       << ",\"fitness_10cm\":" << align.fitness_10cm
       << ",\"inlier_pct_5cm\":" << std::setprecision(2) << align.inlier_pct_5cm
       << ",\"inlier_pct_10cm\":" << align.inlier_pct_10cm
       << ",\"p2p_rmse_mm\":" << align.p2p_rmse_mm
       << ",\"p2p_mean_mm\":" << align.p2p_mean_mm
       << ",\"p2p_median_mm\":" << align.p2p_median_mm
       << ",\"p2p_p95_mm\":" << align.p2p_p95_mm
       << ",\"p2plane_rmse_mm\":" << align.p2plane_rmse_mm
       << ",\"p2plane_mean_mm\":" << align.p2plane_mean_mm
       << ",\"p2plane_median_mm\":" << align.p2plane_median_mm
       << ",\"p2plane_p95_mm\":" << align.p2plane_p95_mm
       << ",\"min_eig_ratio\":" << std::setprecision(4) << align.min_eig_ratio
       << ",\"degen_axes\":" << align.degen_axis_count
       << ",\"corr_dx_mm\":" << std::setprecision(2) << align.corr_dx_mm
       << ",\"corr_dy_mm\":" << align.corr_dy_mm
       << ",\"corr_dz_mm\":" << align.corr_dz_mm
       << ",\"corr_dist_mm\":" << align.corr_dist_mm
       << ",\"corr_dyaw_deg\":" << std::setprecision(3) << align.corr_dyaw_deg
       << ",\"pose_m\":[" << p.x() << "," << p.y() << "," << p.z() << "]"
       << ",\"yaw_deg\":" << std::setprecision(2) << yaw_deg
       << ",\"added_new\":" << added
       << ",\"deleted_old\":" << deleted
       << ",\"active_pts\":" << st.active_submap_points << "}";
    std_msgs::msg::String metrics_msg;
    metrics_msg.data = js.str();
    pub_match_metrics_->publish(metrics_msg);

    if (csv_stream_.is_open()) {
      csv_stream_
          << seq << "," << std::fixed << std::setprecision(4) << t_sec << "," << status << ","
          << std::setprecision(2) << align.align_time_ms << ","
          << std::setprecision(4) << align.fitness_before << "," << align.fitness << ","
          << align.fitness_10cm << "," << std::setprecision(2) << align.inlier_pct_5cm << ","
          << align.inlier_pct_10cm << ","
          << align.p2p_rmse_mm << "," << align.p2p_mean_mm << ","
          << align.p2p_median_mm << "," << align.p2p_p95_mm << ","
          << align.p2plane_rmse_mm << "," << align.p2plane_mean_mm << ","
          << align.p2plane_median_mm << "," << align.p2plane_p95_mm << ","
          << std::setprecision(4) << align.min_eig_ratio << "," << align.degen_axis_count << ","
          << std::setprecision(2) << align.corr_dx_mm << "," << align.corr_dy_mm << "," << align.corr_dz_mm << ","
          << align.corr_dist_mm << "," << std::setprecision(3) << align.corr_dyaw_deg << ","
          << p.x() << "," << p.y() << "," << p.z() << "," << std::setprecision(2) << yaw_deg << ","
          << added << "," << deleted << "," << st.active_submap_points << "\n";
      csv_stream_.flush();
    }

    if (log_every_frame_) {
      RCLCPP_INFO(
          this->get_logger(),
          "[#%llu %s%s] Fit=%.2f(10cm:%.2f) | p2pRMSE=%.1fmm p2plMed=%.1fmm p95=%.1fmm | dPose=%.1fmm/%.2fdeg | Align=%.1fms | +N=%zu -D=%zu",
          static_cast<unsigned long long>(seq), status.c_str(), rolled ? "*" : "",
          align.fitness, align.fitness_10cm, align.p2p_rmse_mm, align.p2plane_median_mm,
          align.p2p_p95_mm, align.corr_dist_mm, align.corr_dyaw_deg, align.align_time_ms,
          added, deleted);
    } else {
      RCLCPP_INFO_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "[Roll #%zu%s %s] ActivePts=%zu | Fit=%.2f(5cm:%.0f%%) | p2pRMSE=%.1fmm p2plMed=%.1fmm p95=%.1fmm | dPose=%.1fmm/%.2fdeg | Align=%.1fms | +New=%zu -Del=%zu",
          st.roll_count, rolled ? "*" : "", status.c_str(), st.active_submap_points,
          align.fitness, align.inlier_pct_5cm, align.p2p_rmse_mm, align.p2plane_median_mm,
          align.p2plane_p95_mm, align.corr_dist_mm, align.corr_dyaw_deg,
          align.align_time_ms, added, deleted);
    }
  }

  void PublishVisualizations() {
    bool initialized = false;
    {
      std::lock_guard<std::mutex> lock(pose_mutex_);
      initialized = pose_initialized_;
    }
    if (!initialized) return;
    const rclcpp::Time stamp = this->now();
    const bool force_submap_pub = (viz_tick_count_++ % 4 == 0);

    if (pub_active_submap_->get_subscription_count() > 0 &&
        (viz_active_dirty_.exchange(false) || force_submap_pub)) {
      auto sub = voxel_map_->GetActiveSubmapCloud();
      sensor_msgs::msg::PointCloud2 msg;
      Open3DToRos(*sub, map_frame_, stamp, msg);
      pub_active_submap_->publish(msg);
    }
    if (pub_new_points_->get_subscription_count() > 0) {
      auto np = voxel_map_->GetNewKeptCloud();
      sensor_msgs::msg::PointCloud2 msg;
      Open3DToRos(*np, map_frame_, stamp, msg);
      pub_new_points_->publish(msg);
    }
    if (pub_removed_points_->get_subscription_count() > 0) {
      auto dp = voxel_map_->GetDeletedOldCloud();
      sensor_msgs::msg::PointCloud2 msg;
      Open3DToRos(*dp, map_frame_, stamp, msg);
      pub_removed_points_->publish(msg);
    }
  }

  void CallbackExportSnapshot(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
      std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    // Hard safety check: NEVER allow overwriting the original prior map file!
    if (snapshot_export_path_ == prior_map_path_ || snapshot_export_path_.empty()) {
      res->success = false;
      res->message = "Refused: export path must differ from read-only prior_map_path.";
      return;
    }
    if (voxel_map_->ExportSnapshotToPath(snapshot_export_path_)) {
      auto st = voxel_map_->GetStats();
      res->success = true;
      res->message = "Exported snapshot to " + snapshot_export_path_ +
                     " (new_kept=" + std::to_string(st.new_kept_count) +
                     ", deleted_old=" + std::to_string(st.deleted_old_count) + ")";
    } else {
      res->success = false;
      res->message = "Export failed or map empty.";
    }
  }

  std::string prior_map_path_;
  std::string snapshot_export_path_;
  std::string metrics_csv_path_;
  std::string odom_topic_;
  std::string scan_topic_;
  std::string initialpose_topic_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string body_frame_;

  bool wait_for_initialpose_ = true;
  bool enable_map_update_ = true;
  bool log_every_frame_ = false;
  double match_min_period_s_ = 0.2;
  double max_update_omega_ = 1.0;
  double pose_smooth_alpha_ = 0.50;
  size_t scan_stack_count_ = 3;
  double accept_min_fit25_ = 0.52;
  double accept_min_fit10_ = 0.45;
  double accept_min_inlier5_ = 40.0;
  double accept_max_p2pl_med_mm_ = 28.0;
  double accept_max_dpos_m_ = 0.30;
  double accept_max_dangle_deg_ = 10.0;
  double recover_max_dpos_m_ = 0.85;
  double recover_max_dangle_deg_ = 20.0;
  double slip_min_fit25_ = 0.70;
  double slip_min_fit10_ = 0.62;
  double slip_min_inlier5_ = 45.0;
  double slip_max_p2pl_med_mm_ = 21.0;
  double slip_max_dpos_m_ = 2.00;
  double slip_max_dangle_deg_ = 35.0;
  double slip_min_improve_ = 0.14;
  double max_corr_step_m_ = 0.12;
  double max_corr_step_deg_ = 3.0;
  int reloc_lost_trigger_ = 3;

  std::unique_ptr<RollingVoxelMap> voxel_map_;
  std::ofstream csv_stream_;

  std::mutex odom_mutex_;
  std::deque<StampedOdom> odom_buffer_;
  double current_omega_ = 0.0;

  std::mutex pose_mutex_;
  Eigen::Matrix4d T_map_odom_ = Eigen::Matrix4d::Identity();
  Eigen::Vector3d last_good_map_pos_ = Eigen::Vector3d::Zero();
  Eigen::Matrix4d last_good_map_T_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d last_good_odom_T_ = Eigen::Matrix4d::Identity();
  size_t lost_streak_ = 0;
  bool pose_initialized_ = false;
  std::atomic<bool> viz_active_dirty_{true};
  size_t viz_tick_count_ = 0;

  std::mutex scan_mutex_;
  std::condition_variable match_cv_;
  PendingScan pending_scan_;
  uint64_t scan_seq_ = 0;
  bool has_pending_scan_ = false;
  bool stop_worker_ = false;
  std::thread match_thread_;

  rclcpp::CallbackGroup::SharedPtr cb_group_odom_;
  rclcpp::CallbackGroup::SharedPtr cb_group_scan_;
  rclcpp::CallbackGroup::SharedPtr cb_group_viz_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_initialpose_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_scan_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_prior_map_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_active_submap_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_new_points_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_removed_points_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_aligned_scan_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_rolling_pose_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_fitness_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_rmse_mm_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_p2pl_med_mm_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_match_metrics_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_export_;
  rclcpp::TimerBase::SharedPtr timer_viz_;
};

}  // namespace rolling_map_updater

int main(int argc, char** argv) {
  // Bound OpenMP/BLAS thread pools so Open3D ICP does not starve FAST-LIO & RViz2 on 4-core CPUs
  setenv("OMP_NUM_THREADS", "2", 0);
  setenv("OPENBLAS_NUM_THREADS", "1", 0);

  rclcpp::init(argc, argv);
  auto node = std::make_shared<rolling_map_updater::RollingMapNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
