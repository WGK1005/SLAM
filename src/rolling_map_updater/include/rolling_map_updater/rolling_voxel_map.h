#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <open3d/Open3D.h>

namespace rolling_map_updater {

struct Int3Key {
  int x = 0;
  int y = 0;
  int z = 0;

  bool operator==(const Int3Key& other) const noexcept {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct Int3KeyHash {
  std::size_t operator()(const Int3Key& k) const noexcept {
    // 64-bit spatial hash combining primes
    std::size_t h1 = std::hash<int>{}(k.x);
    std::size_t h2 = std::hash<int>{}(k.y);
    std::size_t h3 = std::hash<int>{}(k.z);
    return h1 ^ (h2 * 0x9e3779b97f4a7c15ULL + (h1 << 6) + (h1 >> 2)) ^
           (h3 * 0x517cc1b727220a95ULL + (h2 << 5));
  }
};

struct GlobalVoxelKey {
  Int3Key chunk;
  Int3Key voxel;

  bool operator==(const GlobalVoxelKey& other) const noexcept {
    return chunk == other.chunk && voxel == other.voxel;
  }
};

struct GlobalVoxelKeyHash {
  std::size_t operator()(const GlobalVoxelKey& gk) const noexcept {
    Int3KeyHash kh;
    std::size_t h1 = kh(gk.chunk);
    std::size_t h2 = kh(gk.voxel);
    return h1 ^ (h2 + 0x9e3779b97f4a7c15ULL + (h1 << 6) + (h1 >> 2));
  }
};

enum class VoxelState : uint8_t {
  PRIOR_OLD = 0,    ///< Loaded from read-only prior .pcd map
  NEW_KEPT = 1,     ///< Newly observed static feature promoted and kept
  DELETED_OLD = 2,  ///< Outdated feature cleared via raycast visibility check
  CANDIDATE = 3     ///< Unconfirmed transient observation waiting for K hits
};

struct VoxelEntry {
  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
  bool has_normal = false;
  float log_odds = 2.0f;
  uint32_t hit_count = 0;
  uint32_t miss_count = 0;
  uint64_t last_frame_id = 0;
  VoxelState state = VoxelState::PRIOR_OLD;
};

struct RollingMapConfig {
  double chunk_size = 2.0;          ///< Chunk grid edge length (m)
  double voxel_size = 0.08;         ///< Map voxel resolution (m)
  double active_radius_xy = 12.0;   ///< Rolling window half-extent in X/Y (m)
  double active_z_down = 4.0;       ///< Rolling window extent below robot (m)
  double active_z_up = 6.0;         ///< Rolling window extent above robot (m)
  double roll_trigger_dist = 1.0;   ///< Robot displacement to trigger window roll (m)
  size_t min_roll_prior_voxels = 300;  ///< Refuse non-forced roll if window has fewer active voxels

  // Feature matching (Two-stage Coarse-to-Fine Point-to-Plane ICP with Huber loss & degeneracy guard)
  double scan_voxel_size = 0.08;
  double target_icp_voxel_size = 0.12;  ///< Downsampled ICP target voxel size for real-time speed
  size_t max_target_icp_pts = 24000;    ///< Hard cap on ICP target points to bound CPU spikes
  double icp_max_corr_dist = 0.25;
  double coarse_corr_dist = 1.00;
  double coarse_trigger_fitness = 0.62;
  double coarse_trigger_fitness_10cm = 0.56;
  double fine_corr_dist = 0.22;
  double icp_huber_k = 0.15;
  int icp_max_iterations = 25;
  double eval_corr_dist = 0.25;
  double degen_ratio_thresh = 0.08;     ///< Axis lambda/lambda_max below this gets damped toward pred
  double degen_ratio_floor = 0.015;     ///< Axis ratio below this receives zero translation weight in TRACKED mode
  double degen_min_eigenvalue = 25.0;   ///< Min normal outer-product eigenvalue for full weight
  double scan_match_color_dist = 0.16;  ///< Max distance (m) for Green(matched) vs Red(unmatched) scan points
  double error_eval_max_dist = 0.15;    ///< Max distance (m) for per-frame millimeter error statistics
  size_t kdtree_rebuild_dirty_thresh = 40;

  // Map update safety gating
  double min_update_fitness = 0.55;
  double max_update_rmse = 0.12;

  // Keep New parameters
  int min_hits_to_add = 3;
  double max_insert_range = 15.0;
  double new_point_min_dist = 0.14;     ///< Skip Keep-New within 14cm of existing surface (prevents voxel chatter)
  float log_odds_prior_init = 2.0f;
  float log_odds_hit = 0.85f;
  float log_odds_miss = 0.45f;
  float log_odds_min = -3.0f;
  float log_odds_max = 4.5f;
  float log_odds_add_thresh = 1.5f;
  float log_odds_delete_thresh = -0.3f;

  // Delete Old parameters (Spherical range-image raycasting with live proximity & beam-tube guards)
  double ray_min_range = 0.80;
  double ray_max_range = 8.0;
  double ray_clearance_m = 0.35;
  double live_keep_radius_m = 0.28;     ///< Never mark/delete a voxel if live scan hits within this radius
  double ray_tube_radius_m = 0.11;      ///< Max perpendicular distance from ray to voxel for clearing
  double ray_el_min_deg = -16.0;
  double ray_el_max_deg = 16.0;
  double ray_el_step_deg = 0.5;
  double ray_az_step_deg = 0.5;
  double min_ray_incidence_cos = 0.32;  ///< Reject grazing-angle deletion
};

struct AlignmentResult {
  bool success = false;
  bool used_coarse_stage = false;
  Eigen::Matrix4d T_map_body = Eigen::Matrix4d::Identity();
  double fitness_before = 0.0;
  double fitness_10cm_before = 0.0;
  double fitness = 0.0;
  double fitness_10cm = 0.0;
  double inlier_rmse = 0.0;
  double align_time_ms = 0.0;
  double min_eig_ratio = 1.0;
  int degen_axis_count = 0;

  // Pose correction applied relative to predicted pose
  double corr_dx_mm = 0.0;
  double corr_dy_mm = 0.0;
  double corr_dz_mm = 0.0;
  double corr_dist_mm = 0.0;
  double corr_dyaw_deg = 0.0;

  // Detailed millimeter-level surface error statistics (within error_eval_max_dist)
  size_t scan_eval_points = 0;
  size_t inlier_points_15cm = 0;
  double inlier_pct_5cm = 0.0;
  double inlier_pct_10cm = 0.0;
  double p2p_rmse_mm = 0.0;
  double p2p_mean_mm = 0.0;
  double p2p_median_mm = 0.0;
  double p2p_p95_mm = 0.0;
  double p2plane_rmse_mm = 0.0;
  double p2plane_mean_mm = 0.0;
  double p2plane_median_mm = 0.0;
  double p2plane_p95_mm = 0.0;
};

struct MapStats {
  size_t prior_initial_count = 0;
  size_t prior_active_count = 0;
  size_t new_kept_count = 0;
  size_t deleted_old_count = 0;
  size_t active_submap_points = 0;
  size_t active_chunk_count = 0;
  size_t total_chunk_count = 0;
  size_t roll_count = 0;
  double last_roll_ms = 0.0;
  double last_align_ms = 0.0;
  double last_update_ms = 0.0;
};

class RollingVoxelMap {
 public:
  explicit RollingVoxelMap(const RollingMapConfig& config = RollingMapConfig());

  /// Read-only load from .pcd/.ply file into in-memory chunk hash map.
  /// Never modifies or writes back to `file_path`.
  bool LoadPriorMapReadOnly(const std::string& file_path);

  /// Load from an in-memory Open3D point cloud (e.g., for testing).
  bool LoadPriorMapFromCloud(const open3d::geometry::PointCloud& cloud);

  /// Roll active window around `robot_pos` in map frame. Returns true if rolled.
  bool RollWindow(const Eigen::Vector3d& robot_pos, bool force = false);

  /// Rebuild the active Open3D target cloud if dirty (or when `force` is true).
  void RefreshActiveTargetCloud(bool force = false);

  /// Two-stage coarse-to-fine Point-to-Plane ICP against active rolling submap.
  /// If `eval_scan_body` is non-null, mm error metrics are evaluated on `*eval_scan_body`.
  AlignmentResult AlignScanPointToPlane(
      const open3d::geometry::PointCloud& scan_body,
      const Eigen::Matrix4d& T_map_body_pred,
      const open3d::geometry::PointCloud* eval_scan_body = nullptr);

  /// Multi-yaw local recovery search around `T_map_body_hint` when tracking was lost.
  bool TryRecoverLocalRelocalization(
      const open3d::geometry::PointCloud& scan_body,
      const Eigen::Matrix4d& T_map_body_hint,
      AlignmentResult& out_best,
      const open3d::geometry::PointCloud* eval_scan_body = nullptr);

  /// Compute exact per-point Euclidean & Point-to-Plane errors (mm) against active submap.
  void ComputeMillimeterMetrics(
      const open3d::geometry::PointCloud& scan_body,
      AlignmentResult& in_out_align) const;

  /// True if `pos_map` is within `max_xy_dist` (XY) and active Z span of any PRIOR_OLD chunk.
  bool HasPriorNearby(const Eigen::Vector3d& pos_map, double max_xy_dist) const;

  /// Apply Keep-New & Delete-Old online update using an aligned scan.
  /// Returns pair {added_or_updated_count, deleted_old_count} in this step.
  std::pair<size_t, size_t> UpdateKeepNewDeleteOld(
      const open3d::geometry::PointCloud& scan_body,
      const Eigen::Matrix4d& T_map_body_aligned,
      double fitness,
      double rmse);

  /// Extract point clouds for RViz visualization or read-only inspection.
  std::shared_ptr<open3d::geometry::PointCloud> GetActiveSubmapCloud() const;
  std::shared_ptr<open3d::geometry::PointCloud> GetNewKeptCloud() const;
  std::shared_ptr<open3d::geometry::PointCloud> GetDeletedOldCloud() const;
  std::shared_ptr<open3d::geometry::PointCloud> GetWorkingGlobalCloud() const;

  /// Colorize an aligned live scan in map frame:
  ///   - Green: point matches active rolling submap within `match_dist_thresh`
  ///   - Red: unmatched / outlier point (or alignment failed)
  std::shared_ptr<open3d::geometry::PointCloud> ColorizeAlignedScanByMatch(
      const open3d::geometry::PointCloud& aligned_scan_map,
      bool alignment_valid,
      double match_dist_thresh = -1.0);

  /// Export in-memory working map snapshot ONLY to a separate path (never original).
  bool ExportSnapshotToPath(const std::string& out_path) const;

  MapStats GetStats() const;
  const RollingMapConfig& config() const { return cfg_; }

 private:
  struct ChunkData {
    std::unordered_map<Int3Key, VoxelEntry, Int3KeyHash> voxels;
  };

  Int3Key PointToChunkKey(const Eigen::Vector3d& p) const noexcept;
  Int3Key PointToVoxelKey(const Eigen::Vector3d& p) const noexcept;

  void InsertPriorPoint(const Eigen::Vector3d& p,
                        const Eigen::Vector3d& n,
                        bool has_normal);

  void RebuildActiveTargetCloudLocked();
  void PopulateAlignmentDeltas(const Eigen::Matrix4d& T_pred,
                               AlignmentResult& in_out_align) const;
  void ComputeMillimeterMetricsWithTarget(
      const open3d::geometry::PointCloud& scan_body,
      const std::shared_ptr<open3d::geometry::PointCloud>& target_cloud,
      const std::shared_ptr<open3d::geometry::KDTreeFlann>& target_kdtree,
      AlignmentResult& in_out_align) const;

  RollingMapConfig cfg_;
  mutable std::mutex mutex_;

  std::unordered_map<Int3Key, ChunkData, Int3KeyHash> chunks_;
  std::unordered_set<Int3Key, Int3KeyHash> prior_chunks_;
  std::unordered_set<Int3Key, Int3KeyHash> active_chunks_;
  std::unordered_set<GlobalVoxelKey, GlobalVoxelKeyHash> new_kept_keys_;
  std::unordered_set<GlobalVoxelKey, GlobalVoxelKeyHash> deleted_old_keys_;
  std::shared_ptr<open3d::geometry::PointCloud> active_target_cloud_;
  std::shared_ptr<open3d::geometry::PointCloud> active_icp_cloud_;
  std::shared_ptr<open3d::geometry::KDTreeFlann> active_target_kdtree_;

  Eigen::Vector3d prior_min_bound_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d prior_max_bound_ = Eigen::Vector3d::Zero();
  bool has_prior_bounds_ = false;
  bool has_window_ = false;
  bool target_dirty_ = true;
  size_t pending_dirty_voxels_ = 0;
  Eigen::Vector3d last_roll_center_ = Eigen::Vector3d::Zero();
  uint64_t frame_counter_ = 0;

  // Bookkeeping stats
  MapStats stats_;
};

}  // namespace rolling_map_updater
