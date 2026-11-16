#include "rolling_map_updater/rolling_voxel_map.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <numeric>

#include <Eigen/Eigenvalues>
#include <open3d/pipelines/registration/Registration.h>
#include <open3d/pipelines/registration/RobustKernel.h>
#include <open3d/pipelines/registration/TransformationEstimation.h>

namespace rolling_map_updater {

namespace {
constexpr double kRadToDeg = 180.0 / M_PI;

struct OverlapMetrics {
  double fit_25 = 0.0;
  double fit_10 = 0.0;
  double rmse_25 = 0.0;
  size_t inliers_25 = 0;
};

OverlapMetrics EvaluateOverlapWithKDTree(
    const open3d::geometry::PointCloud& src,
    const open3d::geometry::KDTreeFlann& kdtree,
    const Eigen::Matrix4d& T,
    double eval_dist) {
  OverlapMetrics res;
  const size_t n = src.points_.size();
  if (n == 0) return res;

  const Eigen::Matrix3d R = T.block<3, 3>(0, 0);
  const Eigen::Vector3d t = T.block<3, 1>(0, 3);
  const double dist25_sq = eval_dist * eval_dist;
  const double dist10_sq = 0.10 * 0.10;

  std::vector<int> nn_idx(1);
  std::vector<double> nn_dist_sq(1);
  size_t cnt_10 = 0;
  double sum_sq = 0.0;

  for (size_t i = 0; i < n; ++i) {
    const Eigen::Vector3d p_w = R * src.points_[i] + t;
    if (kdtree.SearchKNN(p_w, 1, nn_idx, nn_dist_sq) > 0) {
      const double d_sq = nn_dist_sq[0];
      if (d_sq <= dist25_sq) {
        res.inliers_25++;
        sum_sq += d_sq;
      }
      if (d_sq <= dist10_sq) {
        cnt_10++;
      }
    }
  }

  const double inv_n = 1.0 / static_cast<double>(n);
  res.fit_25 = static_cast<double>(res.inliers_25) * inv_n;
  res.fit_10 = static_cast<double>(cnt_10) * inv_n;
  res.rmse_25 = (res.inliers_25 > 0)
                    ? std::sqrt(sum_sq / static_cast<double>(res.inliers_25))
                    : 0.0;
  return res;
}

Eigen::Matrix4d ApplyDegeneracyRegularization(
    const open3d::geometry::PointCloud& src,
    const open3d::geometry::PointCloud& target_cloud,
    const open3d::geometry::KDTreeFlann& target_kdtree,
    const Eigen::Matrix4d& T_pred,
    const Eigen::Matrix4d& T_icp,
    const RollingMapConfig& cfg,
    bool used_coarse_stage,
    double& out_min_eig_ratio,
    int& out_degen_axes) {
  out_min_eig_ratio = 1.0;
  out_degen_axes = 0;
  if (src.IsEmpty() || target_cloud.IsEmpty() || !target_cloud.HasNormals()) {
    return T_icp;
  }

  const Eigen::Matrix3d R_icp = T_icp.block<3, 3>(0, 0);
  const Eigen::Vector3d t_icp = T_icp.block<3, 1>(0, 3);
  const double max_corr_sq = cfg.error_eval_max_dist * cfg.error_eval_max_dist;

  Eigen::Matrix3d H_t = Eigen::Matrix3d::Zero();
  size_t valid_pairs = 0;
  std::vector<int> nn_idx(1);
  std::vector<double> nn_dist_sq(1);

  for (const auto& p_s : src.points_) {
    const Eigen::Vector3d p_w = R_icp * p_s + t_icp;
    if (target_kdtree.SearchKNN(p_w, 1, nn_idx, nn_dist_sq) > 0 &&
        nn_dist_sq[0] <= max_corr_sq) {
      const size_t q_idx = static_cast<size_t>(nn_idx[0]);
      if (q_idx < target_cloud.normals_.size()) {
        const Eigen::Vector3d& raw_n = target_cloud.normals_[q_idx];
        if (raw_n.allFinite() && raw_n.squaredNorm() > 0.25) {
          const Eigen::Vector3d n = raw_n.normalized();
          H_t.noalias() += n * n.transpose();
          valid_pairs++;
        }
      }
    }
  }

  if (valid_pairs < 20) {
    return T_icp;
  }

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(H_t);
  if (eig.info() != Eigen::Success) {
    return T_icp;
  }

  const Eigen::Vector3d evals = eig.eigenvalues();
  const Eigen::Matrix3d evecs = eig.eigenvectors();
  const double lam_max = std::max(evals(2), 1e-6);
  out_min_eig_ratio = std::max(0.0, evals(0)) / lam_max;

  const Eigen::Vector3d t_pred = T_pred.block<3, 1>(0, 3);
  const Eigen::Vector3d delta_t = t_icp - t_pred;
  Eigen::Vector3d reg_delta_t = Eigen::Vector3d::Zero();

  const double ratio_span = std::max(1e-4, cfg.degen_ratio_thresh - cfg.degen_ratio_floor);
  const double sup_span = std::max(1.0, cfg.degen_min_eigenvalue - 4.0);

  for (int k = 0; k < 3; ++k) {
    const double lam_k = std::max(0.0, evals(k));
    const double r_k = lam_k / lam_max;
    const double w_r = std::clamp((r_k - cfg.degen_ratio_floor) / ratio_span, 0.0, 1.0);
    const double w_s = std::clamp((lam_k - 4.0) / sup_span, 0.0, 1.0);
    double w_k = w_r * w_s;
    if (used_coarse_stage && r_k >= cfg.degen_ratio_floor && lam_k >= 10.0) {
      w_k = std::max(w_k, 0.35);
    }
    if (w_k < 0.99) {
      out_degen_axes++;
    }
    const Eigen::Vector3d v_k = evecs.col(k);
    reg_delta_t += w_k * delta_t.dot(v_k) * v_k;
  }

  Eigen::Matrix4d T_reg = T_icp;
  T_reg.block<3, 1>(0, 3) = t_pred + reg_delta_t;
  return T_reg;
}

}  // namespace

RollingVoxelMap::RollingVoxelMap(const RollingMapConfig& config)
    : cfg_(config),
      active_target_cloud_(std::make_shared<open3d::geometry::PointCloud>()) {}

Int3Key RollingVoxelMap::PointToChunkKey(const Eigen::Vector3d& p) const noexcept {
  const double inv = 1.0 / cfg_.chunk_size;
  return Int3Key{
      static_cast<int>(std::floor(p.x() * inv)),
      static_cast<int>(std::floor(p.y() * inv)),
      static_cast<int>(std::floor(p.z() * inv)),
  };
}

Int3Key RollingVoxelMap::PointToVoxelKey(const Eigen::Vector3d& p) const noexcept {
  const double inv = 1.0 / cfg_.voxel_size;
  return Int3Key{
      static_cast<int>(std::floor(p.x() * inv)),
      static_cast<int>(std::floor(p.y() * inv)),
      static_cast<int>(std::floor(p.z() * inv)),
  };
}

bool RollingVoxelMap::LoadPriorMapReadOnly(const std::string& file_path) {
  auto raw_cloud = std::make_shared<open3d::geometry::PointCloud>();
  if (!open3d::io::ReadPointCloud(file_path, *raw_cloud) || raw_cloud->IsEmpty()) {
    return false;
  }
  return LoadPriorMapFromCloud(*raw_cloud);
}

bool RollingVoxelMap::LoadPriorMapFromCloud(const open3d::geometry::PointCloud& cloud) {
  if (cloud.IsEmpty()) {
    return false;
  }

  auto down = cloud.VoxelDownSample(cfg_.voxel_size);
  if (!down || down->IsEmpty()) {
    return false;
  }

  // Note: PCD files with PointXYZINormal schema can report HasNormals()==true with all-zero vectors.
  size_t valid_normal_cnt = 0;
  if (down->HasNormals()) {
    for (const auto& n : down->normals_) {
      if (n.allFinite() && n.squaredNorm() > 0.25) {
        valid_normal_cnt++;
      }
    }
  }
  if (!down->HasNormals() || valid_normal_cnt * 2 < down->points_.size()) {
    down->EstimateNormals(
        open3d::geometry::KDTreeSearchParamHybrid(cfg_.voxel_size * 3.0, 25));
  }

  std::lock_guard<std::mutex> lock(mutex_);
  chunks_.clear();
  prior_chunks_.clear();
  active_chunks_.clear();
  new_kept_keys_.clear();
  deleted_old_keys_.clear();
  active_icp_cloud_.reset();
  prior_min_bound_ = Eigen::Vector3d::Zero();
  prior_max_bound_ = Eigen::Vector3d::Zero();
  has_prior_bounds_ = false;
  has_window_ = false;
  target_dirty_ = true;
  pending_dirty_voxels_ = 0;
  stats_ = MapStats{};

  const bool has_normals = down->HasNormals();
  for (size_t i = 0; i < down->points_.size(); ++i) {
    const Eigen::Vector3d& p = down->points_[i];
    Eigen::Vector3d n = Eigen::Vector3d::Zero();
    bool valid_n = false;
    if (has_normals && i < down->normals_.size()) {
      const Eigen::Vector3d& raw_n = down->normals_[i];
      if (raw_n.allFinite() && raw_n.squaredNorm() > 0.25) {
        n = raw_n.normalized();
        valid_n = true;
      }
    }
    InsertPriorPoint(p, n, valid_n);
  }

  stats_.total_chunk_count = chunks_.size();
  return true;
}

void RollingVoxelMap::InsertPriorPoint(const Eigen::Vector3d& p,
                                       const Eigen::Vector3d& n,
                                       bool has_normal) {
  const Int3Key ck = PointToChunkKey(p);
  const Int3Key vk = PointToVoxelKey(p);
  prior_chunks_.insert(ck);
  auto& chunk = chunks_[ck];
  auto it = chunk.voxels.find(vk);
  if (it == chunk.voxels.end()) {
    VoxelEntry entry;
    entry.mean = p;
    entry.normal = n;
    entry.has_normal = has_normal;
    entry.log_odds = cfg_.log_odds_prior_init;
    entry.hit_count = 1;
    entry.miss_count = 0;
    entry.state = VoxelState::PRIOR_OLD;
    chunk.voxels.emplace(vk, entry);
    stats_.prior_initial_count++;
    stats_.prior_active_count++;
    if (!has_prior_bounds_) {
      prior_min_bound_ = p;
      prior_max_bound_ = p;
      has_prior_bounds_ = true;
    } else {
      prior_min_bound_ = prior_min_bound_.cwiseMin(p);
      prior_max_bound_ = prior_max_bound_.cwiseMax(p);
    }
  }
}

bool RollingVoxelMap::HasPriorNearby(const Eigen::Vector3d& pos_map,
                                     double max_xy_dist) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (prior_chunks_.empty()) {
    return false;
  }
  if (has_prior_bounds_) {
    const double win_z_top = pos_map.z() + cfg_.active_z_up + 1.5;
    const double win_z_bot = pos_map.z() - cfg_.active_z_down - 1.5;
    if (win_z_top < prior_min_bound_.z() || win_z_bot > prior_max_bound_.z()) {
      return false;
    }
  }
  const double half_c = 0.5 * cfg_.chunk_size;
  const double thresh = max_xy_dist + cfg_.chunk_size * 0.75;
  const double thresh_sq = thresh * thresh;
  const double z_min_allow = pos_map.z() - cfg_.active_z_down - half_c - 1.5;
  const double z_max_allow = pos_map.z() + cfg_.active_z_up + half_c + 1.5;
  for (const auto& ck : prior_chunks_) {
    const double cz = static_cast<double>(ck.z) * cfg_.chunk_size + half_c;
    if (cz < z_min_allow || cz > z_max_allow) {
      continue;
    }
    const double cx = (static_cast<double>(ck.x) * cfg_.chunk_size + half_c) - pos_map.x();
    const double cy = (static_cast<double>(ck.y) * cfg_.chunk_size + half_c) - pos_map.y();
    if ((cx * cx + cy * cy) <= thresh_sq) {
      return true;
    }
  }
  return false;
}

bool RollingVoxelMap::RollWindow(const Eigen::Vector3d& robot_pos, bool force) {
  auto t0 = std::chrono::high_resolution_clock::now();
  std::lock_guard<std::mutex> lock(mutex_);

  if (!force && has_window_) {
    if ((robot_pos - last_roll_center_).norm() < cfg_.roll_trigger_dist) {
      return false;
    }
  }

  const double inv_c = 1.0 / cfg_.chunk_size;
  const int min_cx = static_cast<int>(std::floor((robot_pos.x() - cfg_.active_radius_xy) * inv_c));
  const int max_cx = static_cast<int>(std::floor((robot_pos.x() + cfg_.active_radius_xy) * inv_c));
  const int min_cy = static_cast<int>(std::floor((robot_pos.y() - cfg_.active_radius_xy) * inv_c));
  const int max_cy = static_cast<int>(std::floor((robot_pos.y() + cfg_.active_radius_xy) * inv_c));
  const int min_cz = static_cast<int>(std::floor((robot_pos.z() - cfg_.active_z_down) * inv_c));
  const int max_cz = static_cast<int>(std::floor((robot_pos.z() + cfg_.active_z_up) * inv_c));

  std::unordered_set<Int3Key, Int3KeyHash> next_active;
  size_t next_valid_voxels = 0;
  for (int cx = min_cx; cx <= max_cx; ++cx) {
    for (int cy = min_cy; cy <= max_cy; ++cy) {
      for (int cz = min_cz; cz <= max_cz; ++cz) {
        Int3Key ck{cx, cy, cz};
        auto it_c = chunks_.find(ck);
        if (it_c != chunks_.end()) {
          next_active.insert(ck);
          for (const auto& kv : it_c->second.voxels) {
            if (kv.second.state == VoxelState::PRIOR_OLD ||
                kv.second.state == VoxelState::NEW_KEPT) {
              next_valid_voxels++;
            }
          }
        }
      }
    }
  }

  if (!force) {
    const size_t min_floor = std::min<size_t>(
        cfg_.min_roll_prior_voxels,
        std::max<size_t>(30, stats_.prior_initial_count / 10));
    if (next_valid_voxels < min_floor) {
      return false;
    }
    if (has_window_ && stats_.active_submap_points >= 1000 &&
        next_valid_voxels * 5 < stats_.active_submap_points) {
      return false;
    }
  }

  active_chunks_ = std::move(next_active);
  last_roll_center_ = robot_pos;
  has_window_ = true;
  stats_.active_chunk_count = active_chunks_.size();
  stats_.total_chunk_count = chunks_.size();
  stats_.roll_count++;

  RebuildActiveTargetCloudLocked();
  target_dirty_ = false;

  auto t1 = std::chrono::high_resolution_clock::now();
  stats_.last_roll_ms =
      std::chrono::duration<double, std::milli>(t1 - t0).count();
  return true;
}

void RollingVoxelMap::RebuildActiveTargetCloudLocked() {
  auto next_cloud = std::make_shared<open3d::geometry::PointCloud>();
  size_t estimated_pts = active_chunks_.size() * 64;
  next_cloud->points_.reserve(estimated_pts);
  next_cloud->normals_.reserve(estimated_pts);

  std::vector<std::pair<size_t, VoxelEntry*>> missing_entries;
  for (const auto& ck : active_chunks_) {
    auto it_c = chunks_.find(ck);
    if (it_c == chunks_.end()) continue;
    for (auto& kv : it_c->second.voxels) {
      VoxelEntry& v = kv.second;
      if (v.state == VoxelState::PRIOR_OLD || v.state == VoxelState::NEW_KEPT) {
        const size_t idx = next_cloud->points_.size();
        next_cloud->points_.push_back(v.mean);
        next_cloud->normals_.push_back(v.normal);
        if (!v.has_normal) {
          missing_entries.emplace_back(idx, &v);
        }
      }
    }
  }

  active_target_cloud_ = std::move(next_cloud);
  if (active_target_cloud_ && !active_target_cloud_->IsEmpty()) {
    active_target_kdtree_ =
        std::make_shared<open3d::geometry::KDTreeFlann>(*active_target_cloud_);

    // Incremental O(K_new * log N) PCA normal estimation ONLY for new voxels lacking normals
    if (!missing_entries.empty() && active_target_cloud_->points_.size() >= 6) {
      std::vector<int> nn_idx;
      std::vector<double> nn_dist_sq;
      const double radius = cfg_.voxel_size * 3.0;
      for (const auto& item : missing_entries) {
        const size_t pt_idx = item.first;
        VoxelEntry* v_ptr = item.second;
        const Eigen::Vector3d& p = active_target_cloud_->points_[pt_idx];
        int found = active_target_kdtree_->SearchRadius(p, radius, nn_idx, nn_dist_sq);
        if (found < 6) {
          found = active_target_kdtree_->SearchKNN(p, 12, nn_idx, nn_dist_sq);
        }
        if (found >= 5) {
          Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
          for (int k = 0; k < found; ++k) {
            centroid += active_target_cloud_->points_[static_cast<size_t>(nn_idx[k])];
          }
          centroid /= static_cast<double>(found);
          Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
          for (int k = 0; k < found; ++k) {
            const Eigen::Vector3d d =
                active_target_cloud_->points_[static_cast<size_t>(nn_idx[k])] - centroid;
            cov.noalias() += d * d.transpose();
          }
          Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(cov);
          if (eig.info() == Eigen::Success) {
            Eigen::Vector3d n = eig.eigenvectors().col(0);
            if (n.norm() > 1e-6) {
              n.normalize();
              v_ptr->normal = n;
              v_ptr->has_normal = true;
              active_target_cloud_->normals_[pt_idx] = n;
            }
          }
        }
      }
    }
  } else {
    active_target_kdtree_.reset();
  }

  // Build lightweight ICP target cloud (e.g. 0.12m voxel & capped point budget)
  // while keeping active_target_cloud_/kdtree_ at map resolution for exact mm evaluation.
  if (active_target_cloud_ && !active_target_cloud_->IsEmpty()) {
    std::shared_ptr<open3d::geometry::PointCloud> icp_cloud = active_target_cloud_;
    if (cfg_.target_icp_voxel_size > cfg_.voxel_size + 1e-4 &&
        active_target_cloud_->points_.size() > 400) {
      auto ds = active_target_cloud_->VoxelDownSample(cfg_.target_icp_voxel_size);
      if (ds && ds->points_.size() >= 100 && ds->HasNormals()) {
        auto cleaned = std::make_shared<open3d::geometry::PointCloud>();
        cleaned->points_.reserve(ds->points_.size());
        cleaned->normals_.reserve(ds->points_.size());
        for (size_t i = 0; i < ds->points_.size(); ++i) {
          const Eigen::Vector3d& n = ds->normals_[i];
          const double sn = n.squaredNorm();
          if (n.allFinite() && sn > 0.20) {
            cleaned->points_.push_back(ds->points_[i]);
            cleaned->normals_.push_back(n / std::sqrt(sn));
          }
        }
        if (cleaned->points_.size() >= 100) {
          icp_cloud = std::move(cleaned);
        }
      }
    }
    if (cfg_.max_target_icp_pts > 0 && icp_cloud->points_.size() > cfg_.max_target_icp_pts) {
      auto capped = std::make_shared<open3d::geometry::PointCloud>();
      capped->points_.reserve(cfg_.max_target_icp_pts);
      const bool has_n = icp_cloud->HasNormals();
      if (has_n) capped->normals_.reserve(cfg_.max_target_icp_pts);
      const double step =
          static_cast<double>(icp_cloud->points_.size()) / static_cast<double>(cfg_.max_target_icp_pts);
      for (size_t k = 0; k < cfg_.max_target_icp_pts; ++k) {
        const size_t idx = std::min(icp_cloud->points_.size() - 1,
                                    static_cast<size_t>(static_cast<double>(k) * step));
        capped->points_.push_back(icp_cloud->points_[idx]);
        if (has_n) capped->normals_.push_back(icp_cloud->normals_[idx]);
      }
      icp_cloud = std::move(capped);
    }
    active_icp_cloud_ = std::move(icp_cloud);
  } else {
    active_icp_cloud_.reset();
  }

  stats_.active_submap_points = active_target_cloud_->points_.size();
  target_dirty_ = false;
  pending_dirty_voxels_ = 0;
}

void RollingVoxelMap::RefreshActiveTargetCloud(bool force) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (force || (target_dirty_ && pending_dirty_voxels_ >= cfg_.kdtree_rebuild_dirty_thresh) ||
      !active_target_cloud_ || !active_target_kdtree_) {
    RebuildActiveTargetCloudLocked();
  }
}

void RollingVoxelMap::PopulateAlignmentDeltas(const Eigen::Matrix4d& T_pred,
                                              AlignmentResult& in_out_align) const {
  const Eigen::Vector3d dpos =
      in_out_align.T_map_body.block<3, 1>(0, 3) - T_pred.block<3, 1>(0, 3);
  in_out_align.corr_dx_mm = dpos.x() * 1000.0;
  in_out_align.corr_dy_mm = dpos.y() * 1000.0;
  in_out_align.corr_dz_mm = dpos.z() * 1000.0;
  in_out_align.corr_dist_mm = dpos.norm() * 1000.0;

  const Eigen::Matrix3d dR =
      T_pred.block<3, 3>(0, 0).transpose() * in_out_align.T_map_body.block<3, 3>(0, 0);
  in_out_align.corr_dyaw_deg = std::atan2(dR(1, 0), dR(0, 0)) * kRadToDeg;
}

void RollingVoxelMap::ComputeMillimeterMetricsWithTarget(
    const open3d::geometry::PointCloud& scan_body,
    const std::shared_ptr<open3d::geometry::PointCloud>& target_cloud,
    const std::shared_ptr<open3d::geometry::KDTreeFlann>& target_kdtree,
    AlignmentResult& in_out_align) const {
  if (scan_body.IsEmpty() || !target_cloud || !target_kdtree || target_cloud->IsEmpty()) {
    return;
  }

  open3d::geometry::PointCloud eval_cloud;
  if (scan_body.points_.size() > 4000) {
    auto down = scan_body.VoxelDownSample(cfg_.scan_voxel_size);
    if (down && !down->IsEmpty()) {
      eval_cloud = *down;
    } else {
      eval_cloud = scan_body;
    }
  } else {
    eval_cloud = scan_body;
  }
  eval_cloud.Transform(in_out_align.T_map_body);

  const size_t n = eval_cloud.points_.size();
  in_out_align.scan_eval_points = n;
  if (n == 0) return;

  const double max_eval_sq = cfg_.error_eval_max_dist * cfg_.error_eval_max_dist;
  const bool has_normals = target_cloud->HasNormals();

  std::vector<double> p2p_mm;
  std::vector<double> p2plane_mm;
  p2p_mm.reserve(n);
  p2plane_mm.reserve(n);

  size_t cnt_5cm = 0;
  size_t cnt_10cm = 0;
  double sum_p2p_sq = 0.0;
  double sum_p2p = 0.0;
  double sum_p2pl_sq = 0.0;
  double sum_p2pl = 0.0;

  std::vector<int> nn_idx(1);
  std::vector<double> nn_dist_sq(1);
  for (size_t i = 0; i < n; ++i) {
    const Eigen::Vector3d& p = eval_cloud.points_[i];
    if (target_kdtree->SearchKNN(p, 1, nn_idx, nn_dist_sq) <= 0) {
      continue;
    }
    const double d_sq = nn_dist_sq[0];
    const double d_m = std::sqrt(d_sq);
    if (d_m <= 0.05) cnt_5cm++;
    if (d_m <= 0.10) cnt_10cm++;
    if (d_sq <= max_eval_sq) {
      const size_t q_idx = static_cast<size_t>(nn_idx[0]);
      const double e_mm = d_m * 1000.0;
      double pl_m = d_m;
      if (has_normals && q_idx < target_cloud->normals_.size()) {
        const Eigen::Vector3d& q = target_cloud->points_[q_idx];
        const Eigen::Vector3d& nrm = target_cloud->normals_[q_idx];
        if (nrm.allFinite() && nrm.squaredNorm() > 0.25) {
          pl_m = std::abs((p - q).dot(nrm.normalized()));
        }
      }
      const double pl_mm = pl_m * 1000.0;
      p2p_mm.push_back(e_mm);
      p2plane_mm.push_back(pl_mm);
      sum_p2p += e_mm;
      sum_p2p_sq += e_mm * e_mm;
      sum_p2pl += pl_mm;
      sum_p2pl_sq += pl_mm * pl_mm;
    }
  }

  const double inv_n = 100.0 / static_cast<double>(n);
  in_out_align.inlier_pct_5cm = static_cast<double>(cnt_5cm) * inv_n;
  in_out_align.inlier_pct_10cm = static_cast<double>(cnt_10cm) * inv_n;
  in_out_align.inlier_points_15cm = p2p_mm.size();

  if (!p2p_mm.empty()) {
    const double m = static_cast<double>(p2p_mm.size());
    in_out_align.p2p_mean_mm = sum_p2p / m;
    in_out_align.p2p_rmse_mm = std::sqrt(sum_p2p_sq / m);
    in_out_align.p2plane_mean_mm = sum_p2pl / m;
    in_out_align.p2plane_rmse_mm = std::sqrt(sum_p2pl_sq / m);

    const size_t med_idx = p2p_mm.size() / 2;
    const size_t p95_idx = std::min(p2p_mm.size() - 1, (p2p_mm.size() * 95) / 100);
    std::nth_element(p2p_mm.begin(), p2p_mm.begin() + static_cast<ptrdiff_t>(med_idx), p2p_mm.end());
    in_out_align.p2p_median_mm = p2p_mm[med_idx];
    std::nth_element(p2p_mm.begin(), p2p_mm.begin() + static_cast<ptrdiff_t>(p95_idx), p2p_mm.end());
    in_out_align.p2p_p95_mm = p2p_mm[p95_idx];

    std::nth_element(p2plane_mm.begin(), p2plane_mm.begin() + static_cast<ptrdiff_t>(med_idx), p2plane_mm.end());
    in_out_align.p2plane_median_mm = p2plane_mm[med_idx];
    std::nth_element(p2plane_mm.begin(), p2plane_mm.begin() + static_cast<ptrdiff_t>(p95_idx), p2plane_mm.end());
    in_out_align.p2plane_p95_mm = p2plane_mm[p95_idx];
  }
}

void RollingVoxelMap::ComputeMillimeterMetrics(
    const open3d::geometry::PointCloud& scan_body,
    AlignmentResult& in_out_align) const {
  std::shared_ptr<open3d::geometry::PointCloud> target_copy;
  std::shared_ptr<open3d::geometry::KDTreeFlann> kdtree_copy;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    target_copy = active_target_cloud_;
    kdtree_copy = active_target_kdtree_;
  }
  ComputeMillimeterMetricsWithTarget(scan_body, target_copy, kdtree_copy, in_out_align);
}

AlignmentResult RollingVoxelMap::AlignScanPointToPlane(
    const open3d::geometry::PointCloud& scan_body,
    const Eigen::Matrix4d& T_map_body_pred,
    const open3d::geometry::PointCloud* eval_scan_body) {
  AlignmentResult out;
  out.T_map_body = T_map_body_pred;
  auto t0 = std::chrono::high_resolution_clock::now();

  if (scan_body.IsEmpty()) {
    return out;
  }

  std::shared_ptr<open3d::geometry::PointCloud> target_copy;
  std::shared_ptr<open3d::geometry::PointCloud> target_icp_copy;
  std::shared_ptr<open3d::geometry::KDTreeFlann> kdtree_copy;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!active_target_cloud_ || !active_target_kdtree_ ||
        (target_dirty_ && pending_dirty_voxels_ >= cfg_.kdtree_rebuild_dirty_thresh)) {
      RebuildActiveTargetCloudLocked();
    }
    if (!active_target_cloud_ || active_target_cloud_->points_.size() < 30 ||
        !active_target_cloud_->HasNormals()) {
      return out;
    }
    target_copy = active_target_cloud_;
    kdtree_copy = active_target_kdtree_;
    target_icp_copy =
        (active_icp_cloud_ && active_icp_cloud_->points_.size() >= 30 &&
         active_icp_cloud_->HasNormals())
            ? active_icp_cloud_
            : active_target_cloud_;
  }

  auto src_down = scan_body.VoxelDownSample(cfg_.scan_voxel_size);
  if (!src_down || src_down->points_.size() < 20) {
    return out;
  }

  const OverlapMetrics ov_before =
      EvaluateOverlapWithKDTree(*src_down, *kdtree_copy, T_map_body_pred, cfg_.eval_corr_dist);
  out.fitness_before = ov_before.fit_25;
  out.fitness_10cm_before = ov_before.fit_10;

  auto loss = std::make_shared<open3d::pipelines::registration::HuberLoss>(cfg_.icp_huber_k);
  auto estimation =
      open3d::pipelines::registration::TransformationEstimationPointToPlane(loss);

  Eigen::Matrix4d T_curr = T_map_body_pred;
  double fine_dist = cfg_.icp_max_corr_dist;

  // Stage 1: Coarse pullback if initial prediction fit dropped (validated on both 25cm and 10cm)
  if ((ov_before.fit_25 < cfg_.coarse_trigger_fitness ||
       ov_before.fit_10 < cfg_.coarse_trigger_fitness_10cm) &&
      cfg_.coarse_corr_dist > cfg_.icp_max_corr_dist) {
    auto coarse_criteria = open3d::pipelines::registration::ICPConvergenceCriteria(
        1e-5, 1e-5, 18);
    auto coarse_res = open3d::pipelines::registration::RegistrationICP(
        *src_down, *target_icp_copy, cfg_.coarse_corr_dist, T_curr, estimation, coarse_criteria);
    const OverlapMetrics ov_coarse = EvaluateOverlapWithKDTree(
        *src_down, *kdtree_copy, coarse_res.transformation_, cfg_.eval_corr_dist);
    const bool coarse_helped =
        (ov_coarse.fit_10 >= ov_before.fit_10 + 0.03 && ov_coarse.fit_25 + 0.02 >= ov_before.fit_25) ||
        (ov_before.fit_25 < 0.45 && ov_coarse.fit_25 >= ov_before.fit_25 + 0.08);
    if (coarse_helped) {
      T_curr = coarse_res.transformation_;
      out.used_coarse_stage = true;
      fine_dist = cfg_.fine_corr_dist;
    }
  }

  // Stage 2: Fine Point-to-Plane polish
  auto fine_criteria = open3d::pipelines::registration::ICPConvergenceCriteria(
      1e-6, 1e-6, cfg_.icp_max_iterations);
  auto reg_res = open3d::pipelines::registration::RegistrationICP(
      *src_down, *target_icp_copy, fine_dist, T_curr, estimation, fine_criteria);

  Eigen::Matrix4d T_icp = reg_res.transformation_;
  const OverlapMetrics ov_icp =
      EvaluateOverlapWithKDTree(*src_down, *kdtree_copy, T_icp, cfg_.eval_corr_dist);

  // Degeneracy-aware projection: damp weakly constrained corridor/stair axes back toward T_pred
  Eigen::Matrix4d T_final = ApplyDegeneracyRegularization(
      *src_down, *target_copy, *kdtree_copy, T_map_body_pred, T_icp, cfg_,
      out.used_coarse_stage, out.min_eig_ratio, out.degen_axis_count);

  OverlapMetrics ov_final = ov_icp;
  if (out.degen_axis_count > 0) {
    const OverlapMetrics ov_reg =
        EvaluateOverlapWithKDTree(*src_down, *kdtree_copy, T_final, cfg_.eval_corr_dist);
    // If projection didn't harm a well-conditioned fit, adopt regularized pose; otherwise keep ICP if non-degen
    if (ov_reg.fit_10 + 0.06 >= ov_icp.fit_10 || out.min_eig_ratio < 0.03) {
      ov_final = ov_reg;
    } else {
      T_final = T_icp;
    }
  }

  // Guard: never accept an ICP step that substantially degrades overlap vs prediction
  if (ov_before.fit_25 >= 0.45 &&
      (ov_final.fit_25 + 0.06 < ov_before.fit_25 ||
       ov_final.fit_10 + 0.08 < ov_before.fit_10)) {
    T_final = T_map_body_pred;
    ov_final = ov_before;
    out.used_coarse_stage = false;
  }

  const open3d::geometry::PointCloud& metric_src =
      (eval_scan_body && !eval_scan_body->IsEmpty()) ? *eval_scan_body : *src_down;

  out.T_map_body = T_final;
  out.fitness = ov_final.fit_25;
  out.fitness_10cm = ov_final.fit_10;
  out.inlier_rmse = ov_final.rmse_25;
  out.success = (out.fitness > 0.1 && std::isfinite(out.inlier_rmse));
  PopulateAlignmentDeltas(T_map_body_pred, out);
  ComputeMillimeterMetricsWithTarget(metric_src, target_copy, kdtree_copy, out);

  auto t1 = std::chrono::high_resolution_clock::now();
  out.align_time_ms =
      std::chrono::duration<double, std::milli>(t1 - t0).count();

  {
    std::lock_guard<std::mutex> lock(mutex_);
    stats_.last_align_ms = out.align_time_ms;
  }
  return out;
}

bool RollingVoxelMap::TryRecoverLocalRelocalization(
    const open3d::geometry::PointCloud& scan_body,
    const Eigen::Matrix4d& T_map_body_hint,
    AlignmentResult& out_best,
    const open3d::geometry::PointCloud* eval_scan_body) {
  if (scan_body.IsEmpty()) return false;

  std::shared_ptr<open3d::geometry::PointCloud> target_copy;
  std::shared_ptr<open3d::geometry::PointCloud> target_icp_copy;
  std::shared_ptr<open3d::geometry::KDTreeFlann> kdtree_copy;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!active_target_cloud_ || !active_target_kdtree_) {
      RebuildActiveTargetCloudLocked();
    }
    if (!active_target_cloud_ || active_target_cloud_->points_.size() < 30 ||
        !active_target_cloud_->HasNormals()) {
      return false;
    }
    target_copy = active_target_cloud_;
    kdtree_copy = active_target_kdtree_;
    target_icp_copy =
        (active_icp_cloud_ && active_icp_cloud_->points_.size() >= 30 &&
         active_icp_cloud_->HasNormals())
            ? active_icp_cloud_
            : active_target_cloud_;
  }

  auto coarse_src = scan_body.VoxelDownSample(std::max(0.12, cfg_.scan_voxel_size));
  auto fine_src = scan_body.VoxelDownSample(cfg_.scan_voxel_size);
  if (!coarse_src || coarse_src->points_.size() < 20 || !fine_src) return false;

  auto t0 = std::chrono::high_resolution_clock::now();
  const OverlapMetrics ov_hint =
      EvaluateOverlapWithKDTree(*fine_src, *kdtree_copy, T_map_body_hint, cfg_.eval_corr_dist);
  auto loss = std::make_shared<open3d::pipelines::registration::HuberLoss>(cfg_.icp_huber_k);
  auto estimation =
      open3d::pipelines::registration::TransformationEstimationPointToPlane(loss);
  auto coarse_crit = open3d::pipelines::registration::ICPConvergenceCriteria(1e-4, 1e-4, 14);

  struct RelocSeed {
    double dyaw_deg;
    double dx_local;
  };
  const RelocSeed seeds[] = {
      {0.0, 0.0},   {0.0, 0.45},  {0.0, -0.45}, {15.0, 0.0}, {-15.0, 0.0},
      {30.0, 0.0},  {-30.0, 0.0}, {50.0, 0.0},  {-50.0, 0.0}, {90.0, 0.0},
      {-90.0, 0.0}, {180.0, 0.0}};

  double best_fit = 0.0;
  double best_fit10 = 0.0;
  double best_rmse = 1e9;
  Eigen::Matrix4d best_T = T_map_body_hint;

  for (const auto& seed : seeds) {
    const double rad = seed.dyaw_deg * M_PI / 180.0;
    Eigen::Matrix3d Rz = Eigen::AngleAxisd(rad, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    Eigen::Matrix4d T_init = T_map_body_hint;
    T_init.block<3, 3>(0, 0) = Rz * T_map_body_hint.block<3, 3>(0, 0);
    T_init.block<3, 1>(0, 3) += T_init.block<3, 3>(0, 0) * Eigen::Vector3d(seed.dx_local, 0.0, 0.0);

    auto res = open3d::pipelines::registration::RegistrationICP(
        *coarse_src, *target_icp_copy, 1.50, T_init, estimation, coarse_crit);
    const OverlapMetrics ev = EvaluateOverlapWithKDTree(
        *coarse_src, *kdtree_copy, res.transformation_, cfg_.eval_corr_dist);
    if (ev.fit_10 > best_fit10 + 0.02 ||
        (std::abs(ev.fit_10 - best_fit10) <= 0.02 &&
         (ev.fit_25 > best_fit + 0.02 ||
          (std::abs(ev.fit_25 - best_fit) <= 0.02 && ev.rmse_25 < best_rmse)))) {
      best_fit = ev.fit_25;
      best_fit10 = ev.fit_10;
      best_rmse = ev.rmse_25;
      best_T = res.transformation_;
    }
  }

  if (best_fit < 0.50) {
    return false;
  }

  auto fine_crit = open3d::pipelines::registration::ICPConvergenceCriteria(
      1e-6, 1e-6, cfg_.icp_max_iterations);
  auto fine_res = open3d::pipelines::registration::RegistrationICP(
      *fine_src, *target_icp_copy, cfg_.fine_corr_dist, best_T, estimation, fine_crit);
  const OverlapMetrics ev_final = EvaluateOverlapWithKDTree(
      *fine_src, *kdtree_copy, fine_res.transformation_, cfg_.eval_corr_dist);
  if (ev_final.fit_25 < 0.58 || ev_final.fit_10 < 0.50) {
    return false;
  }

  out_best = AlignmentResult{};
  out_best.success = true;
  out_best.used_coarse_stage = true;
  out_best.T_map_body = fine_res.transformation_;
  out_best.fitness_before = ov_hint.fit_25;
  out_best.fitness_10cm_before = ov_hint.fit_10;
  out_best.fitness = ev_final.fit_25;
  out_best.fitness_10cm = ev_final.fit_10;
  out_best.inlier_rmse = ev_final.rmse_25;
  PopulateAlignmentDeltas(T_map_body_hint, out_best);
  const open3d::geometry::PointCloud& metric_src =
      (eval_scan_body && !eval_scan_body->IsEmpty()) ? *eval_scan_body : *fine_src;
  ComputeMillimeterMetricsWithTarget(metric_src, target_copy, kdtree_copy, out_best);

  auto t1 = std::chrono::high_resolution_clock::now();
  out_best.align_time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
  return true;
}

std::pair<size_t, size_t> RollingVoxelMap::UpdateKeepNewDeleteOld(
    const open3d::geometry::PointCloud& scan_body,
    const Eigen::Matrix4d& T_map_body_aligned,
    double fitness,
    double rmse) {
  auto t0 = std::chrono::high_resolution_clock::now();

  if (scan_body.IsEmpty()) {
    return {0, 0};
  }
  // Safety gate: never corrupt the map if localization is uncertain
  if (fitness < cfg_.min_update_fitness || rmse > cfg_.max_update_rmse) {
    return {0, 0};
  }

  // Build fine spherical range grid in sensor/body frame + KDTree for live-hit protection
  const int n_el = std::max(1, static_cast<int>(std::ceil(
      (cfg_.ray_el_max_deg - cfg_.ray_el_min_deg) / cfg_.ray_el_step_deg)));
  const int n_az = std::max(1, static_cast<int>(std::ceil(360.0 / cfg_.ray_az_step_deg)));
  struct RayCell {
    double min_r = std::numeric_limits<double>::infinity();
    Eigen::Vector3d dir = Eigen::Vector3d::Zero();
  };
  std::vector<RayCell> ray_grid(static_cast<size_t>(n_el * n_az));

  for (const auto& p_s : scan_body.points_) {
    const double r = p_s.norm();
    if (r < cfg_.ray_min_range || r > cfg_.ray_max_range * 2.0) continue;
    const double el_deg = std::asin(std::clamp(p_s.z() / r, -1.0, 1.0)) * kRadToDeg;
    if (el_deg < cfg_.ray_el_min_deg || el_deg >= cfg_.ray_el_max_deg) continue;
    double az_deg = std::atan2(p_s.y(), p_s.x()) * kRadToDeg + 180.0;
    if (az_deg < 0.0) az_deg += 360.0;
    if (az_deg >= 360.0) az_deg -= 360.0;

    const int eb = std::clamp(
        static_cast<int>(std::floor((el_deg - cfg_.ray_el_min_deg) / cfg_.ray_el_step_deg)),
        0, n_el - 1);
    const int ab = std::clamp(
        static_cast<int>(std::floor(az_deg / cfg_.ray_az_step_deg)),
        0, n_az - 1);
    RayCell& cell = ray_grid[static_cast<size_t>(eb * n_az + ab)];
    if (r < cell.min_r) {
      cell.min_r = r;
      cell.dir = p_s / r;
    }
  }

  // Prepare downsampled scan points in body & world frame
  auto scan_down = scan_body.VoxelDownSample(cfg_.voxel_size);
  std::unique_ptr<open3d::geometry::KDTreeFlann> scan_down_kdtree;
  if (scan_down && !scan_down->IsEmpty()) {
    scan_down_kdtree = std::make_unique<open3d::geometry::KDTreeFlann>(*scan_down);
  }
  const Eigen::Matrix4d T_body_map = T_map_body_aligned.inverse();
  const Eigen::Matrix3d R_body_map = T_body_map.block<3, 3>(0, 0);
  const Eigen::Vector3d t_body_map = T_body_map.block<3, 1>(0, 3);
  const Eigen::Vector3d sensor_pos_map = T_map_body_aligned.block<3, 1>(0, 3);

  std::lock_guard<std::mutex> lock(mutex_);
  frame_counter_++;
  size_t newly_promoted = 0;
  size_t newly_deleted = 0;
  bool dirty = false;

  // --- Phase A: Delete Old (Live-Hit Protected + Narrow Beam-Tube Ray-Pass Check) ---
  const double keep_r_sq = cfg_.live_keep_radius_m * cfg_.live_keep_radius_m;
  const double tube_r_sq = cfg_.ray_tube_radius_m * cfg_.ray_tube_radius_m;
  std::vector<int> live_nn_idx(1);
  std::vector<double> live_nn_dist_sq(1);

  for (const auto& ck : active_chunks_) {
    auto it_c = chunks_.find(ck);
    if (it_c == chunks_.end()) continue;

    for (auto& kv : it_c->second.voxels) {
      VoxelEntry& v = kv.second;
      if (v.state != VoxelState::PRIOR_OLD && v.state != VoxelState::NEW_KEPT) {
        continue;
      }

      const Eigen::Vector3d p_s = R_body_map * v.mean + t_body_map;
      const double r_v = p_s.norm();
      if (r_v < cfg_.ray_min_range || r_v > cfg_.ray_max_range) {
        continue;
      }

      // Live-scan proximity protection: if live scan returned within live_keep_radius_m,
      // the surface is still physically present — reinforce instead of deleting.
      if (scan_down_kdtree &&
          scan_down_kdtree->SearchKNN(p_s, 1, live_nn_idx, live_nn_dist_sq) > 0 &&
          live_nn_dist_sq[0] <= keep_r_sq) {
        v.miss_count = 0;
        v.log_odds = std::min(cfg_.log_odds_max, v.log_odds + 0.35f * cfg_.log_odds_hit);
        continue;
      }

      // Grazing-angle guard: skip if ray hits surface nearly tangentially
      if (v.has_normal) {
        const Eigen::Vector3d ray_dir = p_s / r_v;
        const Eigen::Vector3d n_s = R_body_map * v.normal;
        if (std::abs(ray_dir.dot(n_s)) < cfg_.min_ray_incidence_cos) {
          continue;
        }
      }

      const double el_deg = std::asin(std::clamp(p_s.z() / r_v, -1.0, 1.0)) * kRadToDeg;
      if (el_deg < cfg_.ray_el_min_deg || el_deg >= cfg_.ray_el_max_deg) {
        continue;
      }
      double az_deg = std::atan2(p_s.y(), p_s.x()) * kRadToDeg + 180.0;
      if (az_deg < 0.0) az_deg += 360.0;
      if (az_deg >= 360.0) az_deg -= 360.0;

      const int eb = std::clamp(
          static_cast<int>(std::floor((el_deg - cfg_.ray_el_min_deg) / cfg_.ray_el_step_deg)),
          0, n_el - 1);
      const int ab = std::clamp(
          static_cast<int>(std::floor(az_deg / cfg_.ray_az_step_deg)),
          0, n_az - 1);

      bool occluded_or_present = false;
      bool pierced_by_tube = false;
      for (int de = -1; de <= 1 && !occluded_or_present; ++de) {
        const int eb2 = eb + de;
        if (eb2 < 0 || eb2 >= n_el) continue;
        for (int da = -2; da <= 2; ++da) {
          const int ab2 = ((ab + da) % n_az + n_az) % n_az;
          const RayCell& cell = ray_grid[static_cast<size_t>(eb2 * n_az + ab2)];
          if (!std::isfinite(cell.min_r)) continue;

          // If any neighboring beam hits at or closer than (r_v + ray_clearance_m), do not clear
          if (cell.min_r <= r_v + cfg_.ray_clearance_m) {
            occluded_or_present = true;
            break;
          }
          // Check strict perpendicular ray-tube intersection around voxel v
          const double proj = p_s.dot(cell.dir);
          if (proj > 0.0) {
            const double perp_sq = (p_s - proj * cell.dir).squaredNorm();
            if (perp_sq <= tube_r_sq) {
              pierced_by_tube = true;
            }
          }
        }
      }

      if (!occluded_or_present && pierced_by_tube) {
        v.miss_count++;
        v.hit_count = 0;
        v.log_odds = std::max(cfg_.log_odds_min, v.log_odds - cfg_.log_odds_miss);
        if (v.log_odds <= cfg_.log_odds_delete_thresh) {
          if (v.state == VoxelState::PRIOR_OLD && stats_.prior_active_count > 0) {
            stats_.prior_active_count--;
          } else if (v.state == VoxelState::NEW_KEPT && stats_.new_kept_count > 0) {
            stats_.new_kept_count--;
          }
          const GlobalVoxelKey gvk{ck, kv.first};
          new_kept_keys_.erase(gvk);
          deleted_old_keys_.insert(gvk);
          v.state = VoxelState::DELETED_OLD;
          stats_.deleted_old_count++;
          newly_deleted++;
          dirty = true;
        }
      }
    }
  }

  // --- Phase B: Keep New (Promote verified new static geometry & replace deleted) ---
  if (scan_down && !scan_down->IsEmpty()) {
    const Eigen::Matrix3d R_map_body = T_map_body_aligned.block<3, 3>(0, 0);
    const Eigen::Vector3d t_map_body = T_map_body_aligned.block<3, 1>(0, 3);
    const double new_min_dist_sq =
        (cfg_.new_point_min_dist > 0.0) ? (cfg_.new_point_min_dist * cfg_.new_point_min_dist) : 0.0;
    std::vector<int> nn_idx(1);
    std::vector<double> nn_dist_sq(1);

    for (const auto& p_s : scan_down->points_) {
      if (p_s.norm() < cfg_.ray_min_range || p_s.norm() > cfg_.max_insert_range) {
        continue;
      }
      const Eigen::Vector3d p_w = R_map_body * p_s + t_map_body;
      const Int3Key ck = PointToChunkKey(p_w);
      const Int3Key vk = PointToVoxelKey(p_w);

      auto it_c = chunks_.find(ck);
      auto it_v = (it_c != chunks_.end()) ? it_c->second.voxels.find(vk)
                                          : std::unordered_map<Int3Key, VoxelEntry, Int3KeyHash>::iterator();
      const bool exists = (it_c != chunks_.end() && it_v != it_c->second.voxels.end());

      if (!exists) {
        // Reject points within new_point_min_dist of existing PRIOR_OLD/NEW_KEPT surfaces
        // so voxel-boundary quantization jitter on walls/floor never blooms into Blue noise.
        if (new_min_dist_sq > 0.0 && active_target_kdtree_ &&
            active_target_kdtree_->SearchKNN(p_w, 1, nn_idx, nn_dist_sq) > 0 &&
            nn_dist_sq[0] <= new_min_dist_sq) {
          continue;
        }
        auto& chunk = chunks_[ck];
        if (has_window_ &&
            std::abs(p_w.x() - last_roll_center_.x()) <= cfg_.active_radius_xy &&
            std::abs(p_w.y() - last_roll_center_.y()) <= cfg_.active_radius_xy &&
            p_w.z() >= last_roll_center_.z() - cfg_.active_z_down &&
            p_w.z() <= last_roll_center_.z() + cfg_.active_z_up) {
          active_chunks_.insert(ck);
        }
        VoxelEntry entry;
        entry.mean = p_w;
        entry.has_normal = false;
        entry.log_odds = cfg_.log_odds_hit;
        entry.hit_count = 1;
        entry.miss_count = 0;
        entry.last_frame_id = frame_counter_;
        entry.state = (cfg_.min_hits_to_add <= 1) ? VoxelState::NEW_KEPT : VoxelState::CANDIDATE;
        if (entry.state == VoxelState::NEW_KEPT) {
          new_kept_keys_.insert(GlobalVoxelKey{ck, vk});
          stats_.new_kept_count++;
          newly_promoted++;
          dirty = true;
        }
        chunk.voxels.emplace(vk, entry);
      } else {
        if (has_window_ &&
            std::abs(p_w.x() - last_roll_center_.x()) <= cfg_.active_radius_xy &&
            std::abs(p_w.y() - last_roll_center_.y()) <= cfg_.active_radius_xy &&
            p_w.z() >= last_roll_center_.z() - cfg_.active_z_down &&
            p_w.z() <= last_roll_center_.z() + cfg_.active_z_up) {
          active_chunks_.insert(ck);
        }
        VoxelEntry& v = it_v->second;
        if (v.last_frame_id == frame_counter_) {
          continue;  // At most one update per voxel per frame
        }
        v.last_frame_id = frame_counter_;
        v.miss_count = 0;

        if (v.state == VoxelState::CANDIDATE) {
          v.hit_count++;
          const double n = static_cast<double>(v.hit_count);
          v.mean = (v.mean * (n - 1.0) + p_w) / n;
          v.log_odds = std::min(cfg_.log_odds_max, v.log_odds + cfg_.log_odds_hit);
          if (static_cast<int>(v.hit_count) >= cfg_.min_hits_to_add &&
              v.log_odds >= cfg_.log_odds_add_thresh) {
            v.state = VoxelState::NEW_KEPT;
            v.has_normal = false;
            new_kept_keys_.insert(GlobalVoxelKey{ck, vk});
            stats_.new_kept_count++;
            newly_promoted++;
            dirty = true;
          }
        } else if (v.state == VoxelState::DELETED_OLD) {
          // Previously cleared outdated spot is now re-occupied by new geometry
          v.hit_count++;
          if (v.hit_count == 1) {
            v.mean = p_w;
            v.log_odds = cfg_.log_odds_hit;
          } else {
            const double n = static_cast<double>(v.hit_count);
            v.mean = (v.mean * (n - 1.0) + p_w) / n;
            v.log_odds = std::min(cfg_.log_odds_max, v.log_odds + cfg_.log_odds_hit);
          }
          if (static_cast<int>(v.hit_count) >= cfg_.min_hits_to_add &&
              v.log_odds >= cfg_.log_odds_add_thresh) {
            v.state = VoxelState::NEW_KEPT;
            v.has_normal = false;
            const GlobalVoxelKey gvk{ck, vk};
            deleted_old_keys_.erase(gvk);
            new_kept_keys_.insert(gvk);
            if (stats_.deleted_old_count > 0) stats_.deleted_old_count--;
            stats_.new_kept_count++;
            newly_promoted++;
            dirty = true;
          }
        } else if (v.state == VoxelState::NEW_KEPT) {
          v.hit_count++;
          v.mean = 0.85 * v.mean + 0.15 * p_w;
          v.log_odds = std::min(cfg_.log_odds_max, v.log_odds + 0.5f * cfg_.log_odds_hit);
        } else {
          // PRIOR_OLD: reinforce static confidence
          v.hit_count++;
          v.log_odds = std::min(cfg_.log_odds_max, v.log_odds + 0.5f * cfg_.log_odds_hit);
        }
      }
    }
  }

  stats_.total_chunk_count = chunks_.size();
  stats_.active_chunk_count = active_chunks_.size();

  if (dirty) {
    target_dirty_ = true;
    pending_dirty_voxels_ += newly_promoted + newly_deleted;
    if (pending_dirty_voxels_ >= cfg_.kdtree_rebuild_dirty_thresh ||
        !active_target_cloud_ || !active_target_kdtree_) {
      RebuildActiveTargetCloudLocked();
    }
  }

  auto t1 = std::chrono::high_resolution_clock::now();
  stats_.last_update_ms =
      std::chrono::duration<double, std::milli>(t1 - t0).count();
  (void)sensor_pos_map;
  return {newly_promoted, newly_deleted};
}

std::shared_ptr<open3d::geometry::PointCloud> RollingVoxelMap::GetActiveSubmapCloud() const {
  std::lock_guard<std::mutex> lock(mutex_);
  auto out = std::make_shared<open3d::geometry::PointCloud>();
  for (const auto& ck : active_chunks_) {
    auto it_c = chunks_.find(ck);
    if (it_c == chunks_.end()) continue;
    for (const auto& kv : it_c->second.voxels) {
      if (kv.second.state == VoxelState::PRIOR_OLD) {
        out->points_.push_back(kv.second.mean);
        out->colors_.emplace_back(0.62, 0.62, 0.68);
      } else if (kv.second.state == VoxelState::NEW_KEPT) {
        out->points_.push_back(kv.second.mean);
        out->colors_.emplace_back(0.08, 0.58, 1.00);
      }
    }
  }
  return out;
}

std::shared_ptr<open3d::geometry::PointCloud> RollingVoxelMap::GetNewKeptCloud() const {
  std::lock_guard<std::mutex> lock(mutex_);
  auto out = std::make_shared<open3d::geometry::PointCloud>();
  out->points_.reserve(new_kept_keys_.size());
  out->colors_.reserve(new_kept_keys_.size());
  for (const auto& gk : new_kept_keys_) {
    auto it_c = chunks_.find(gk.chunk);
    if (it_c == chunks_.end()) continue;
    auto it_v = it_c->second.voxels.find(gk.voxel);
    if (it_v != it_c->second.voxels.end() && it_v->second.state == VoxelState::NEW_KEPT) {
      out->points_.push_back(it_v->second.mean);
      out->colors_.emplace_back(0.08, 0.58, 1.00);
    }
  }
  return out;
}

std::shared_ptr<open3d::geometry::PointCloud> RollingVoxelMap::GetDeletedOldCloud() const {
  std::lock_guard<std::mutex> lock(mutex_);
  auto out = std::make_shared<open3d::geometry::PointCloud>();
  out->points_.reserve(deleted_old_keys_.size());
  out->colors_.reserve(deleted_old_keys_.size());
  for (const auto& gk : deleted_old_keys_) {
    auto it_c = chunks_.find(gk.chunk);
    if (it_c == chunks_.end()) continue;
    auto it_v = it_c->second.voxels.find(gk.voxel);
    if (it_v != it_c->second.voxels.end() && it_v->second.state == VoxelState::DELETED_OLD) {
      out->points_.push_back(it_v->second.mean);
      out->colors_.emplace_back(0.98, 0.16, 0.16);
    }
  }
  return out;
}

std::shared_ptr<open3d::geometry::PointCloud> RollingVoxelMap::GetWorkingGlobalCloud() const {
  std::lock_guard<std::mutex> lock(mutex_);
  auto out = std::make_shared<open3d::geometry::PointCloud>();
  for (const auto& ck_pair : chunks_) {
    for (const auto& kv : ck_pair.second.voxels) {
      if (kv.second.state == VoxelState::PRIOR_OLD || kv.second.state == VoxelState::NEW_KEPT) {
        out->points_.push_back(kv.second.mean);
        if (kv.second.has_normal) {
          out->normals_.push_back(kv.second.normal);
        }
        if (kv.second.state == VoxelState::NEW_KEPT) {
          out->colors_.emplace_back(0.08, 0.58, 1.00);
        } else {
          out->colors_.emplace_back(0.62, 0.62, 0.68);
        }
      }
    }
  }
  return out;
}

std::shared_ptr<open3d::geometry::PointCloud> RollingVoxelMap::ColorizeAlignedScanByMatch(
    const open3d::geometry::PointCloud& aligned_scan_map,
    bool alignment_valid,
    double match_dist_thresh) {
  auto out = std::make_shared<open3d::geometry::PointCloud>();
  out->points_ = aligned_scan_map.points_;
  const size_t n = out->points_.size();
  out->colors_.resize(n, Eigen::Vector3d(0.98, 0.18, 0.18));  // Default: unmatched = Red
  if (n == 0 || !alignment_valid) {
    return out;
  }

  std::shared_ptr<open3d::geometry::KDTreeFlann> kdtree_copy;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (target_dirty_ || !active_target_kdtree_) {
      RebuildActiveTargetCloudLocked();
    }
    kdtree_copy = active_target_kdtree_;
  }
  if (!kdtree_copy) {
    return out;
  }

  const double thresh =
      (match_dist_thresh > 0.0) ? match_dist_thresh : cfg_.scan_match_color_dist;
  const double thresh_sq = thresh * thresh;
  const Eigen::Vector3d kMatchedGreen(0.12, 0.95, 0.28);
  const Eigen::Vector3d kUnmatchedRed(0.98, 0.18, 0.18);

  std::vector<int> nn_idx(1);
  std::vector<double> nn_dist_sq(1);
  for (size_t i = 0; i < n; ++i) {
    if (kdtree_copy->SearchKNN(out->points_[i], 1, nn_idx, nn_dist_sq) > 0 &&
        nn_dist_sq[0] <= thresh_sq) {
      out->colors_[i] = kMatchedGreen;
    } else {
      out->colors_[i] = kUnmatchedRed;
    }
  }
  return out;
}

bool RollingVoxelMap::ExportSnapshotToPath(const std::string& out_path) const {
  auto cloud = GetWorkingGlobalCloud();
  if (!cloud || cloud->IsEmpty()) {
    return false;
  }
  return open3d::io::WritePointCloud(out_path, *cloud);
}

MapStats RollingVoxelMap::GetStats() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return stats_;
}

}  // namespace rolling_map_updater
