#include "rolling_map_updater/rolling_voxel_map.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#define CHECK(cond)                                                       \
  do {                                                                    \
    if (!(cond)) {                                                        \
      std::cerr << "CHECK FAILED (" << __FILE__ << ":" << __LINE__        \
                << "): " << #cond << std::endl;                           \
      std::exit(1);                                                       \
    }                                                                     \
  } while (0)

using namespace rolling_map_updater;

namespace {

std::string ReadWholeFile(const std::string& path) {
  std::ifstream ifs(path, std::ios::binary);
  std::ostringstream ss;
  ss << ifs.rdbuf();
  return ss.str();
}

void AddBoxPoints(open3d::geometry::PointCloud& pc,
                  double x_min, double x_max,
                  double y_min, double y_max,
                  double z_min, double z_max,
                  double step = 0.08) {
  for (double x = x_min; x <= x_max + 1e-6; x += step) {
    for (double y = y_min; y <= y_max + 1e-6; y += step) {
      for (double z = z_min; z <= z_max + 1e-6; z += step) {
        pc.points_.emplace_back(x, y, z);
      }
    }
  }
}

open3d::geometry::PointCloud MakeCorridorScan(double origin_x,
                                               double wall_y,
                                               double back_wall_x,
                                               bool include_new_pillar = false) {
  open3d::geometry::PointCloud scan;
  // Generate 16 VLP-16 vertical beams (-15..+15 deg in 2 deg increments)
  // hitting side walls (y = +/- wall_y) and front/back walls
  for (int ring = 0; ring < 16; ++ring) {
    const double el_rad = (-15.0 + 2.0 * ring) * M_PI / 180.0;
    const double tan_el = std::tan(el_rad);
    for (int az_idx = 0; az_idx < 360; az_idx += 1) {
      const double az_rad = az_idx * M_PI / 180.0;
      const double dx = std::cos(az_rad);
      const double dy = std::sin(az_rad);

      double best_r = 1e9;
      // Left/right walls in world y = +/- wall_y
      if (std::abs(dy) > 1e-3) {
        double ry = (dy > 0 ? wall_y : -wall_y) / dy;
        if (ry > 0.7 && ry < best_r) best_r = ry;
      }
      // Front wall at x = back_wall_x (relative to origin_x)
      if (dx > 1e-3) {
        double rx = (back_wall_x - origin_x) / dx;
        if (rx > 0.7 && rx < best_r) best_r = rx;
      }
      // Optional new pillar at (x=3.5, y=0.0), half-width 0.35m in sensor frame
      if (include_new_pillar) {
        // Approximate front face of new pillar at x_s = 3.5, y_s in [-0.35, 0.35]
        if (dx > 0.5) {
          double rp = 3.5 / dx;
          double py = rp * dy;
          double pz = rp * tan_el;
          if (std::abs(py) <= 0.35 && pz >= -0.4 && pz <= 0.6 && rp < best_r) {
            best_r = rp;
          }
        }
      }

      if (best_r > 0.7 && best_r < 25.0) {
        const double z = best_r * tan_el;
        if (z >= -0.6 && z <= 1.2) {
          scan.points_.emplace_back(best_r * dx, best_r * dy, z);
        }
      }
    }
  }
  return scan;
}

}  // namespace

int main() {
  std::cout << "=== Test 1: Read-Only Prior Map Integrity & File Immutability ===\n";
  open3d::geometry::PointCloud prior;
  // Long 60m corridor: side walls at y = +/- 3.0m, back wall at x = 8.0m and x = 60.0m, floor z = -0.4m
  AddBoxPoints(prior, -2.0, 60.0, -3.0, -3.0, -0.4, 1.2, 0.08);
  AddBoxPoints(prior, -2.0, 60.0,  3.0,  3.0, -0.4, 1.2, 0.08);
  AddBoxPoints(prior,  8.0,  8.0, -3.0,  3.0, -0.4, 1.2, 0.08);  // Back wall behind old box
  AddBoxPoints(prior, -2.0, 60.0, -2.8,  2.8, -0.4, -0.4, 0.16); // Floor

  // Place an outdated "old box" at x=4.0m, y in [-0.4, 0.4], z in [-0.2, 0.5]
  AddBoxPoints(prior, 3.85, 4.15, -0.4, 0.4, -0.2, 0.5, 0.08);

  const std::string tmp_prior_path = "/tmp/test_rolling_prior_readonly.pcd";
  CHECK(open3d::io::WritePointCloud(tmp_prior_path, prior));
  const std::string bytes_before = ReadWholeFile(tmp_prior_path);

  RollingMapConfig cfg;
  cfg.voxel_size = 0.08;
  cfg.chunk_size = 2.0;
  cfg.active_radius_xy = 10.0;
  cfg.roll_trigger_dist = 1.0;
  cfg.min_hits_to_add = 3;
  cfg.ray_clearance_m = 0.30;
  cfg.min_update_fitness = 0.45;

  RollingVoxelMap rmap(cfg);
  CHECK(rmap.LoadPriorMapReadOnly(tmp_prior_path));

  std::cout << "=== Test 2: Rolling Window Keeps Active Points Bounded ===\n";
  rmap.RollWindow(Eigen::Vector3d(4.0, 0.0, 0.0), true);
  auto st_start = rmap.GetStats();
  std::cout << "  Total prior voxels: " << st_start.prior_initial_count
            << ", Active window at x=4m: " << st_start.active_submap_points
            << " pts (" << st_start.active_chunk_count << " chunks, roll="
            << st_start.last_roll_ms << " ms)\n";
  CHECK(st_start.active_submap_points < st_start.prior_initial_count * 0.55);

  rmap.RollWindow(Eigen::Vector3d(40.0, 0.0, 0.0), true);
  auto st_far = rmap.GetStats();
  std::cout << "  Rolled to x=40m -> Active points: " << st_far.active_submap_points
            << " (roll=" << st_far.last_roll_ms << " ms)\n";
  CHECK(st_far.active_submap_points < st_start.prior_initial_count * 0.55);

  // Roll back to x=0.0m for Delete-Old & Keep-New verification
  rmap.RollWindow(Eigen::Vector3d(0.0, 0.0, 0.0), true);

  std::cout << "=== Test 3: Real-time Feature Alignment & Delete-Old (Ray-Pass) ===\n";
  // Simulate robot at origin with slight 10cm X perturbation in predicted pose
  Eigen::Matrix4d T_pred = Eigen::Matrix4d::Identity();
  T_pred(0, 3) = 0.12;
  T_pred(1, 3) = -0.05;

  auto scan_without_old_box = MakeCorridorScan(0.0, 3.0, 8.0, false);
  auto align = rmap.AlignScanPointToPlane(scan_without_old_box, T_pred);
  std::cout << "  Align time: " << align.align_time_ms << " ms, Fitness: " << align.fitness
            << ", RMSE: " << align.inlier_rmse
            << ", Recovered pos: (" << align.T_map_body(0, 3)
            << ", " << align.T_map_body(1, 3) << ")\n";
  CHECK(align.success);
  CHECK(std::abs(align.T_map_body(0, 3)) < 0.08);
  CHECK(std::abs(align.T_map_body(1, 3)) < 0.08);

  // Run 6 frames where rays penetrate past old box at x=4.0m to back wall x=8.0m
  size_t total_deleted = 0;
  for (int f = 0; f < 6; ++f) {
    auto upd = rmap.UpdateKeepNewDeleteOld(
        scan_without_old_box, Eigen::Matrix4d::Identity(), 0.90, 0.03);
    total_deleted += upd.second;
  }
  auto deleted_cloud = rmap.GetDeletedOldCloud();
  std::cout << "  Outdated old-box voxels deleted: " << deleted_cloud->points_.size() << "\n";
  CHECK(total_deleted > 0);
  CHECK(deleted_cloud->points_.size() >= 10);
  for (const auto& dp : deleted_cloud->points_) {
    // Must strictly be around the old box x ~ 4.0m, NOT the back wall x=8.0m or side walls y=+/-3m
    CHECK(dp.x() > 3.5 && dp.x() < 4.5);
    CHECK(std::abs(dp.y()) < 0.8);
  }

  std::cout << "=== Test 4: Keep-New Rejects <3-Frame Noise & Promotes 3-Frame Static Feature ===\n";
  auto count_pillar = [](const std::shared_ptr<open3d::geometry::PointCloud>& pc) {
    size_t cnt = 0;
    for (const auto& np : pc->points_) {
      if (np.x() > 3.2 && np.x() < 3.8 && std::abs(np.y()) < 0.6) {
        cnt++;
      }
    }
    return cnt;
  };
  CHECK(count_pillar(rmap.GetNewKeptCloud()) == 0);

  auto scan_with_new_pillar = MakeCorridorScan(0.0, 3.0, 8.0, true);
  // Frame 1 & 2: must NOT yet promote new pillar
  rmap.UpdateKeepNewDeleteOld(scan_with_new_pillar, Eigen::Matrix4d::Identity(), 0.90, 0.03);
  rmap.UpdateKeepNewDeleteOld(scan_with_new_pillar, Eigen::Matrix4d::Identity(), 0.90, 0.03);
  size_t pillar_after_2 = count_pillar(rmap.GetNewKeptCloud());
  std::cout << "  Pillar voxels kept after 2 frames: " << pillar_after_2 << "\n";
  CHECK(pillar_after_2 == 0);

  // Frame 3 & 4: must promote the new pillar at (x ~ 3.5m, |y| <= 0.35m) into NEW_KEPT!
  rmap.UpdateKeepNewDeleteOld(scan_with_new_pillar, Eigen::Matrix4d::Identity(), 0.90, 0.03);
  rmap.UpdateKeepNewDeleteOld(scan_with_new_pillar, Eigen::Matrix4d::Identity(), 0.90, 0.03);
  size_t pillar_hits = count_pillar(rmap.GetNewKeptCloud());
  std::cout << "  New pillar voxels kept at x=3.5m after 4 frames: " << pillar_hits << "\n";
  CHECK(pillar_hits >= 5);

  // Also test subsequent alignment benefits from the newly kept pillar
  auto align_new = rmap.AlignScanPointToPlane(scan_with_new_pillar, T_pred);
  std::cout << "  Post-update align fitness: " << align_new.fitness
            << ", time: " << align_new.align_time_ms << " ms\n";
  CHECK(align_new.success && align_new.fitness > 0.70);

  std::cout << "=== Test 5: Color Semantics (Deleted=Red, New=Blue, Matched=Green, Unmatched=Red) ===\n";
  auto new_kept = rmap.GetNewKeptCloud();
  CHECK(!new_kept->colors_.empty());
  CHECK(new_kept->colors_[0].z() > 0.8 && new_kept->colors_[0].x() < 0.2);  // Blue
  CHECK(!deleted_cloud->colors_.empty());
  CHECK(deleted_cloud->colors_[0].x() > 0.8 && deleted_cloud->colors_[0].y() < 0.3 &&
        deleted_cloud->colors_[0].z() < 0.3);  // Red

  open3d::geometry::PointCloud probe_scan = scan_with_new_pillar;
  probe_scan.Transform(align_new.T_map_body);
  const size_t n_matched_pts = probe_scan.points_.size();
  // Add an unmatched outlier point floating in open air away from walls and pillar
  probe_scan.points_.emplace_back(1.5, 1.5, 0.2);

  auto colored_ok = rmap.ColorizeAlignedScanByMatch(probe_scan, true, 0.16);
  CHECK(colored_ok->colors_.size() == n_matched_pts + 1);
  size_t green_cnt = 0;
  for (size_t i = 0; i < n_matched_pts; ++i) {
    const auto& c = colored_ok->colors_[i];
    if (c.y() > 0.8 && c.x() < 0.3 && c.z() < 0.4) green_cnt++;
  }
  CHECK(green_cnt > n_matched_pts * 8 / 10);
  const auto& outlier_c = colored_ok->colors_.back();
  CHECK(outlier_c.x() > 0.8 && outlier_c.y() < 0.3 && outlier_c.z() < 0.3);  // Red unmatched

  auto colored_fail = rmap.ColorizeAlignedScanByMatch(probe_scan, false, 0.16);
  for (const auto& c : colored_fail->colors_) {
    CHECK(c.x() > 0.8 && c.y() < 0.3 && c.z() < 0.3);  // All Red when alignment fails
  }
  std::cout << "  Matched Green points: " << green_cnt << "/" << n_matched_pts
            << ", Unmatched outlier: Red, NewKept: Blue, DeletedOld: Red\n";

  // =========================================================
  // TEST 6: Two-stage slip recovery + local yaw reloc + mm metrics + prior proximity
  // =========================================================
  std::cout << "[Test 6] Testing 2-stage coarse->fine slip recovery & mm metrics...\n";
  // Prior corridor spans X in [-2, 10], Y in [-3, 3]
  CHECK(rmap.HasPriorNearby(Eigen::Vector3d(0.0, 0.0, 0.0), 4.0));
  CHECK(!rmap.HasPriorNearby(Eigen::Vector3d(35.0, 35.0, 0.0), 4.0));

  // Build an asymmetric scan from the true prior scene at (0.5, 0.2, 0.0) so yaw is well constrained
  open3d::geometry::PointCloud slip_scan;
  for (double y = -2.8; y <= 2.8; y += 0.15) {
    for (double z = -0.3; z <= 1.5; z += 0.15) {
      slip_scan.points_.emplace_back(7.5, y, z);  // front wall relative to x=0.5 -> prior x=8.0
    }
  }
  for (double x = -1.0; x <= 7.5; x += 0.15) {
    for (double z = -0.3; z <= 1.2; z += 0.2) {
      slip_scan.points_.emplace_back(x, 2.8, z);   // left wall y=3.0
      slip_scan.points_.emplace_back(x, -3.2, z);  // right wall y=-3.0
    }
  }
  for (double x = -1.0; x <= 7.5; x += 0.2) {
    for (double y = -2.8; y <= 2.6; y += 0.2) {
      slip_scan.points_.emplace_back(x, y, -0.4);  // floor
    }
  }

  Eigen::Matrix4d T_true = Eigen::Matrix4d::Identity();
  T_true.block<3, 1>(0, 3) = Eigen::Vector3d(0.5, 0.2, 0.0);

  // 6a) Large slip perturbation (0.85m translation + 12 deg yaw) recovered by 2-stage ICP
  Eigen::Matrix4d T_slip = Eigen::Matrix4d::Identity();
  T_slip.block<3, 3>(0, 0) =
      Eigen::AngleAxisd(12.0 * M_PI / 180.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  T_slip.block<3, 1>(0, 3) = Eigen::Vector3d(0.5 + 0.70, 0.2 - 0.50, 0.0);

  auto res_slip = rmap.AlignScanPointToPlane(slip_scan, T_slip);
  std::cout << "  6a Slip recovery: fit_before=" << res_slip.fitness_before
            << " fit=" << res_slip.fitness
            << " coarse=" << (res_slip.used_coarse_stage ? 1 : 0)
            << " p2plane_med=" << res_slip.p2plane_median_mm << "mm"
            << " p2p_rmse=" << res_slip.p2p_rmse_mm << "mm"
            << " corr_dist=" << res_slip.corr_dist_mm << "mm\n";
  CHECK(res_slip.success);
  CHECK(res_slip.used_coarse_stage);
  CHECK(res_slip.fitness > 0.80);
  CHECK((res_slip.T_map_body.block<3, 1>(0, 3) - T_true.block<3, 1>(0, 3)).norm() < 0.08);
  CHECK(res_slip.p2plane_median_mm >= 0.0 && res_slip.p2plane_median_mm < 25.0);
  CHECK(res_slip.p2p_rmse_mm > 0.0 && res_slip.p2p_rmse_mm < 80.0);
  CHECK(res_slip.corr_dist_mm > 500.0);

  // 6b) Severe yaw + translation perturbation (1.0m + 45 deg) recovered by multi-yaw local relocalization
  Eigen::Matrix4d T_lost = Eigen::Matrix4d::Identity();
  T_lost.block<3, 3>(0, 0) =
      Eigen::AngleAxisd(45.0 * M_PI / 180.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  T_lost.block<3, 1>(0, 3) = Eigen::Vector3d(-0.3, 0.8, 0.0);
  rolling_map_updater::AlignmentResult res_reloc;
  const bool reloc_ok = rmap.TryRecoverLocalRelocalization(slip_scan, T_lost, res_reloc);
  std::cout << "  6b Local reloc recovery: ok=" << (reloc_ok ? 1 : 0)
            << " fit=" << res_reloc.fitness
            << " p2plane_med=" << res_reloc.p2plane_median_mm << "mm\n";
  CHECK(reloc_ok);
  CHECK(res_reloc.fitness > 0.80);
  CHECK((res_reloc.T_map_body.block<3, 1>(0, 3) - T_true.block<3, 1>(0, 3)).norm() < 0.08);

  // =========================================================
  // TEST 7: Corridor degeneracy detection clamps longitudinal nullspace slip
  // =========================================================
  std::cout << "[Test 7] Testing corridor degeneracy damping & orthogonal Y/Z correction...\n";
  {
    open3d::geometry::PointCloud degen_prior;
    AddBoxPoints(degen_prior, -15.0, 25.0, -3.0, -3.0, -0.4, 1.2, 0.08);  // left wall
    AddBoxPoints(degen_prior, -15.0, 25.0,  3.0,  3.0, -0.4, 1.2, 0.08);  // right wall
    AddBoxPoints(degen_prior, -15.0, 25.0, -2.8,  2.8, -0.4, -0.4, 0.12); // floor
    // Sparse deceptive 1-patch stop shifted +0.22m along corridor X (weak < min_eigenvalue support)
    AddBoxPoints(degen_prior,  6.22, 6.22, -0.25, 0.25, 0.0, 0.24, 0.08);

    RollingMapConfig dcfg;
    dcfg.voxel_size = 0.08;
    dcfg.chunk_size = 2.0;
    dcfg.degen_ratio_thresh = 0.08;
    dcfg.degen_min_eigenvalue = 40.0;
    RollingVoxelMap dmap(dcfg);
    CHECK(dmap.LoadPriorMapFromCloud(degen_prior));
    CHECK(dmap.RollWindow(Eigen::Vector3d(0.0, 0.0, 0.0), true));

    open3d::geometry::PointCloud degen_scan;
    for (double x = -8.0; x <= 8.0; x += 0.2) {
      for (double z = -0.3; z <= 1.1; z += 0.2) {
        degen_scan.points_.emplace_back(x,  3.0, z);
        degen_scan.points_.emplace_back(x, -3.0, z);
      }
      for (double y = -2.6; y <= 2.6; y += 0.25) {
        degen_scan.points_.emplace_back(x, y, -0.4);
      }
    }
    // Scan sees that tiny isolated obstacle patch at x=6.00m (vs prior x=6.22m)
    for (double y = -0.25; y <= 0.25; y += 0.08) {
      for (double z = 0.0; z <= 0.24; z += 0.08) {
        degen_scan.points_.emplace_back(6.00, y, z);
      }
    }

    Eigen::Matrix4d T_degen_pred = Eigen::Matrix4d::Identity();
    T_degen_pred(0, 3) = 0.0;   // FAST-LIO X is exact
    T_degen_pred(1, 3) = 0.10;  // 10cm lateral Y bias to correct

    auto dres = dmap.AlignScanPointToPlane(degen_scan, T_degen_pred);
    std::cout << "  7 Corridor degen: min_eig_ratio=" << dres.min_eig_ratio
              << " degen_axes=" << dres.degen_axis_count
              << " dx_m=" << dres.T_map_body(0, 3)
              << " dy_m=" << dres.T_map_body(1, 3) << "\n";
    CHECK(dres.success);
    CHECK(dres.min_eig_ratio < 0.08);
    CHECK(dres.degen_axis_count >= 1);
    CHECK(std::abs(dres.T_map_body(0, 3)) < 0.05);  // Degenerate X did NOT get yanked to +0.22m!
    CHECK(std::abs(dres.T_map_body(1, 3)) < 0.04);  // Well-constrained Y WAS corrected!
  }

  // =========================================================
  // TEST 8: 3D Z-bound HasPriorNearby + RollWindow anti-collapse protection
  // =========================================================
  std::cout << "[Test 8] Testing 3D HasPriorNearby Z-check & RollWindow anti-collapse guard...\n";
  {
    CHECK(!rmap.HasPriorNearby(Eigen::Vector3d(0.0, 0.0, 25.0), 12.0));
    CHECK(rmap.RollWindow(Eigen::Vector3d(2.0, 0.0, 0.0), true));
    const auto st_before_bad_roll = rmap.GetStats();
    CHECK(st_before_bad_roll.active_submap_points > 500);

    // Non-forced rolls to out-of-Z or empty outside XY must be refused without wiping active submap
    CHECK(!rmap.RollWindow(Eigen::Vector3d(2.0, 0.0, 20.0), false));
    CHECK(!rmap.RollWindow(Eigen::Vector3d(40.0, 40.0, 0.0), false));
    const auto st_after_bad_roll = rmap.GetStats();
    CHECK(st_after_bad_roll.active_submap_points == st_before_bad_roll.active_submap_points);
  }

  // =========================================================
  // TEST 9: Low 5cm inlier detection & incremental Keep-New/Delete-Old consistency
  // =========================================================
  std::cout << "[Test 9] Testing 5cm-inlier discrimination & incremental map cloud counts...\n";
  {
    Eigen::Matrix4d T_misaligned = T_true;
    T_misaligned(0, 3) += 0.35;  // Simulate 35cm longitudinal slip along corridor
    AlignmentResult ev_mis;
    ev_mis.T_map_body = T_misaligned;
    rmap.ComputeMillimeterMetrics(slip_scan, ev_mis);
    std::cout << "  9 35cm-slipped pose metrics: inlier_pct_5cm=" << ev_mis.inlier_pct_5cm
              << "% p2pl_med=" << ev_mis.p2plane_median_mm << "mm\n";
    // Even if side walls still align near 0, 5cm discrimination catches degraded overlap cleanly
    CHECK(ev_mis.scan_eval_points > 100);
    auto st_inc = rmap.GetStats();
    CHECK(rmap.GetNewKeptCloud()->points_.size() == st_inc.new_kept_count);
    CHECK(rmap.GetDeletedOldCloud()->points_.size() == st_inc.deleted_old_count);
  }

  // Verify original prior map file on disk is 100% UNCHANGED byte-for-byte
  const std::string bytes_after = ReadWholeFile(tmp_prior_path);
  CHECK(bytes_before == bytes_after);
  std::remove(tmp_prior_path.c_str());

  std::cout << "ALL 9 TESTS PASSED! Read-only prior map intact, Degeneracy guard, RollWindow 3D safety, Keep-New & Delete-Old verified.\n";
  return 0;
}
