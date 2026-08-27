#pragma once

/**
 * @file relocalization_solver.hpp
 * @brief 重定位求解核心（单/多 marker PnP）
 */

#include <aruco_relocalization/common/types.hpp>
#include <aruco_relocalization/map/marker_map_loader.hpp>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#include <string>
#include <vector>

namespace aruco_reloc {

struct SolverConfig {
  int min_markers = 1;               // 最小有效 marker 数
  bool use_ransac = true;            // 是否使用 RANSAC
  int ransac_iterations = 100;       // RANSAC 迭代次数
  double ransac_reproj_error = 8.0;  // RANSAC 重投影误差阈值（pixels）
  double ransac_confidence = 0.99;   // RANSAC 置信度
};

struct SolveResult {
  bool success = false;
  Eigen::Isometry3d T_map_cam = Eigen::Isometry3d::Identity();  // 相机在 map 下的位姿
  Eigen::Isometry3d T_map_base = Eigen::Isometry3d::Identity(); // base_link 在 map 下的位姿
  int num_markers_used = 0;
  std::vector<int> marker_ids;
  std::vector<int> inlier_indices;
  double solve_time_ms = 0.0;
  std::string failure_reason;
};

class RelocalizationSolver {
 public:
  bool initialize(const SolverConfig& config, const cv::Mat& camera_matrix,
                  const cv::Mat& dist_coeffs, const Eigen::Isometry3d& T_base_cam);

  /// 执行重定位求解
  SolveResult solve(const FrameDetection& detection, const MarkerMapLoader& map) const;

 private:
  /// 单 marker 求解
  SolveResult solveSingleMarker(const MarkerObservation& obs,
                                const MarkerItem& map_item) const;

  /// 多 marker 联合 PnP 求解
  SolveResult solveMultiMarker(const std::vector<MarkerObservation>& observations,
                               const std::vector<MarkerItem>& map_items) const;

  /// 计算 marker 在 map 下的 3D 角点
  std::vector<cv::Point3d> computeMarkerCorners(const MarkerItem& item) const;

  SolverConfig config_;
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
  Eigen::Isometry3d T_base_cam_ = Eigen::Isometry3d::Identity();
};

}  // namespace aruco_reloc
