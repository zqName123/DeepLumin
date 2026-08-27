#pragma once

/**
 * @file relocalization_validator.hpp
 * @brief 重定位结果多重校验
 */

#include <aruco_relocalization/common/types.hpp>
#include <aruco_relocalization/map/marker_map_loader.hpp>
#include <aruco_relocalization/solver/relocalization_solver.hpp>

#include <opencv2/core.hpp>

#include <utility>

namespace aruco_reloc {

struct ValidatorConfig {
  double max_reproj_error = 10.0;            // pixels
  double max_marker_distance_error = 0.1;    // meters
  double max_translation_jump = 2.0;         // meters
  double max_rotation_jump = 30.0;           // degrees
  bool use_motion_check = true;
  int min_markers = 2;                       // 推荐 >=2
};

struct ValidationResult {
  bool passed = false;
  double mean_reproj_error = 0.0;
  double max_marker_distance_error = 0.0;
  double translation_jump = 0.0;
  double rotation_jump = 0.0;
  int num_valid_markers = 0;
  std::string reject_reason;
};

class RelocalizationValidator {
 public:
  bool initialize(const ValidatorConfig& config, const cv::Mat& camera_matrix,
                  const cv::Mat& dist_coeffs, const Eigen::Isometry3d& T_base_cam);

  /// 执行校验
  ValidationResult validate(const SolveResult& candidate,
                            const FrameDetection& detection,
                            const MarkerMapLoader& map,
                            const Eigen::Isometry3d* last_pose = nullptr) const;

 private:
  /// 校验 1：重投影误差
  double computeReprojectionError(const Eigen::Isometry3d& T_map_cam,
                                  const std::vector<MarkerObservation>& observations,
                                  const MarkerMapLoader& map) const;

  /// 校验 2：几何一致性
  double computeMarkerDistanceError(const std::vector<MarkerObservation>& observations,
                                   const MarkerMapLoader& map,
                                   const Eigen::Isometry3d& T_map_cam) const;

  /// 校验 3：运动一致性
  std::pair<double, double> computeMotionJump(const Eigen::Isometry3d& current_pose,
                                              const Eigen::Isometry3d& last_pose) const;

  /// 计算 marker 在 map 下的 3D 角点
  std::vector<cv::Point3d> computeMarkerCorners(const MarkerItem& item) const;

  ValidatorConfig config_;
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
  Eigen::Isometry3d T_base_cam_ = Eigen::Isometry3d::Identity();
};

}  // namespace aruco_reloc
