#include <aruco_relocalization/validator/relocalization_validator.hpp>

#include <ros/ros.h>

#include <cmath>

namespace aruco_reloc {

bool RelocalizationValidator::initialize(const ValidatorConfig& config,
                                         const cv::Mat& camera_matrix,
                                         const cv::Mat& dist_coeffs,
                                         const Eigen::Isometry3d& T_base_cam) {
  config_ = config;
  camera_matrix_ = camera_matrix.clone();
  dist_coeffs_ = dist_coeffs.clone();
  T_base_cam_ = T_base_cam;
  return true;
}

ValidationResult RelocalizationValidator::validate(const SolveResult& candidate,
                                                   const FrameDetection& detection,
                                                   const MarkerMapLoader& map,
                                                   const Eigen::Isometry3d* last_pose) const {
  ValidationResult result;

  if (!candidate.success) {
    result.reject_reason = "candidate_not_success";
    return result;
  }

  // 获取匹配的观测
  std::vector<MarkerObservation> matched_obs;
  for (const auto& obs : detection.markers) {
    for (int id : candidate.marker_ids) {
      if (obs.id == id) {
        matched_obs.push_back(obs);
        break;
      }
    }
  }

  result.num_valid_markers = static_cast<int>(matched_obs.size());

  // 校验 1：数量阈值
  if (result.num_valid_markers < config_.min_markers) {
    result.reject_reason = "not_enough_markers (" +
                           std::to_string(result.num_valid_markers) + "/" +
                           std::to_string(config_.min_markers) + ")";
    return result;
  }

  // 校验 2：重投影误差
  result.mean_reproj_error = computeReprojectionError(
      candidate.T_map_cam, matched_obs, map);
  if (result.mean_reproj_error > config_.max_reproj_error) {
    result.reject_reason = "reproj_error_too_large (" +
                           std::to_string(result.mean_reproj_error) + "/" +
                           std::to_string(config_.max_reproj_error) + ")";
    return result;
  }

  // 校验 3：几何一致性（多 marker 时）
  if (matched_obs.size() >= 2) {
    result.max_marker_distance_error = computeMarkerDistanceError(
        matched_obs, map, candidate.T_map_cam);
    if (result.max_marker_distance_error > config_.max_marker_distance_error) {
      result.reject_reason = "geom_inconsistency (" +
                             std::to_string(result.max_marker_distance_error) + "/" +
                             std::to_string(config_.max_marker_distance_error) + ")";
      return result;
    }
  }

  // 校验 4：运动一致性
  if (config_.use_motion_check && last_pose != nullptr) {
    auto [trans_jump, rot_jump] = computeMotionJump(candidate.T_map_base, *last_pose);
    result.translation_jump = trans_jump;
    result.rotation_jump = rot_jump;

    if (trans_jump > config_.max_translation_jump) {
      result.reject_reason = "translation_jump (" +
                             std::to_string(trans_jump) + "/" +
                             std::to_string(config_.max_translation_jump) + ")";
      return result;
    }
    if (rot_jump > config_.max_rotation_jump) {
      result.reject_reason = "rotation_jump (" +
                             std::to_string(rot_jump) + "/" +
                             std::to_string(config_.max_rotation_jump) + ")";
      return result;
    }
  }

  // 全部校验通过
  result.passed = true;
  return result;
}

double RelocalizationValidator::computeReprojectionError(
    const Eigen::Isometry3d& T_map_cam,
    const std::vector<MarkerObservation>& observations,
    const MarkerMapLoader& map) const {
  double total_error = 0.0;
  int total_points = 0;

  Eigen::Isometry3d T_cam_map = T_map_cam.inverse();

  for (const auto& obs : observations) {
    MarkerItem map_item;
    if (!map.getMarker(obs.id, map_item)) continue;

    auto corners_3d = computeMarkerCorners(map_item);

    // 转换到相机坐标系
    std::vector<cv::Point3d> corners_cam;
    for (const auto& p : corners_3d) {
      Eigen::Vector3d p_cam = T_cam_map * Eigen::Vector3d(p.x, p.y, p.z);
      corners_cam.emplace_back(p_cam.x(), p_cam.y(), p_cam.z());
    }

    // 投影到图像
    std::vector<cv::Point2d> projected;
    cv::projectPoints(corners_cam, cv::Vec3d::zeros(), cv::Vec3d::zeros(),
                      camera_matrix_, dist_coeffs_, projected);

    // 计算误差
    for (int i = 0; i < 4; ++i) {
      double dx = projected[i].x - obs.corners[i].x;
      double dy = projected[i].y - obs.corners[i].y;
      total_error += std::sqrt(dx * dx + dy * dy);
      total_points++;
    }
  }

  return total_points > 0 ? total_error / total_points : 999.0;
}

double RelocalizationValidator::computeMarkerDistanceError(
    const std::vector<MarkerObservation>& observations,
    const MarkerMapLoader& map,
    const Eigen::Isometry3d& T_map_cam) const {
  double max_error = 0.0;

  for (size_t i = 0; i < observations.size(); ++i) {
    for (size_t j = i + 1; j < observations.size(); ++j) {
      MarkerItem item_i, item_j;
      if (!map.getMarker(observations[i].id, item_i) ||
          !map.getMarker(observations[j].id, item_j)) continue;

      // 地图中的距离
      double dist_map = (item_i.T_map_marker.translation() -
                         item_j.T_map_marker.translation()).norm();

      // 通过当前帧估计的距离
      // T_map_marker_i_est = T_map_cam * T_cam_marker_i
      Eigen::Isometry3d T_map_marker_i = T_map_cam * observations[i].T_cam_marker;
      Eigen::Isometry3d T_map_marker_j = T_map_cam * observations[j].T_cam_marker;
      double dist_est = (T_map_marker_i.translation() -
                         T_map_marker_j.translation()).norm();

      double error = std::abs(dist_map - dist_est);
      max_error = std::max(max_error, error);
    }
  }

  return max_error;
}

std::pair<double, double> RelocalizationValidator::computeMotionJump(
    const Eigen::Isometry3d& current_pose,
    const Eigen::Isometry3d& last_pose) const {
  double trans_jump = (current_pose.translation() - last_pose.translation()).norm();

  Eigen::Matrix3d R_rel = last_pose.rotation().transpose() * current_pose.rotation();
  double trace = R_rel.trace();
  trace = std::max(-1.0, std::min(3.0, trace));
  double rot_rad = std::acos((trace - 1.0) / 2.0);
  double rot_jump = rot_rad * 180.0 / M_PI;

  return {trans_jump, rot_jump};
}

std::vector<cv::Point3d> RelocalizationValidator::computeMarkerCorners(
    const MarkerItem& item) const {
  double half = item.size / 2.0;

  std::vector<Eigen::Vector3d> corners_marker = {
    {-half, -half, 0},
    { half, -half, 0},
    { half,  half, 0},
    {-half,  half, 0}
  };

  std::vector<cv::Point3d> corners_map;
  for (const auto& p : corners_marker) {
    Eigen::Vector3d p_map = item.T_map_marker * p;
    corners_map.emplace_back(p_map.x(), p_map.y(), p_map.z());
  }

  return corners_map;
}

}  // namespace aruco_reloc
