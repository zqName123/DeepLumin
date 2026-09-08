/**
 * @file relocalization_solver.cpp
 * @brief 重定位求解核心（单 marker 为主，可选多 marker 联合解算）
 */

#include <aruco_relocalization/solver/relocalization_solver.hpp>

#include <ros/ros.h>

#include <chrono>
#include <cmath>

namespace aruco_reloc {

bool RelocalizationSolver::initialize(const SolverConfig& config, const cv::Mat& camera_matrix,
                                      const cv::Mat& dist_coeffs,
                                      const Eigen::Isometry3d& T_base_cam) {
  config_ = config;
  camera_matrix_ = camera_matrix.clone();
  dist_coeffs_ = dist_coeffs.clone();
  T_base_cam_ = T_base_cam;
  return true;
}

SolveResult RelocalizationSolver::solve(const FrameDetection& detection,
                                        const MarkerMapLoader& map) const {
  auto start = std::chrono::steady_clock::now();

  SolveResult result;

  // 1. ID 匹配过滤
  std::vector<MarkerObservation> matched_obs;
  std::vector<MarkerItem> matched_items;
  for (const auto& obs : detection.markers) {
    MarkerItem item;
    if (map.getMarker(obs.id, item)) {
      matched_obs.push_back(obs);
      matched_items.push_back(item);
    }
  }

  // 2. 检查最小 marker 数量
  if (static_cast<int>(matched_obs.size()) < config_.min_markers) {
    result.failure_reason = "not_enough_markers (" +
                           std::to_string(matched_obs.size()) + "/" +
                           std::to_string(config_.min_markers) + ")";
    auto end = std::chrono::steady_clock::now();
    result.solve_time_ms = std::chrono::duration<double, std::milli>(end - start).count();
    return result;
  }

  // 3. 单 marker 解算：对每个 marker 单独求解，选择最优结果
  SolveResult best_single;
  best_single.success = false;
  best_single.reproj_error = 1e9;

  for (size_t i = 0; i < matched_obs.size(); ++i) {
    auto single_result = solveSingleMarker(matched_obs[i], matched_items[i]);
    if (!single_result.success) continue;

    // 计算重投影误差
    single_result.reproj_error = computeReprojError(
        single_result.T_map_cam, matched_obs[i], matched_items[i]);

    // 转换到 base_link
    single_result.T_map_base = single_result.T_map_cam * T_base_cam_.inverse();
    single_result.marker_id = matched_obs[i].id;
    single_result.marker_ids = {matched_obs[i].id};

    // 选择重投影误差最小的结果
    if (single_result.reproj_error < best_single.reproj_error) {
      best_single = single_result;
    }
  }

  // 4. 可选：多 marker 联合 PnP 解算
  if (config_.use_multi_marker && matched_obs.size() >= 2) {
    auto multi_result = solveMultiMarker(matched_obs, matched_items);
    if (multi_result.success) {
      multi_result.T_map_base = multi_result.T_map_cam * T_base_cam_.inverse();
      multi_result.marker_ids.clear();
      for (const auto& obs : matched_obs) {
        multi_result.marker_ids.push_back(obs.id);
      }

      // 如果多 marker 结果更好，使用多 marker 结果
      // 这里用 inlier 数量作为评判标准
      if (best_single.success && multi_result.inlier_indices.size() > 0) {
        ROS_INFO("[aruco_reloc] multi marker solve: %zu markers, %zu inliers, "
                 "single best: marker_id=%d, reproj=%.2f",
                 matched_obs.size(), multi_result.inlier_indices.size(),
                 best_single.marker_id, best_single.reproj_error);
      }
    }
  }

  // 5. 返回最优结果
  if (best_single.success) {
    result = best_single;
  } else if (matched_obs.size() >= 2 && config_.use_multi_marker) {
    // 如果单 marker 全部失败，尝试多 marker 联合解算
    result = solveMultiMarker(matched_obs, matched_items);
    if (result.success) {
      result.T_map_base = result.T_map_cam * T_base_cam_.inverse();
      result.marker_ids.clear();
      for (const auto& obs : matched_obs) {
        result.marker_ids.push_back(obs.id);
      }
    }
  } else {
    result.failure_reason = "all_single_marker_failed";
  }

  auto end = std::chrono::steady_clock::now();
  result.solve_time_ms = std::chrono::duration<double, std::milli>(end - start).count();
  return result;
}

SolveResult RelocalizationSolver::solveSingleMarker(const MarkerObservation& obs,
                                                    const MarkerItem& map_item) const {
  SolveResult result;

  // T_map_cam = T_map_marker * T_cam_marker^(-1)
  result.T_map_cam = map_item.T_map_marker * obs.T_cam_marker.inverse();
  result.success = true;
  result.num_markers_used = 1;
  result.marker_id = obs.id;

  return result;
}

SolveResult RelocalizationSolver::solveMultiMarker(
    const std::vector<MarkerObservation>& observations,
    const std::vector<MarkerItem>& map_items) const {
  SolveResult result;

  // 构建 2D-3D 点对
  std::vector<cv::Point3d> object_points;
  std::vector<cv::Point2d> image_points;

  for (size_t i = 0; i < observations.size(); ++i) {
    auto corners_3d = computeMarkerCorners(map_items[i]);
    for (int j = 0; j < 4; ++j) {
      object_points.push_back(corners_3d[j]);
      image_points.push_back(cv::Point2d(observations[i].corners[j].x,
                                         observations[i].corners[j].y));
    }
  }

  cv::Mat rvec, tvec;
  bool solve_ok = false;

  if (config_.use_ransac) {
    cv::Mat inliers;
    solve_ok = cv::solvePnPRansac(object_points, image_points, camera_matrix_, dist_coeffs_,
                                  rvec, tvec, false,
                                  config_.ransac_iterations,
                                  config_.ransac_reproj_error,
                                  config_.ransac_confidence,
                                  inliers,
                                  cv::SOLVEPNP_ITERATIVE);
    if (solve_ok && !inliers.empty()) {
      result.inlier_indices.reserve(inliers.rows);
      for (int i = 0; i < inliers.rows; ++i) {
        result.inlier_indices.push_back(inliers.at<int>(i));
      }
    }
  } else {
    solve_ok = cv::solvePnP(object_points, image_points, camera_matrix_, dist_coeffs_,
                            rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
  }

  if (!solve_ok) {
    result.failure_reason = "pnp_failed";
    return result;
  }

  // 转换为 Isometry3d
  cv::Mat rmat;
  cv::Rodrigues(rvec, rmat);

  result.T_map_cam = Eigen::Isometry3d::Identity();
  result.T_map_cam(0, 3) = tvec.at<double>(0);
  result.T_map_cam(1, 3) = tvec.at<double>(1);
  result.T_map_cam(2, 3) = tvec.at<double>(2);
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      result.T_map_cam(i, j) = rmat.at<double>(i, j);
    }
  }

  result.success = true;
  result.num_markers_used = static_cast<int>(observations.size());

  ROS_INFO("[aruco_reloc] multi marker solve: %zu markers, %zu inliers",
            observations.size(), result.inlier_indices.size());

  return result;
}

double RelocalizationSolver::computeReprojError(const Eigen::Isometry3d& T_map_cam,
                                                const MarkerObservation& obs,
                                                const MarkerItem& map_item) const {
  // 计算 marker 在 map 下的 3D 角点
  auto corners_3d = computeMarkerCorners(map_item);

  // 转换到相机坐标系
  Eigen::Isometry3d T_cam_map = T_map_cam.inverse();
  std::vector<cv::Point3d> corners_cam;
  for (const auto& p : corners_3d) {
    Eigen::Vector3d p_cam = T_cam_map * Eigen::Vector3d(p.x, p.y, p.z);
    corners_cam.emplace_back(p_cam.x(), p_cam.y(), p_cam.z());
  }

  // 投影到图像
  std::vector<cv::Point2d> projected;
  cv::projectPoints(corners_cam, cv::Vec3d::zeros(), cv::Vec3d::zeros(),
                    camera_matrix_, dist_coeffs_, projected);

  // 计算平均重投影误差
  double total_error = 0.0;
  for (int i = 0; i < 4; ++i) {
    double dx = projected[i].x - obs.corners[i].x;
    double dy = projected[i].y - obs.corners[i].y;
    total_error += std::sqrt(dx * dx + dy * dy);
  }

  return total_error / 4.0;
}

std::vector<cv::Point3d> RelocalizationSolver::computeMarkerCorners(
    const MarkerItem& item) const {
  double half = item.size / 2.0;

  // marker 坐标系下的 4 个角点
  std::vector<Eigen::Vector3d> corners_marker = {
    {-half, -half, 0},
    { half, -half, 0},
    { half,  half, 0},
    {-half,  half, 0}
  };

  // 转换到 map 坐标系
  std::vector<cv::Point3d> corners_map;
  for (const auto& p : corners_marker) {
    Eigen::Vector3d p_map = item.T_map_marker * p;
    corners_map.emplace_back(p_map.x(), p_map.y(), p_map.z());
  }

  return corners_map;
}

}  // namespace aruco_reloc
