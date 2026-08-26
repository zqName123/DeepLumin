#include "localization_manager/core/map_height_provider.hpp"

#include <pcl/io/pcd_io.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace localization_manager {

bool MapHeightProvider::initialize(const InitialPoseCompletionConfig& config) {
  config_ = config;
  ready_ = false;
  map_.reset(new CloudT());

  if (!config_.enabled) {
    return false;
  }
  if (config_.pcd_path.empty()) {
    return false;
  }

  CloudPtr raw(new CloudT());
  if (pcl::io::loadPCDFile<PointT>(config_.pcd_path, *raw) != 0 || raw->empty()) {
    map_.reset(new CloudT());
    return false;
  }

  map_->reserve(raw->size());
  for (const auto& pt : raw->points) {
    if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z)) {
      map_->push_back(pt);
    }
  }
  map_->width = static_cast<uint32_t>(map_->size());
  map_->height = 1;
  map_->is_dense = true;
  ready_ = !map_->empty();
  return ready_;
}

HeightQueryResult MapHeightProvider::query(const geometry_msgs::PoseWithCovarianceStamped& pose) const {
  HeightQueryResult out;
  out.z = config_.default_z;
  out.z_std = config_.default_z_std;
  out.source = "default";

  if (!ready_ || !map_ || map_->empty()) {
    out.reason = "map_not_ready";
    out.success = config_.allow_default_z_fallback;
    return out;
  }

  const double x0 = pose.pose.pose.position.x;
  const double y0 = pose.pose.pose.position.y;
  const double yaw = yawFromPose(pose);
  if (!std::isfinite(x0) || !std::isfinite(y0) || !std::isfinite(yaw)) {
    out.reason = "invalid_initialpose_xy_yaw";
    return out;
  }

  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  std::vector<double> near_zs;
  std::vector<double> fov_zs;
  near_zs.reserve(2048);
  fov_zs.reserve(4096);
  double raw_min_z = std::numeric_limits<double>::infinity();
  double raw_max_z = -std::numeric_limits<double>::infinity();
  const double radius = std::max(0.1, config_.height_search_radius);
  const double radius_sq = radius * radius;

  for (const auto& pt : map_->points) {
    const double dx = static_cast<double>(pt.x) - x0;
    const double dy = static_cast<double>(pt.y) - y0;
    const double z = static_cast<double>(pt.z);
    if ((dx * dx + dy * dy) <= radius_sq) {
      near_zs.push_back(z);
      raw_min_z = std::min(raw_min_z, z);
      raw_max_z = std::max(raw_max_z, z);
      continue;
    }

    const double local_x = c * dx + s * dy;
    const double local_y = -s * dx + c * dy;
    if (local_x < -config_.backward || local_x > config_.forward ||
        local_y < -config_.right || local_y > config_.left) {
      continue;
    }
    fov_zs.push_back(z);
    raw_min_z = std::min(raw_min_z, z);
    raw_max_z = std::max(raw_max_z, z);
  }

  std::vector<double>* zs = &near_zs;
  std::string query_source = "near_radius";
  if (static_cast<int>(near_zs.size()) < config_.min_map_points) {
    zs = &fov_zs;
    query_source = "fov_fallback";
  }

  out.point_count = static_cast<int>(zs->size());
  if (out.point_count < config_.min_map_points) {
    out.reason = "insufficient_map_height_points:" + query_source;
    out.success = config_.allow_default_z_fallback;
    return out;
  }

  std::sort(zs->begin(), zs->end());
  const auto quantile = [&](double q) {
    const size_t idx = static_cast<size_t>(std::round(q * static_cast<double>(zs->size() - 1)));
    return (*zs)[std::min(idx, zs->size() - 1)];
  };
  const double low_z = quantile(0.05);
  const double high_z = quantile(0.95);
  out.min_z = std::isfinite(raw_min_z) ? raw_min_z : low_z;
  out.max_z = std::isfinite(raw_max_z) ? raw_max_z : high_z;
  if ((high_z - low_z) > config_.max_height_span) {
    out.reason = "height_percentile_span_too_large:" + query_source;
    out.success = config_.allow_default_z_fallback;
    return out;
  }

  out.z = estimateZ(*zs, low_z, high_z) + config_.base_height_offset;
  out.z_std = config_.map_height_z_std;
  out.source = "map_height:" + query_source + ":" + config_.height_method;
  out.success = true;
  out.reason = "map_height_ok";
  return out;
}

geometry_msgs::PoseWithCovarianceStamped MapHeightProvider::completePose(
    const geometry_msgs::PoseWithCovarianceStamped& pose,
    const std::string& frame_id,
    HeightQueryResult* result) const {
  HeightQueryResult local_result = query(pose);
  if (result) {
    *result = local_result;
  }

  geometry_msgs::PoseWithCovarianceStamped out = pose;
  out.header.frame_id = frame_id;
  if (local_result.success) {
    out.pose.pose.position.z = local_result.z;
    out.pose.covariance[14] = local_result.z_std * local_result.z_std;
  }
  return out;
}

double MapHeightProvider::yawFromPose(const geometry_msgs::PoseWithCovarianceStamped& pose) {
  const auto& q = pose.pose.pose.orientation;
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

double MapHeightProvider::estimateZ(std::vector<double>& values, double min_z, double max_z) const {
  if (values.empty()) {
    return config_.default_z;
  }
  if (config_.height_method == "mid_range") {
    return 0.5 * (min_z + max_z);
  }
  if (config_.height_method == "mean") {
    double sum = 0.0;
    for (double z : values) sum += z;
    return sum / static_cast<double>(values.size());
  }
  const size_t mid = values.size() / 2;
  return values.size() % 2 == 0 ? 0.5 * (values[mid - 1] + values[mid]) : values[mid];
}

}  // namespace localization_manager
