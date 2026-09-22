#include "slam_mapping_v2/core/gnss_optimizer.hpp"

#include <gtsam/geometry/Point3.h>
#include <gtsam/linear/LossFunctions.h>
#include <gtsam/navigation/GPSFactor.h>

#include "syslog/logger.h"

#include <algorithm>
#include <cmath>

namespace slam_mapping_v2 {

namespace {

constexpr double kEarthRadius = 6378137.0;
constexpr double kEarthFlattening = 1.0 / 298.257223563;
constexpr double kEarthEccentricitySquared = kEarthFlattening * (2.0 - kEarthFlattening);
constexpr double kDegToRad = M_PI / 180.0;

Eigen::Vector3d geodeticToEcef(double lat_rad, double lon_rad, double alt) {
  const double sin_lat = std::sin(lat_rad);
  const double cos_lat = std::cos(lat_rad);
  const double n = kEarthRadius / std::sqrt(1.0 - kEarthEccentricitySquared * sin_lat * sin_lat);
  Eigen::Vector3d ecef;
  ecef.x() = (n + alt) * cos_lat * std::cos(lon_rad);
  ecef.y() = (n + alt) * cos_lat * std::sin(lon_rad);
  ecef.z() = (n * (1.0 - kEarthEccentricitySquared) + alt) * sin_lat;
  return ecef;
}

}  // namespace

void GnssOptimizer::configure(const GnssConfig& cfg) {
  cfg_ = cfg;
  reset();
}

void GnssOptimizer::reset() {
  origin_set_ = false;
  origin_lat_ = 0.0;
  origin_lon_ = 0.0;
  origin_alt_ = 0.0;
  origin_ecef_.setZero();
  alignment_complete_ = false;
  rot_map_enu_.setIdentity();
  trans_map_enu_.setZero();
  alignment_pairs_.clear();
  last_checked_index_ = -1;
  last_added_xy_.setZero();
  factor_count_ = 0;
  rejected_count_ = 0;
  accepted_positions_.clear();
  accepted_factors_.clear();
  rejected_positions_.clear();
  rejected_factors_.clear();
}

void GnssOptimizer::update(const std::vector<Keyframe>& keyframes,
                           gtsam::NonlinearFactorGraph* graph, int latest_index) {
  if (!cfg_.enable || !graph || keyframes.empty()) {
    return;
  }
  latest_index = std::min(latest_index, static_cast<int>(keyframes.size()) - 1);
  for (int i = last_checked_index_ + 1; i <= latest_index; ++i) {
    const Keyframe& kf = keyframes[i];
    if (!kf.gnss.valid || !isReliable(kf)) {
      ++rejected_count_;
      continue;
    }
    if (std::abs(kf.gnss.timestamp - kf.timestamp) > cfg_.max_age_sec) {
      ++rejected_count_;
      continue;
    }
    if (!origin_set_ && !ensureOrigin(kf)) {
      ++rejected_count_;
      continue;
    }
    Vec3d enu;
    if (!toEnu(kf.gnss, &enu)) {
      ++rejected_count_;
      continue;
    }
    enu = correctLeverArm(kf, enu);

    if (!alignment_complete_) {
      alignment_pairs_.emplace_back(enu, kf.pose_map.position);
      if (static_cast<int>(alignment_pairs_.size()) >= cfg_.alignment_frames) {
        if (estimateAlignment()) {
          last_added_xy_ = alignment_pairs_.back().second.head<2>();
        }
      }
      continue;
    }

    Vec3d map_pos;
    if (!toMapFrame(enu, kf, &map_pos)) {
      ++rejected_count_;
      continue;
    }
    if (cfg_.min_spacing_m > 0.0 &&
        (map_pos.head<2>() - last_added_xy_).norm() < cfg_.min_spacing_m) {
      recordRejected(kf.id, map_pos);
      ++rejected_count_;
      continue;
    }
    if (cfg_.consistency_thresh_m > 0.0 &&
        (map_pos - kf.pose_map.position).norm() > cfg_.consistency_thresh_m) {
      recordRejected(kf.id, map_pos);
      ++rejected_count_;
      continue;
    }
    addGpsFactor(kf, map_pos, graph);
    last_added_xy_ = map_pos.head<2>();
    ++factor_count_;
  }
  last_checked_index_ = latest_index;
}

bool GnssOptimizer::isReliable(const Keyframe& kf) const {
  const auto& g = kf.gnss;
  if (g.status < cfg_.min_status) return false;
  if (cfg_.min_nsv > 0 && g.nsv < cfg_.min_nsv) return false;
  if (!cfg_.status_sigma.empty()) {
    bool in_table = false;
    for (const auto& kv : cfg_.status_sigma) {
      if (kv.first == g.status) {
        in_table = true;
        break;
      }
    }
    if (!in_table) return false;
  }
  return true;
}

bool GnssOptimizer::ensureOrigin(const Keyframe& kf) {
  const auto& g = kf.gnss;
  if (std::abs(g.latitude) < 1e-9 && std::abs(g.longitude) < 1e-9) return false;
  origin_lat_ = g.latitude * kDegToRad;
  origin_lon_ = g.longitude * kDegToRad;
  origin_alt_ = g.altitude;
  origin_ecef_ = geodeticToEcef(origin_lat_, origin_lon_, origin_alt_);
  origin_set_ = true;
  return true;
}

bool GnssOptimizer::toEnu(const GnssObservation& gnss, Vec3d* enu) const {
  if (!origin_set_ || !enu) return false;
  const double lat = gnss.latitude * kDegToRad;
  const double lon = gnss.longitude * kDegToRad;
  const Eigen::Vector3d ecef = geodeticToEcef(lat, lon, gnss.altitude);
  const double slat = std::sin(origin_lat_);
  const double clat = std::cos(origin_lat_);
  const double slon = std::sin(origin_lon_);
  const double clon = std::cos(origin_lon_);
  Eigen::Matrix3d r_enu_ecef;
  r_enu_ecef << -slon, clon, 0.0, -clon * slat, -slon * slat, clat, clon * clat, slon * clat, slat;
  *enu = r_enu_ecef * (ecef - origin_ecef_);
  return true;
}

Vec3d GnssOptimizer::correctLeverArm(const Keyframe& kf, const Vec3d& enu) const {
  if (cfg_.lever_arm.norm() < 1e-9) return enu;
  const Mat3d r_map_body = kf.pose_map.orientation.normalized().toRotationMatrix();
  const Mat3d r_enu_map = rot_map_enu_.transpose();
  return enu - r_enu_map * (r_map_body * cfg_.lever_arm);
}

bool GnssOptimizer::estimateAlignment() {
  const std::size_t n = alignment_pairs_.size();
  if (n < 3) return false;

  Eigen::Vector2d mean_enu = Eigen::Vector2d::Zero();
  Eigen::Vector2d mean_map = Eigen::Vector2d::Zero();
  std::vector<double> dz;
  dz.reserve(n);
  for (const auto& pair : alignment_pairs_) {
    mean_enu += pair.first.head<2>();
    mean_map += pair.second.head<2>();
    dz.push_back(pair.second.z() - pair.first.z());
  }
  mean_enu /= static_cast<double>(n);
  mean_map /= static_cast<double>(n);

  double sxx = 0.0, sxy = 0.0, syx = 0.0, syy = 0.0;
  for (const auto& pair : alignment_pairs_) {
    const Eigen::Vector2d pe = pair.first.head<2>() - mean_enu;
    const Eigen::Vector2d pm = pair.second.head<2>() - mean_map;
    sxx += pe.x() * pm.x();
    sxy += pe.x() * pm.y();
    syx += pe.y() * pm.x();
    syy += pe.y() * pm.y();
  }
  const double theta = std::atan2(sxy - syx, sxx + syy);
  const double c = std::cos(theta);
  const double s = std::sin(theta);
  rot_map_enu_.setIdentity();
  rot_map_enu_(0, 0) = c;
  rot_map_enu_(0, 1) = -s;
  rot_map_enu_(1, 0) = s;
  rot_map_enu_(1, 1) = c;
  trans_map_enu_.setZero();
  trans_map_enu_.head<2>() = mean_map - rot_map_enu_.topLeftCorner<2, 2>() * mean_enu;
  const std::size_t mid = n / 2;
  std::nth_element(dz.begin(), dz.begin() + static_cast<std::ptrdiff_t>(mid), dz.end());
  trans_map_enu_.z() = dz[mid];
  alignment_complete_ = true;
  // ROS_INFO("[slam_mapping_v2][gnss] alignment done yaw_deg=%.4f t=(%.3f, %.3f, %.3f)",
  //          std::atan2(rot_map_enu_(1, 0), rot_map_enu_(0, 0)) * 180.0 / M_PI,
  //          trans_map_enu_.x(), trans_map_enu_.y(), trans_map_enu_.z());
  SYS_LOG(syslog::LogSeverity::INFO, true) << "alignment done yaw_deg=" << std::atan2(rot_map_enu_(1, 0), rot_map_enu_(0, 0)) * 180.0 / M_PI
      << " t=(" << trans_map_enu_.x() << ", " << trans_map_enu_.y() << ", " << trans_map_enu_.z() << ")";
  return true;
}

bool GnssOptimizer::getMapEnuTransform(Mat3d* rot, Vec3d* trans) const {
  if (!alignment_complete_ || !rot || !trans) return false;
  *rot = rot_map_enu_;
  *trans = trans_map_enu_;
  return true;
}

bool GnssOptimizer::getEnuOrigin(double* lat_deg, double* lon_deg, double* alt) const {
  if (!origin_set_ || !lat_deg || !lon_deg || !alt) return false;
  *lat_deg = origin_lat_ / kDegToRad;
  *lon_deg = origin_lon_ / kDegToRad;
  *alt = origin_alt_;
  return true;
}

bool GnssOptimizer::toMapFrame(const Vec3d& enu, const Keyframe& kf, Vec3d* map_pos) const {
  if (!alignment_complete_ || !map_pos) return false;
  *map_pos = rot_map_enu_ * enu + trans_map_enu_;
  if (!cfg_.use_gps_elevation) {
    map_pos->z() = kf.pose_map.position.z();
  }
  return true;
}

double GnssOptimizer::positionSigma(const GnssObservation& gnss) const {
  for (const auto& kv : cfg_.status_sigma) {
    if (kv.first == gnss.status) {
      return std::max(kv.second, cfg_.sigma_floor);
    }
  }
  double sigma = cfg_.sigma_unknown;
  if (gnss.status >= cfg_.min_status + 2) {
    sigma = cfg_.sigma_fixed;
  } else if (gnss.status >= cfg_.min_status + 1) {
    sigma = cfg_.sigma_float;
  }
  return std::max(sigma, cfg_.sigma_floor);
}

void GnssOptimizer::addGpsFactor(const Keyframe& kf, const Vec3d& map_pos,
                                 gtsam::NonlinearFactorGraph* graph) {
  const double sigma_xy = positionSigma(kf.gnss);
  const double sigma_z = cfg_.use_gps_elevation ? sigma_xy : std::max(cfg_.z_override_sigma, 1e-6);
  const gtsam::Vector variances =
      (gtsam::Vector(3) << sigma_xy * sigma_xy, sigma_xy * sigma_xy, sigma_z * sigma_z).finished();
  gtsam::SharedNoiseModel noise = gtsam::noiseModel::Diagonal::Variances(variances);
  if (cfg_.robust_delta > 0.0) {
    noise = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Huber::Create(cfg_.robust_delta), noise);
  }
  graph->add(gtsam::GPSFactor(kf.id, gtsam::Point3(map_pos.x(), map_pos.y(), map_pos.z()), noise));
  accepted_positions_.push_back(map_pos);
  accepted_factors_.emplace_back(kf.id, map_pos);
}

void GnssOptimizer::recordRejected(int keyframe_id, const Vec3d& map_pos) {
  rejected_positions_.push_back(map_pos);
  rejected_factors_.emplace_back(keyframe_id, map_pos);
  constexpr std::size_t kMaxRejectedPositions = 1000;
  if (rejected_positions_.size() > kMaxRejectedPositions) {
    rejected_positions_.erase(rejected_positions_.begin());
    rejected_factors_.erase(rejected_factors_.begin());
  }
}

}  // namespace slam_mapping_v2
