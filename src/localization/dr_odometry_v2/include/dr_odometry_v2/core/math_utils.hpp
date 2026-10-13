#pragma once

/**
 * @file math_utils.hpp
 * @brief 四元数 / 旋转对数映射 / skew / WGS84 转换（纯 C++）
 */

#include <cmath>

#include "dr_odometry_v2/core/types.hpp"

namespace dr_odometry_v2 {
namespace math {

constexpr double kPi = 3.14159265358979323846;

inline double deg2rad(double deg) { return deg * kPi / 180.0; }
inline double rad2deg(double rad) { return rad * 180.0 / kPi; }

inline Mat3d skew(const Vec3d& v) {
  Mat3d m;
  m << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
  return m;
}

inline Quatd quatFromYaw(double yaw) {
  return Quatd(Eigen::AngleAxisd(yaw, Vec3d::UnitZ()));
}

inline Quatd quatFromRpy(double roll, double pitch, double yaw) {
  return Quatd(Eigen::AngleAxisd(yaw, Vec3d::UnitZ()) *
               Eigen::AngleAxisd(pitch, Vec3d::UnitY()) *
               Eigen::AngleAxisd(roll, Vec3d::UnitX()))
      .normalized();
}

/// ZYX 欧拉角（roll, pitch, yaw）
inline Vec3d rpyFromQuat(const Quatd& q_in) {
  const Eigen::Matrix3d R = q_in.normalized().toRotationMatrix();
  const double pitch = std::asin(-R(2, 0));
  const double roll = std::atan2(R(2, 1), R(2, 2));
  const double yaw = std::atan2(R(1, 0), R(0, 0));
  return Vec3d(roll, pitch, yaw);
}

inline double yawFromQuat(const Quatd& q_in) {
  const Eigen::Matrix3d R = q_in.normalized().toRotationMatrix();
  return std::atan2(R(1, 0), R(0, 0));
}

/// 小角度旋转向量 → 四元数（一阶）
inline Quatd deltaQuat(const Vec3d& rotvec, double scale = 1.0) {
  const Vec3d half = 0.5 * scale * rotvec;
  return Quatd(1.0, half.x(), half.y(), half.z()).normalized();
}

/// 旋转对数映射：R → 旋转向量（短角度稳定）
inline Vec3d logRot(const Mat3d& R) {
  const double cos_theta = std::max(-1.0, std::min(1.0, (R.trace() - 1.0) * 0.5));
  const double theta = std::acos(cos_theta);
  if (theta < 1e-9) {
    return Vec3d(R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1)) * 0.5;
  }
  const double factor = theta / (2.0 * std::sin(theta));
  return factor * Vec3d(R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1));
}

inline Vec3d logQuat(const Quatd& a, const Quatd& b) {
  // Log(Ra^T Rb)
  return logRot((a.normalized().toRotationMatrix().transpose()) *
                b.normalized().toRotationMatrix());
}

inline double normalizeAngle(double a) {
  a = std::fmod(a + kPi, 2.0 * kPi);
  if (a < 0.0) {
    a += 2.0 * kPi;
  }
  return a - kPi;
}

inline Quatd slerp(const Quatd& a, const Quatd& b, double t) {
  return a.slerp(t, b).normalized();
}

inline bool finiteVec(const Vec3d& v) {
  return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
}

template <typename Derived>
inline bool finiteEigen(const Eigen::MatrixBase<Derived>& m) {
  for (Eigen::Index r = 0; r < m.rows(); ++r) {
    for (Eigen::Index c = 0; c < m.cols(); ++c) {
      if (!std::isfinite(m(r, c))) {
        return false;
      }
    }
  }
  return true;
}

// ---------------- WGS84：LLA ↔ ECEF ↔ ENU ----------------

constexpr double kEarthA = 6378137.0;
constexpr double kEarthE2 = 6.69437999014e-3;

inline Vec3d llaToEcef(double lat_deg, double lon_deg, double alt) {
  const double lat = deg2rad(lat_deg);
  const double lon = deg2rad(lon_deg);
  const double sin_lat = std::sin(lat);
  const double cos_lat = std::cos(lat);
  const double sin_lon = std::sin(lon);
  const double cos_lon = std::cos(lon);
  const double N = kEarthA / std::sqrt(1.0 - kEarthE2 * sin_lat * sin_lat);
  return Vec3d((N + alt) * cos_lat * cos_lon, (N + alt) * cos_lat * sin_lon,
               (N * (1.0 - kEarthE2) + alt) * sin_lat);
}

inline Vec3d llaToEnu(double lat_deg, double lon_deg, double alt, double origin_lat_deg,
                      double origin_lon_deg, const Vec3d& origin_ecef) {
  const Vec3d d = llaToEcef(lat_deg, lon_deg, alt) - origin_ecef;
  const double sin_lat = std::sin(deg2rad(origin_lat_deg));
  const double cos_lat = std::cos(deg2rad(origin_lat_deg));
  const double sin_lon = std::sin(deg2rad(origin_lon_deg));
  const double cos_lon = std::cos(deg2rad(origin_lon_deg));
  Vec3d enu;
  enu.x() = -sin_lon * d.x() + cos_lon * d.y();
  enu.y() = -sin_lat * cos_lon * d.x() - sin_lat * sin_lon * d.y() + cos_lat * d.z();
  enu.z() = cos_lat * cos_lon * d.x() + cos_lat * sin_lon * d.y() + sin_lat * d.z();
  return enu;
}

}  // namespace math
}  // namespace dr_odometry_v2
