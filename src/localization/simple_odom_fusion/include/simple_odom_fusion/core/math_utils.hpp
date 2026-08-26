#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>

namespace simple_odom_fusion {

using Vec3d = Eigen::Vector3d;
using Quatd = Eigen::Quaterniond;
using Mat6d = Eigen::Matrix<double, 6, 6>;

struct Pose3d {
  Vec3d position = Vec3d::Zero();
  Quatd orientation = Quatd::Identity();
};

inline double clamp(double value, double low, double high) {
  return std::max(low, std::min(high, value));
}

inline bool isFinite(const Vec3d& v) {
  return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
}

inline bool isFinite(const Quatd& q) {
  return std::isfinite(q.w()) && std::isfinite(q.x()) && std::isfinite(q.y()) &&
         std::isfinite(q.z()) && q.norm() > 1e-9;
}

inline Pose3d inverse(const Pose3d& pose) {
  Pose3d out;
  out.orientation = pose.orientation.normalized().conjugate();
  out.position = -(out.orientation * pose.position);
  return out;
}

inline Pose3d compose(const Pose3d& a, const Pose3d& b) {
  Pose3d out;
  out.orientation = (a.orientation.normalized() * b.orientation.normalized()).normalized();
  out.position = a.position + a.orientation.normalized() * b.position;
  return out;
}

inline Pose3d between(const Pose3d& from, const Pose3d& to) {
  return compose(inverse(from), to);
}

inline Pose3d interpolate(const Pose3d& a, const Pose3d& b, double alpha) {
  alpha = clamp(alpha, 0.0, 1.0);
  Pose3d out;
  out.position = (1.0 - alpha) * a.position + alpha * b.position;
  out.orientation = a.orientation.normalized().slerp(alpha, b.orientation.normalized()).normalized();
  return out;
}

inline Pose3d blend(const Pose3d& from, const Pose3d& to, double gain) {
  return interpolate(from, to, clamp(gain, 0.0, 1.0));
}

}  // namespace simple_odom_fusion
