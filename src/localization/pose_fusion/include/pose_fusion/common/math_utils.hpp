#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>

namespace pose_fusion {

using Vec3d = Eigen::Vector3d;
using Quatd = Eigen::Quaterniond;
using Mat3d = Eigen::Matrix3d;
using Mat4d = Eigen::Matrix4d;
using Mat6d = Eigen::Matrix<double, 6, 6>;
using Vec6d = Eigen::Matrix<double, 6, 1>;

struct Pose3d {
  Vec3d position = Vec3d::Zero();
  Quatd orientation = Quatd::Identity();
};

inline double clamp(double value, double low, double high) {
  return std::max(low, std::min(high, value));
}

inline double normalizeAngle(double angle) {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

inline double yawFromQuat(const Quatd& q) {
  const Quatd n = q.normalized();
  return std::atan2(2.0 * (n.w() * n.z() + n.x() * n.y()),
                    1.0 - 2.0 * (n.y() * n.y() + n.z() * n.z()));
}

inline Mat4d toMatrix(const Pose3d& pose) {
  Mat4d t = Mat4d::Identity();
  t.block<3, 3>(0, 0) = pose.orientation.normalized().toRotationMatrix();
  t.block<3, 1>(0, 3) = pose.position;
  return t;
}

inline Pose3d fromMatrix(const Mat4d& t) {
  Pose3d pose;
  pose.position = t.block<3, 1>(0, 3);
  pose.orientation = Quatd(t.block<3, 3>(0, 0)).normalized();
  return pose;
}

inline Pose3d inverse(const Pose3d& pose) {
  const Quatd q_inv = pose.orientation.normalized().conjugate();
  Pose3d out;
  out.orientation = q_inv;
  out.position = -(q_inv * pose.position);
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

inline Vec3d rotationLog(const Quatd& q) {
  Quatd n = q.normalized();
  if (n.w() < 0.0) {
    n.coeffs() *= -1.0;
  }
  Eigen::AngleAxisd aa(n);
  return aa.axis() * normalizeAngle(aa.angle());
}

inline Vec6d poseResidual(const Pose3d& predicted, const Pose3d& observed) {
  Vec6d r;
  r.segment<3>(0) = observed.position - predicted.position;
  r.segment<3>(3) = rotationLog(predicted.orientation.normalized().conjugate() *
                                 observed.orientation.normalized());
  return r;
}

inline double yawDiff(const Pose3d& a, const Pose3d& b) {
  return normalizeAngle(yawFromQuat(b.orientation) - yawFromQuat(a.orientation));
}

inline double translationNorm(const Pose3d& a, const Pose3d& b) {
  return (b.position - a.position).norm();
}

inline bool isFinite(const Vec3d& v) {
  return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
}

inline bool isFinite(const Quatd& q) {
  return std::isfinite(q.w()) && std::isfinite(q.x()) &&
         std::isfinite(q.y()) && std::isfinite(q.z()) && q.norm() > 1e-9;
}

}  // namespace pose_fusion
