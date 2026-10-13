#pragma once

/**
 * @file types.hpp
 * @brief localization_v3 核心数据类型与位姿数学工具（纯 C++，不依赖 ROS）
 *
 * 位姿约定：
 * - Pose3d 表示 "parent -> child" 的刚体变换（平移 + 四元数旋转）
 * - compose(a, b) = a ⊕ b（先 b 后 a 的复合变换）
 * - inverse(a)    = a 的逆变换
 *
 * 融合公式（与 v2 一致，v3 不改变）：
 *   map_odom = map_base ⊕ (odom_base)⁻¹
 *   map_base = map_odom ⊕ odom_base
 */

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>

namespace localization_v3 {

using Vec3d = Eigen::Vector3d;
using Quatd = Eigen::Quaterniond;

struct Pose3d {
  Vec3d position = Vec3d::Zero();
  Quatd orientation = Quatd::Identity();
};

inline bool isFinite(const Vec3d& v) {
  return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
}

inline bool isFinite(const Quatd& q) {
  return std::isfinite(q.w()) && std::isfinite(q.x()) && std::isfinite(q.y()) &&
         std::isfinite(q.z()) && q.norm() > 1e-9;
}

inline bool isFinite(const Pose3d& pose) {
  return isFinite(pose.position) && isFinite(pose.orientation);
}

inline Pose3d identity() { return Pose3d(); }

inline bool isIdentity(const Pose3d& pose, double tol = 1e-9) {
  return pose.position.norm() < tol &&
         std::abs(pose.orientation.normalized().w()) > 1.0 - tol;
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

/// 两个位姿绕 z 轴的偏航角差（度，取最小角差，范围 [-180, 180]）
inline double yawDiffDeg(const Pose3d& a, const Pose3d& b) {
  constexpr double kPi = 3.14159265358979323846;
  const Quatd rel = a.orientation.normalized().conjugate() * b.orientation.normalized();
  const Eigen::Matrix3d R = rel.toRotationMatrix();
  double yaw = std::atan2(R(1, 0), R(0, 0));
  yaw = std::fmod(yaw + kPi, 2.0 * kPi);
  if (yaw < 0.0) {
    yaw += 2.0 * kPi;
  }
  yaw -= kPi;
  return yaw * 180.0 / kPi;
}

/// 两个位姿之间的平移距离（米）
inline double translationDiff(const Pose3d& a, const Pose3d& b) {
  return (a.position - b.position).norm();
}

}  // namespace localization_v3
