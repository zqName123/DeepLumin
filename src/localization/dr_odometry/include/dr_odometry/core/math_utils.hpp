#pragma once

/**
 * @file math_utils.hpp
 * @brief DR 核心用到的姿态/反对称等小型数学工具（头文件内联）。
 */

#include "dr_odometry/core/types.hpp"

#include <cmath>

namespace dr_odometry {
namespace math {

/** @brief 向量反对称矩阵 [v]_×，用于误差状态雅可比。 */
inline Mat3d skew(const Vec3d& v) {
  Mat3d m;
  m << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
  return m;
}

/**
 * @brief 角速度积分得到姿态增量四元数：q(ω·dt)。
 * 小角度用近似四元数；否则用 AngleAxis。返回已归一化。
 */
inline Quatd deltaQuat(const Vec3d& omega, double dt) {
  const Vec3d half = 0.5 * omega * dt;
  const double angle = half.norm();
  if (angle < 1e-12) {
    return Quatd(1.0, half.x(), half.y(), half.z()).normalized();
  }
  return Quatd(Eigen::AngleAxisd(2.0 * angle, half / angle)).normalized();
}

/** @brief 将角度规范到 (-π, π]。 */
inline double normalizeAngle(double angle) {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

/** @brief 从四元数提取 yaw（atan2(R10, R00)，ENU）。 */
inline double yawFromQuat(const Quatd& q) {
  const Mat3d R = q.toRotationMatrix();
  return std::atan2(R(1, 0), R(0, 0));
}

/** @brief 仅绕 Z 轴的 yaw → 四元数（roll=pitch=0）。 */
inline Quatd quatFromYaw(double yaw) {
  return Quatd(Eigen::AngleAxisd(yaw, Vec3d::UnitZ()));
}

}  // namespace math
}  // namespace dr_odometry
