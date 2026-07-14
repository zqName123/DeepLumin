/**
 * @file math_utils.hpp
 * @brief 姿态/几何小工具（DR ESKF 预测与观测雅可比会用到）
 *
 * DR 相关：
 *   skew(v)         → [v]_×，用于 F 矩阵、轮速观测 H 中的姿态耦合项
 *   deltaQuat(ω,dt) → 由角速度积分出的增量四元数（右乘到 orientation）
 */
#pragma once

#include "localization/common/types.hpp"
#include <cmath>

namespace localization {
namespace math {

/** 反对称矩阵 [v]_×，满足 [v]_× a = v × a */
inline Mat3d skew(const Vec3d& v) {
    Mat3d m;
    m << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
    return m;
}

/**
 * @brief 角速度积分成增量四元数：q ← q ⊗ deltaQuat(ω, dt)
 * @param omega 角速度 (rad/s)，通常已去 bias
 * @param dt    时间间隔 (s)；injectError 时传 dt=1，把小角度误差向量当 ω·dt
 */
inline Quatd deltaQuat(const Vec3d& omega, double dt) {
    const Vec3d half = 0.5 * omega * dt;
    const double angle = half.norm();
    if (angle < 1e-12) {
        // 小角度近似：q ≈ [1, θ/2]
        return Quatd(1.0, half.x(), half.y(), half.z());
    }
    const Vec3d axis = half / angle;
    return Quatd(Eigen::AngleAxisd(angle, axis));
}

inline Vec3d quatToEulerZYX(const Quatd& q) {
    const Eigen::Matrix3d R = q.toRotationMatrix();
    const double yaw = std::atan2(R(1, 0), R(0, 0));
    const double pitch = std::asin(-R(2, 0));
    const double roll = std::atan2(R(2, 1), R(2, 2));
    return Vec3d(roll, pitch, yaw);
}

inline Eigen::Matrix4d poseToMatrix(const Pose& pose) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = pose.orientation.toRotationMatrix();
    T.block<3, 1>(0, 3) = pose.position;
    return T;
}

inline Pose matrixToPose(const Eigen::Matrix4d& T) {
    Pose pose;
    pose.position = T.block<3, 1>(0, 3);
    pose.orientation = Quatd(T.block<3, 3>(0, 0));
    return pose;
}

}  // namespace math
}  // namespace localization
