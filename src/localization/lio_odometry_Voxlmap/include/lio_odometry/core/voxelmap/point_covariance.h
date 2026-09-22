#pragma once
#include "lio_odometry/core/eskf.h"
namespace slam_odometry {
inline Mat3d SensorCovarianceWorld(const NavState& s, const Mat3d& sensor) {
    const Mat3d R = (s.rot_ * s.offset_R_lidar_).matrix();
    return R * sensor * R.transpose();
}
inline Mat3d PointCovarianceWorld(const NavState& s, const ESKF::CovType& P,
                                  const Vec3d& point_lidar, const Mat3d& sensor,
                                  bool estimate_extrinsics) {
    const Vec3d pi = s.offset_R_lidar_ * point_lidar + s.offset_t_lidar_;
    Eigen::Matrix<double, 3, 23> J = Eigen::Matrix<double, 3, 23>::Zero();
    J.block<3,3>(0,0).setIdentity(); // NavState: position BEFORE orientation.
    J.block<3,3>(0,3) = -s.rot_.matrix() * SO3::hat(pi);
    if (estimate_extrinsics) {
        J.block<3,3>(0,6) = -(s.rot_ * s.offset_R_lidar_).matrix() * SO3::hat(point_lidar);
        J.block<3,3>(0,9) = s.rot_.matrix();
    }
    const Mat3d result = SensorCovarianceWorld(s, sensor) + J * P * J.transpose();
    return 0.5 * (result + result.transpose());
}
} // namespace slam_odometry
