#pragma once

/**
 * @file pose_fusion_eskf.hpp
 * @brief ROS-free local pose ESKF for fusing DR prediction and SLAM pose observations.
 *
 * State is T_odom_base and a 6x6 error covariance [position, rotation]. DR provides relative
 * pose increments for prediction. SLAM is first aligned into the fusion odom frame and then used
 * as an absolute local pose observation.
 */

#include "pose_fusion/common/types.hpp"

namespace pose_fusion {

class PoseFusionEskf {
 public:
  void reset(const Pose3d& pose, const Vec3d& velocity, const Mat6d& covariance);
  void predict(const Pose3d& delta, const Vec3d& velocity, double dt, const PredictionConfig& config);
  bool updatePose(const Pose3d& observed, const Mat6d& observation_covariance, double weight);

  const Pose3d& pose() const { return pose_; }
  const Vec3d& velocity() const { return velocity_; }
  const Mat6d& covariance() const { return covariance_; }

 private:
  static Mat6d symmetrize(const Mat6d& mat);
  static Mat6d sanitizeCovariance(const Mat6d& input, const Mat6d& fallback);
  static Quatd deltaQuat(const Vec3d& delta_theta);

  Pose3d pose_;
  Vec3d velocity_ = Vec3d::Zero();
  Mat6d covariance_ = Mat6d::Identity();
};

}  // namespace pose_fusion
