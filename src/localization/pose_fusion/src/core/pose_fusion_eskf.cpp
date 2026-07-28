#include "pose_fusion/core/pose_fusion_eskf.hpp"

#include <Eigen/Cholesky>

#include <cmath>

namespace pose_fusion {

void PoseFusionEskf::reset(const Pose3d& pose, const Vec3d& velocity, const Mat6d& covariance) {
  pose_ = pose;
  pose_.orientation.normalize();
  velocity_ = velocity;
  covariance_ = sanitizeCovariance(covariance, Mat6d::Identity());
}

void PoseFusionEskf::predict(const Pose3d& delta, const Vec3d& velocity, double dt, const PredictionConfig& config) {
  pose_ = compose(pose_, delta);
  pose_.orientation.normalize();
  velocity_ = velocity;

  const double safe_dt = std::max(1e-3, dt);
  Mat6d q = Mat6d::Zero();
  const double pos_q = config.process_noise_position * config.process_noise_position * safe_dt;
  const double yaw_q = std::pow(config.process_noise_yaw_deg * M_PI / 180.0, 2.0) * safe_dt;
  q(0, 0) = pos_q;
  q(1, 1) = pos_q;
  q(2, 2) = pos_q;
  q(3, 3) = yaw_q;
  q(4, 4) = yaw_q;
  q(5, 5) = yaw_q;
  covariance_ = symmetrize(covariance_ + q);
}

bool PoseFusionEskf::updatePose(const Pose3d& observed, const Mat6d& observation_covariance, double weight) {
  if (!isFinite(observed.position) || !isFinite(observed.orientation)) {
    return false;
  }
  weight = clamp(weight, 1e-3, 10.0);

  Mat6d fallback_r = Mat6d::Identity();
  fallback_r.block<3, 3>(0, 0) *= 0.25;
  fallback_r.block<3, 3>(3, 3) *= std::pow(5.0 * M_PI / 180.0, 2.0);
  Mat6d r_cov = sanitizeCovariance(observation_covariance, fallback_r) / weight;

  const Vec6d residual = poseResidual(pose_, observed);
  const Mat6d innovation_cov = symmetrize(covariance_ + r_cov);
  Eigen::LDLT<Mat6d> ldlt(innovation_cov);
  if (ldlt.info() != Eigen::Success) {
    return false;
  }
  const Mat6d k = covariance_ * ldlt.solve(Mat6d::Identity());
  const Vec6d dx = k * residual;

  pose_.position += dx.segment<3>(0);
  pose_.orientation = (pose_.orientation.normalized() * deltaQuat(dx.segment<3>(3))).normalized();

  const Mat6d i = Mat6d::Identity();
  covariance_ = symmetrize((i - k) * covariance_ * (i - k).transpose() + k * r_cov * k.transpose());
  return isFinite(pose_.position) && isFinite(pose_.orientation);
}

Mat6d PoseFusionEskf::symmetrize(const Mat6d& mat) {
  Mat6d out = 0.5 * (mat + mat.transpose());
  for (int i = 0; i < 6; ++i) {
    if (!std::isfinite(out(i, i)) || out(i, i) < 1e-9) {
      out(i, i) = 1e-9;
    }
  }
  return out;
}

Mat6d PoseFusionEskf::sanitizeCovariance(const Mat6d& input, const Mat6d& fallback) {
  Mat6d out = input;
  bool valid = true;
  for (int r = 0; r < 6; ++r) {
    for (int c = 0; c < 6; ++c) {
      if (!std::isfinite(out(r, c))) {
        valid = false;
      }
    }
  }
  if (!valid) {
    out = fallback;
  }
  out = symmetrize(out);
  for (int i = 0; i < 6; ++i) {
    if (out(i, i) < 1e-9) {
      out(i, i) = fallback(i, i) > 1e-9 ? fallback(i, i) : 1e-6;
    }
  }
  return out;
}

Quatd PoseFusionEskf::deltaQuat(const Vec3d& delta_theta) {
  const double angle = delta_theta.norm();
  if (angle < 1e-12) {
    return Quatd::Identity();
  }
  return Quatd(Eigen::AngleAxisd(angle, delta_theta / angle)).normalized();
}

}  // namespace pose_fusion
