#pragma once

/**
 * @file gnss_optimizer.hpp
 * @brief GNSS 后端因子管理器（适配 v2 Keyframe/GnssObservation）
 *
 * 流程：可靠性门控 → ENU 原点 → WGS84→ENU → 2D Umeyama 对齐 →
 * 间距/一致性门控 → gtsam::GPSFactor（可选 Huber）。
 */

#include "slam_mapping_v2/common/config.hpp"
#include "slam_mapping_v2/common/types.hpp"

#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <vector>

namespace slam_mapping_v2 {

class GnssOptimizer {
 public:
  void configure(const GnssConfig& cfg);
  void reset();

  /// 增量处理 [last_checked_index_+1, latest_index] 的关键帧，向 graph 添加 GPSFactor
  void update(const std::vector<Keyframe>& keyframes, gtsam::NonlinearFactorGraph* graph,
              int latest_index);

  bool getMapEnuTransform(Mat3d* rot, Vec3d* trans) const;
  bool getEnuOrigin(double* lat_deg, double* lon_deg, double* alt) const;
  std::size_t factorCount() const { return factor_count_; }
  std::size_t rejectedCount() const { return rejected_count_; }
  bool aligned() const { return alignment_complete_; }

  const std::vector<Vec3d>& acceptedPositions() const { return accepted_positions_; }
  const std::vector<Vec3d>& rejectedPositions() const { return rejected_positions_; }
  const std::vector<std::pair<int, Vec3d>>& acceptedFactors() const { return accepted_factors_; }
  const std::vector<std::pair<int, Vec3d>>& rejectedFactors() const { return rejected_factors_; }

 private:
  bool isReliable(const Keyframe& kf) const;
  bool ensureOrigin(const Keyframe& kf);
  bool toEnu(const GnssObservation& gnss, Vec3d* enu) const;
  Vec3d correctLeverArm(const Keyframe& kf, const Vec3d& enu) const;
  bool estimateAlignment();
  bool toMapFrame(const Vec3d& enu, const Keyframe& kf, Vec3d* map_pos) const;
  double positionSigma(const GnssObservation& gnss) const;
  void addGpsFactor(const Keyframe& kf, const Vec3d& map_pos,
                    gtsam::NonlinearFactorGraph* graph);
  void recordRejected(int keyframe_id, const Vec3d& map_pos);

  GnssConfig cfg_;
  bool origin_set_ = false;
  double origin_lat_ = 0.0, origin_lon_ = 0.0, origin_alt_ = 0.0;
  Eigen::Vector3d origin_ecef_ = Eigen::Vector3d::Zero();
  bool alignment_complete_ = false;
  Mat3d rot_map_enu_ = Mat3d::Identity();
  Vec3d trans_map_enu_ = Vec3d::Zero();
  std::vector<std::pair<Vec3d, Vec3d>> alignment_pairs_;
  int last_checked_index_ = -1;
  Eigen::Vector2d last_added_xy_ = Eigen::Vector2d::Zero();
  std::size_t factor_count_ = 0;
  std::size_t rejected_count_ = 0;
  std::vector<Vec3d> accepted_positions_;
  std::vector<Vec3d> rejected_positions_;
  std::vector<std::pair<int, Vec3d>> accepted_factors_;
  std::vector<std::pair<int, Vec3d>> rejected_factors_;
};

}  // namespace slam_mapping_v2
