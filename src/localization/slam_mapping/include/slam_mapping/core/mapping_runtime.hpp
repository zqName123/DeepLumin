#pragma once

#include "slam_mapping/common/config.hpp"
#include "slam_mapping/common/types.hpp"
#include "slam_mapping/core/keyframe_manager.hpp"
#include "slam_mapping/core/loop_closure_backend.hpp"
#include "slam_mapping/core/map_saver.hpp"

#include <localization/interface/i_slam_odometry.hpp>

#include <deque>
#include <memory>

namespace slam_mapping {

class MappingRuntime {
 public:
  bool initialize(const SlamMappingConfig& cfg);
  bool start(const RuntimeStartRequest& req, double now_sec);
  bool stop(const RuntimeStopRequest& req);
  bool save(const SaveRequest& req);
  void reset();

  void feedImu(const localization::ImuData& imu);
  void feedLidar(const localization::TimestampedPointCloud& cloud);
  void feedCanVelocity(const localization::CanVelocityData& can);
  void feedGnssAttitude(const localization::GnssAttitudeData& gnss);
  void feedGnss(const GnssObservation& gnss);

  bool processOnce();

  MappingRuntimeStatus status() const;
  std::vector<MappingKeyframe> keyframes() const;
  std::vector<std::pair<int, int>> loopPairs() const;
  std::vector<Eigen::Vector3d> gnssFactorPositions() const;
  std::vector<Eigen::Vector3d> gnssRejectedPositions() const;
  std::vector<std::pair<int, Eigen::Vector3d>> gnssAcceptedFactors() const;
  std::vector<std::pair<int, Eigen::Vector3d>> gnssRejectedFactors() const;
  bool gnssMapEnuTransform(Eigen::Matrix3d* rot, Eigen::Vector3d* trans) const;
  bool gnssEnuOrigin(double* lat_deg, double* lon_deg, double* alt) const;
  localization::OdomResult currentOdom() const;
  localization::TimestampedPointCloud currentScan() const;
  localization::TimestampedPointCloud localMap() const;
  localization::TimestampedPointCloud latestLoopCurrentSubmap() const;
  localization::TimestampedPointCloud latestLoopHistorySubmap() const;

 private:
  bool handleFrame(const localization::OdomResult& odom,
                   const localization::TimestampedPointCloud& scan,
                   const localization::TimestampedPointCloud& local_map);
  void updateStatusOnFrame(const localization::OdomResult& odom,
                           const localization::TimestampedPointCloud& scan);
  void updateImuWarmup(double stamp);
  bool canFeedLidar() const;
  bool warmupReady() const;
  void updateWarmupStatusMessage();
  void associateGnssToLatestKeyframe();

  SlamMappingConfig config_;
  std::shared_ptr<localization::ISlamOdometry> slam_;
  MappingRuntimeStatus status_;
  KeyframeManager keyframe_manager_;
  MapSaver map_saver_;
  LoopClosureBackend loop_backend_;
  localization::OdomResult current_odom_;
  localization::TimestampedPointCloud current_scan_;
  localization::TimestampedPointCloud local_map_;
  localization::TimestampedPointCloud latest_loop_current_submap_;
  localization::TimestampedPointCloud latest_loop_history_submap_;
  std::vector<std::pair<int, int>> loop_pairs_;
  std::deque<GnssObservation> gnss_buffer_;  ///< GNSS 位置观测缓冲（时间对齐用）
  double first_imu_stamp_ = 0.0;
  double last_imu_stamp_ = 0.0;
  std::uint64_t imu_warmup_samples_ = 0;
  std::uint64_t dropped_lidar_before_warmup_ = 0;
  bool imu_warmup_ready_ = true;
};

}  // namespace slam_mapping
