#pragma once

#include "slam_mapping/common/config.hpp"
#include "slam_mapping/common/types.hpp"
#include "slam_mapping/core/keyframe_manager.hpp"
#include "slam_mapping/core/loop_closure_backend.hpp"
#include "slam_mapping/core/map_saver.hpp"

#include <localization/interface/i_slam_odometry.hpp>

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

  bool processOnce();

  MappingRuntimeStatus status() const;
  std::vector<MappingKeyframe> keyframes() const;
  std::vector<std::pair<int, int>> loopPairs() const;
  localization::OdomResult currentOdom() const;
  localization::TimestampedPointCloud currentScan() const;
  localization::TimestampedPointCloud localMap() const;

 private:
  bool handleFrame(const localization::OdomResult& odom,
                   const localization::TimestampedPointCloud& scan,
                   const localization::TimestampedPointCloud& local_map);
  void updateStatusOnFrame(const localization::OdomResult& odom,
                           const localization::TimestampedPointCloud& scan);

  SlamMappingConfig config_;
  std::shared_ptr<localization::ISlamOdometry> slam_;
  MappingRuntimeStatus status_;
  KeyframeManager keyframe_manager_;
  MapSaver map_saver_;
  LoopClosureBackend loop_backend_;
  localization::OdomResult current_odom_;
  localization::TimestampedPointCloud current_scan_;
  localization::TimestampedPointCloud local_map_;
  std::vector<std::pair<int, int>> loop_pairs_;
};

}  // namespace slam_mapping
