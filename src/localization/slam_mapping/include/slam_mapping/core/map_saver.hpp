#pragma once

#include "slam_mapping/common/config.hpp"
#include "slam_mapping/common/types.hpp"

namespace slam_mapping {

class MapSaver {
 public:
  void configure(const MapSaverConfig& cfg);

  bool saveAll(const SaveRequest& req,
               const std::vector<MappingKeyframe>& keyframes,
               SaveResult* result);
  localization::PointCloudPtr buildMap(const std::vector<MappingKeyframe>& keyframes,
                                       double resolution) const;
  bool savePcd(const std::string& path,
               const localization::PointCloud& cloud) const;
  bool saveKeyframes(const std::string& dir,
                     const std::vector<MappingKeyframe>& keyframes) const;
  bool saveKeyframePoses(const std::string& path,
                         const std::vector<MappingKeyframe>& keyframes) const;
  bool saveTrajectory(const std::string& path,
                      const std::vector<MappingKeyframe>& keyframes) const;

 private:
  void transformCloud(const localization::PointCloud& in,
                      const localization::Pose& pose_map,
                      localization::PointCloud* out) const;

  MapSaverConfig config_;
};

}  // namespace slam_mapping
