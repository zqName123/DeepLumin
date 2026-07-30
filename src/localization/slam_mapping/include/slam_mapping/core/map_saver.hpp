#pragma once

#include "slam_mapping/common/config.hpp"
#include "slam_mapping/common/types.hpp"

namespace slam_mapping {

class MapSaver {
 public:
  void configure(const MapSaverConfig& cfg);

  bool saveAll(const SaveRequest& req,
               const std::vector<MappingKeyframe>& keyframes,
               const std::vector<std::pair<int, int>>& loop_pairs,
               const localization::PointCloudPtr& latest_loop_current_submap,
               const localization::PointCloudPtr& latest_loop_history_submap,
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
  bool saveLoopPairs(const std::string& path,
                     const std::vector<std::pair<int, int>>& loop_pairs) const;
  bool saveLoopSubmap(const std::string& path,
                      const localization::PointCloudPtr& cloud) const;

 private:
  void transformCloud(const localization::PointCloud& in,
                      const localization::Pose& pose_map,
                      localization::PointCloud* out) const;

  MapSaverConfig config_;
};

}  // namespace slam_mapping
