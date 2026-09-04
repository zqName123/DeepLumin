#pragma once

/**
 * @file map_saver.hpp
 * @brief 地图保存：PCD / CSV（适配 v2 Keyframe）
 */

#include "slam_mapping_v2/common/config.hpp"
#include "slam_mapping_v2/common/types.hpp"

namespace slam_mapping_v2 {

class MapSaver {
 public:
  void configure(const MapSaverConfig& cfg);

  bool saveAll(const SaveRequest& req, const std::vector<Keyframe>& keyframes,
               const std::vector<std::pair<int, int>>& loop_pairs,
               const PointCloudPtr& latest_loop_current_submap,
               const PointCloudPtr& latest_loop_history_submap, SaveResult* result);

  PointCloudPtr buildMap(const std::vector<Keyframe>& keyframes, double resolution) const;
  bool savePcd(const std::string& path, const PointCloud& cloud) const;
  bool saveGnssAlignment(const std::string& path, const Mat3d& rot_map_enu,
                         const Vec3d& trans_map_enu, double origin_lat_deg,
                         double origin_lon_deg, double origin_alt) const;

 private:
  bool saveKeyframes(const std::string& dir, const std::string& images_dir,
                     const std::vector<Keyframe>& keyframes) const;
  bool saveKeyframePoses(const std::string& path, const std::vector<Keyframe>& keyframes) const;
  bool saveTrajectory(const std::string& path, const std::vector<Keyframe>& keyframes) const;
  bool saveLoopPairs(const std::string& path,
                     const std::vector<std::pair<int, int>>& loop_pairs) const;
  bool saveLoopSubmap(const std::string& path, const PointCloudPtr& cloud) const;
  void transformCloud(const PointCloud& in, const Pose& pose_map, const Pose& pose_odom,
                      PointCloud* out) const;
  void transformCloud(const PointCloud& in, const Eigen::Matrix4f& tf,
                      PointCloud* out) const;

  MapSaverConfig config_;
};

}  // namespace slam_mapping_v2
