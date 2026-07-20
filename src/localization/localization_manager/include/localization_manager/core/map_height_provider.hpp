#pragma once

#include "localization_manager/common/config.hpp"

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <string>
#include <vector>

namespace localization_manager {

struct HeightQueryResult {
  bool success = false;
  double z = 0.0;
  double z_std = 0.0;
  int point_count = 0;
  double min_z = 0.0;
  double max_z = 0.0;
  std::string source;
  std::string reason;
};

class MapHeightProvider {
 public:
  bool initialize(const InitialPoseCompletionConfig& config);
  bool ready() const { return ready_; }
  const std::string& mapPath() const { return config_.pcd_path; }

  HeightQueryResult query(const geometry_msgs::PoseWithCovarianceStamped& pose) const;
  geometry_msgs::PoseWithCovarianceStamped completePose(
      const geometry_msgs::PoseWithCovarianceStamped& pose,
      const std::string& frame_id,
      HeightQueryResult* result) const;

 private:
  using PointT = pcl::PointXYZ;
  using CloudT = pcl::PointCloud<PointT>;
  using CloudPtr = CloudT::Ptr;

  static double yawFromPose(const geometry_msgs::PoseWithCovarianceStamped& pose);
  double estimateZ(std::vector<double>& values, double min_z, double max_z) const;

  InitialPoseCompletionConfig config_;
  CloudPtr map_;
  bool ready_ = false;
};

}  // namespace localization_manager
