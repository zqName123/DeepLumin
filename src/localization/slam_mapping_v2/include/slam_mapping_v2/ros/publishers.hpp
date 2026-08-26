#pragma once

/**
 * @file publishers.hpp
 * @brief 轻量发布器（status/path/map/调试 marker；odom/scan 转发默认关）
 */

#include "slam_mapping_v2/common/config.hpp"
#include "slam_mapping_v2/common/types.hpp"
#include "slam_mapping_v2/ros/converters.hpp"

#include <deeplumin_msgs/MappingStatus.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>

namespace slam_mapping_v2_ros {

class Publishers {
 public:
  void initialize(ros::NodeHandle& nh, const slam_mapping_v2::MappingConfig& cfg);

  void publishStatus(const slam_mapping_v2::RuntimeStatus& status, const ros::Time& stamp);
  void publishPath(const std::vector<slam_mapping_v2::Keyframe>& keyframes);
  void publishMap(const slam_mapping_v2::PointCloudPtr& cloud, const std::string& frame_id, double timestamp);
  void publishOdom(const slam_mapping_v2::FrameInput& f);
  void publishCurrentScan(const slam_mapping_v2::FrameInput& f);
  void publishLoopMarkers(const std::vector<std::pair<int, int>>& loop_pairs,
                          const std::vector<slam_mapping_v2::Keyframe>& keyframes);
  void publishKeyframeMarkers(const std::vector<slam_mapping_v2::Keyframe>& keyframes);

 private:
  slam_mapping_v2::MappingConfig cfg_;
  ros::Publisher status_pub_;
  ros::Publisher path_pub_;
  ros::Publisher map_pub_;
  ros::Publisher loop_markers_pub_;
  ros::Publisher keyframe_markers_pub_;
  ros::Publisher odom_pub_;
  ros::Publisher scan_pub_;
  nav_msgs::Path path_;
  std::size_t path_count_ = 0;
};

}  // namespace slam_mapping_v2_ros
