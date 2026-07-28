#pragma once

#include "slam_mapping/common/config.hpp"
#include "slam_mapping/common/types.hpp"

#include <deeplumin_msgs/MappingStatus.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>

namespace slam_mapping_ros {

class VisualizationPublisher {
 public:
  void initialize(ros::NodeHandle& nh,
                  const slam_mapping::VisualizationConfig& cfg,
                  const localization::FrameConfig& frames,
                  const slam_mapping::OutputTopicConfig& topics);
  void publishOdom(const localization::OdomResult& odom);
  void publishPath(const std::vector<slam_mapping::MappingKeyframe>& keyframes);
  void publishCurrentScan(const localization::TimestampedPointCloud& scan,
                          const std::string& frame_id);
  void publishLocalMap(const localization::TimestampedPointCloud& map,
                       const std::string& frame_id);
  void publishMapPreview(const std::vector<slam_mapping::MappingKeyframe>& keyframes);
  void publishKeyframeMarkers(const std::vector<slam_mapping::MappingKeyframe>& keyframes);
  void publishLoopMarkers(const std::vector<std::pair<int, int>>& loop_pairs,
                          const std::vector<slam_mapping::MappingKeyframe>& keyframes);
  void publishStatus(const slam_mapping::MappingRuntimeStatus& status);

 private:
  slam_mapping::VisualizationConfig cfg_;
  localization::FrameConfig frames_;
  slam_mapping::OutputTopicConfig topics_;
  ros::Publisher odom_pub_;
  ros::Publisher path_pub_;
  ros::Publisher current_scan_pub_;
  ros::Publisher local_map_pub_;
  ros::Publisher map_preview_pub_;
  ros::Publisher keyframe_markers_pub_;
  ros::Publisher loop_markers_pub_;
  ros::Publisher status_pub_;
};

}  // namespace slam_mapping_ros
