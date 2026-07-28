#include "slam_mapping/ros/visualization_publisher.hpp"

#include "slam_mapping/ros/ros_converters.hpp"

#include <geometry_msgs/Point.h>

namespace slam_mapping_ros {

void VisualizationPublisher::initialize(ros::NodeHandle& nh,
                                        const slam_mapping::VisualizationConfig& cfg,
                                        const localization::FrameConfig& frames,
                                        const slam_mapping::OutputTopicConfig& topics) {
  cfg_ = cfg;
  frames_ = frames;
  topics_ = topics;
  odom_pub_ = nh.advertise<nav_msgs::Odometry>(topics_.odom_topic, 10);
  path_pub_ = nh.advertise<nav_msgs::Path>(topics_.path_topic, 10);
  current_scan_pub_ = nh.advertise<sensor_msgs::PointCloud2>(topics_.current_scan_topic, 10);
  local_map_pub_ = nh.advertise<sensor_msgs::PointCloud2>(topics_.local_map_topic, 10);
  map_preview_pub_ = nh.advertise<sensor_msgs::PointCloud2>(topics_.map_preview_topic, 1);
  keyframe_markers_pub_ = nh.advertise<visualization_msgs::MarkerArray>(topics_.keyframe_markers_topic, 10);
  loop_markers_pub_ = nh.advertise<visualization_msgs::MarkerArray>(topics_.loop_markers_topic, 10);
  status_pub_ = nh.advertise<deeplumin_msgs::MappingStatus>(topics_.status_topic, 10);
}

void VisualizationPublisher::publishOdom(const localization::OdomResult& odom) {
  if (odom_pub_) odom_pub_.publish(toRosOdom(odom, frames_.odom, frames_.base_link));
}

void VisualizationPublisher::publishPath(const std::vector<slam_mapping::MappingKeyframe>& keyframes) {
  if (!cfg_.publish_path || !path_pub_) return;
  nav_msgs::Path path;
  path.header.frame_id = frames_.map;
  path.header.stamp = ros::Time::now();
  for (const auto& kf : keyframes) {
    geometry_msgs::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = kf.pose_map.position.x();
    pose.pose.position.y = kf.pose_map.position.y();
    pose.pose.position.z = kf.pose_map.position.z();
    pose.pose.orientation.x = kf.pose_map.orientation.x();
    pose.pose.orientation.y = kf.pose_map.orientation.y();
    pose.pose.orientation.z = kf.pose_map.orientation.z();
    pose.pose.orientation.w = kf.pose_map.orientation.w();
    path.poses.push_back(pose);
  }
  path_pub_.publish(path);
}

void VisualizationPublisher::publishCurrentScan(const localization::TimestampedPointCloud& scan,
                                                const std::string& frame_id) {
  if (!cfg_.publish_current_scan || !current_scan_pub_ || !scan.cloud) return;
  auto msg = toRosPointCloud(scan);
  msg.header.frame_id = frame_id;
  current_scan_pub_.publish(msg);
}

void VisualizationPublisher::publishLocalMap(const localization::TimestampedPointCloud& map,
                                             const std::string& frame_id) {
  if (!cfg_.publish_local_map || !local_map_pub_ || !map.cloud) return;
  auto msg = toRosPointCloud(map);
  msg.header.frame_id = frame_id;
  local_map_pub_.publish(msg);
}

void VisualizationPublisher::publishMapPreview(const std::vector<slam_mapping::MappingKeyframe>&) {}

void VisualizationPublisher::publishKeyframeMarkers(const std::vector<slam_mapping::MappingKeyframe>& keyframes) {
  if (!cfg_.publish_keyframes || !keyframe_markers_pub_) return;
  visualization_msgs::MarkerArray array;
  visualization_msgs::Marker marker;
  marker.header.frame_id = frames_.map;
  marker.header.stamp = ros::Time::now();
  marker.ns = "keyframes";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.3;
  marker.scale.y = 0.3;
  marker.scale.z = 0.3;
  marker.color.r = 0.2f;
  marker.color.g = 0.8f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0f;
  for (const auto& kf : keyframes) {
    geometry_msgs::Point p;
    p.x = kf.pose_map.position.x();
    p.y = kf.pose_map.position.y();
    p.z = kf.pose_map.position.z();
    marker.points.push_back(p);
  }
  array.markers.push_back(marker);
  keyframe_markers_pub_.publish(array);
}

void VisualizationPublisher::publishLoopMarkers(const std::vector<std::pair<int, int>>& loop_pairs,
                                                const std::vector<slam_mapping::MappingKeyframe>& keyframes) {
  if (!cfg_.publish_loop_markers || !loop_markers_pub_) return;
  visualization_msgs::MarkerArray array;
  visualization_msgs::Marker marker;
  marker.header.frame_id = frames_.map;
  marker.header.stamp = ros::Time::now();
  marker.ns = "loop_edges";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.1;
  marker.color.g = 1.0f;
  marker.color.a = 1.0f;
  for (const auto& lp : loop_pairs) {
    if (lp.first < 0 || lp.second < 0 || lp.first >= static_cast<int>(keyframes.size()) || lp.second >= static_cast<int>(keyframes.size())) continue;
    geometry_msgs::Point a, b;
    a.x = keyframes[lp.first].pose_map.position.x();
    a.y = keyframes[lp.first].pose_map.position.y();
    a.z = keyframes[lp.first].pose_map.position.z();
    b.x = keyframes[lp.second].pose_map.position.x();
    b.y = keyframes[lp.second].pose_map.position.y();
    b.z = keyframes[lp.second].pose_map.position.z();
    marker.points.push_back(a);
    marker.points.push_back(b);
  }
  array.markers.push_back(marker);
  loop_markers_pub_.publish(array);
}

void VisualizationPublisher::publishStatus(const slam_mapping::MappingRuntimeStatus& status) {
  if (status_pub_) status_pub_.publish(toRosStatus(status, ros::Time::now()));
}

}  // namespace slam_mapping_ros
