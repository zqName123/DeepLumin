#include "slam_mapping/ros/visualization_publisher.hpp"

#include "slam_mapping/ros/ros_converters.hpp"

#include <geometry_msgs/Point.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

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

void VisualizationPublisher::publishMapPreview(const std::vector<slam_mapping::MappingKeyframe>& keyframes) {
  if (!cfg_.publish_accumulated_map_preview || !map_preview_pub_) return;
  localization::PointCloudPtr map(new localization::PointCloud());
  for (const auto& kf : keyframes) {
    if (!kf.cloud_body || kf.cloud_body->empty()) continue;
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    T.block<3, 3>(0, 0) = kf.pose_map.orientation.normalized().toRotationMatrix().cast<float>();
    T.block<3, 1>(0, 3) = kf.pose_map.position.cast<float>();
    localization::PointCloud temp;
    pcl::transformPointCloud(*kf.cloud_body, temp, T);
    *map += temp;
  }
  if (map->empty()) return;
  if (cfg_.accumulated_map_voxel_leaf > 0.0) {
    pcl::VoxelGrid<localization::PointType> voxel;
    voxel.setLeafSize(cfg_.accumulated_map_voxel_leaf, cfg_.accumulated_map_voxel_leaf, cfg_.accumulated_map_voxel_leaf);
    voxel.setInputCloud(map);
    localization::PointCloudPtr filtered(new localization::PointCloud());
    voxel.filter(*filtered);
    map = filtered;
  }
  localization::TimestampedPointCloud pc;
  pc.timestamp = ros::Time::now().toSec();
  pc.frame_id = frames_.map;
  pc.cloud = map;
  map_preview_pub_.publish(toRosPointCloud(pc));
}

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
  visualization_msgs::Marker nodes;
  nodes.header.frame_id = frames_.map;
  nodes.header.stamp = ros::Time::now();
  nodes.ns = "loop_nodes";
  nodes.id = 0;
  nodes.type = visualization_msgs::Marker::SPHERE_LIST;
  nodes.action = visualization_msgs::Marker::ADD;
  nodes.pose.orientation.w = 1.0;
  nodes.scale.x = 0.3;
  nodes.scale.y = 0.3;
  nodes.scale.z = 0.3;
  nodes.color.r = 1.0f;
  nodes.color.g = 0.8f;
  nodes.color.b = 0.0f;
  nodes.color.a = 1.0f;

  visualization_msgs::Marker edges;
  edges.header = nodes.header;
  edges.ns = "loop_edges";
  edges.id = 1;
  edges.type = visualization_msgs::Marker::LINE_LIST;
  edges.action = visualization_msgs::Marker::ADD;
  edges.pose.orientation.w = 1.0;
  edges.scale.x = 0.1;
  edges.color.r = 0.0f;
  edges.color.g = 0.8f;
  edges.color.b = 0.0f;
  edges.color.a = 1.0f;

  for (const auto& lp : loop_pairs) {
    if (lp.first < 0 || lp.second < 0 || lp.first >= static_cast<int>(keyframes.size()) || lp.second >= static_cast<int>(keyframes.size())) continue;
    geometry_msgs::Point a, b;
    a.x = keyframes[lp.first].pose_map.position.x();
    a.y = keyframes[lp.first].pose_map.position.y();
    a.z = keyframes[lp.first].pose_map.position.z();
    b.x = keyframes[lp.second].pose_map.position.x();
    b.y = keyframes[lp.second].pose_map.position.y();
    b.z = keyframes[lp.second].pose_map.position.z();
    nodes.points.push_back(a);
    nodes.points.push_back(b);
    edges.points.push_back(a);
    edges.points.push_back(b);
  }
  array.markers.push_back(nodes);
  array.markers.push_back(edges);
  loop_markers_pub_.publish(array);
}

void VisualizationPublisher::publishStatus(const slam_mapping::MappingRuntimeStatus& status) {
  if (status_pub_) status_pub_.publish(toRosStatus(status, ros::Time::now()));
}

}  // namespace slam_mapping_ros
