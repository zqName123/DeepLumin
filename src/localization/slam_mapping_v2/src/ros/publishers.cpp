#include "slam_mapping_v2/ros/publishers.hpp"

#include <pcl_conversions/pcl_conversions.h>

#include <geometry_msgs/Point.h>

namespace slam_mapping_v2_ros {
using namespace slam_mapping_v2;

void Publishers::initialize(ros::NodeHandle& nh, const MappingConfig& cfg) {
  cfg_ = cfg;
  status_pub_ = nh.advertise<deeplumin_msgs::MappingStatus>(cfg_.output.status_topic, 10);
  path_pub_ = nh.advertise<nav_msgs::Path>(cfg_.output.path_topic, 10);
  if (cfg_.map.publish_global_map) {
    map_pub_ = nh.advertise<sensor_msgs::PointCloud2>(cfg_.output.map_topic, 1);
  }
  if (cfg_.output.publish_debug_markers) {
    loop_markers_pub_ =
        nh.advertise<visualization_msgs::MarkerArray>(cfg_.output.loop_markers_topic, 10);
    keyframe_markers_pub_ =
        nh.advertise<visualization_msgs::MarkerArray>(cfg_.output.keyframe_markers_topic, 10);
  }
  if (cfg_.output.publish_odom) {
    odom_pub_ = nh.advertise<nav_msgs::Odometry>(cfg_.output.odom_topic, 10);
  }
  if (cfg_.output.publish_current_scan) {
    scan_pub_ = nh.advertise<sensor_msgs::PointCloud2>(cfg_.output.current_scan_topic, 10);
  }
}

void Publishers::publishStatus(const RuntimeStatus& status, const ros::Time& stamp) {
  if (status_pub_) status_pub_.publish(toStatusMsg(status, stamp));
}

void Publishers::publishPath(const std::vector<Keyframe>& keyframes) {
  if (!path_pub_) return;
  path_.poses.clear();  // 全量重建：优化会更新旧关键帧 pose_map，须重发整条轨迹
  path_.header.frame_id = cfg_.frames.map;
  path_.header.stamp = ros::Time::now();
  for (const auto& kf : keyframes) {
    geometry_msgs::PoseStamped pose;
    pose.header = path_.header;
    pose.pose.position.x = kf.pose_map.position.x();
    pose.pose.position.y = kf.pose_map.position.y();
    pose.pose.position.z = kf.pose_map.position.z();
    pose.pose.orientation.x = kf.pose_map.orientation.x();
    pose.pose.orientation.y = kf.pose_map.orientation.y();
    pose.pose.orientation.z = kf.pose_map.orientation.z();
    pose.pose.orientation.w = kf.pose_map.orientation.w();
    path_.poses.push_back(pose);
  }
  path_pub_.publish(path_);
}

void Publishers::publishMap(const PointCloudPtr& cloud, const std::string& frame_id,
                            double timestamp) {
  if (!map_pub_ || !cloud || cloud->empty()) return;
  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(*cloud, msg);
  msg.header.stamp = ros::Time(timestamp);
  msg.header.frame_id = frame_id;
  map_pub_.publish(msg);
}

void Publishers::publishOdom(const FrameInput& f) {
  if (!odom_pub_ || !f.valid) return;
  nav_msgs::Odometry msg;
  msg.header.stamp = ros::Time(f.timestamp);
  msg.header.frame_id = cfg_.frames.odom;
  msg.child_frame_id = cfg_.frames.base_link;
  msg.pose.pose.position.x = f.position.x();
  msg.pose.pose.position.y = f.position.y();
  msg.pose.pose.position.z = f.position.z();
  msg.pose.pose.orientation.x = f.orientation.x();
  msg.pose.pose.orientation.y = f.orientation.y();
  msg.pose.pose.orientation.z = f.orientation.z();
  msg.pose.pose.orientation.w = f.orientation.w();
  odom_pub_.publish(msg);
}

void Publishers::publishCurrentScan(const FrameInput& f) {
  if (!scan_pub_ || !f.cloud || f.cloud->empty()) return;
  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(*f.cloud, msg);
  msg.header.stamp = ros::Time(f.timestamp);
  msg.header.frame_id = cfg_.frames.map;
  scan_pub_.publish(msg);
}

void Publishers::publishLoopMarkers(const std::vector<std::pair<int, int>>& loop_pairs,
                                    const std::vector<Keyframe>& keyframes) {
  if (!loop_markers_pub_) return;
  visualization_msgs::MarkerArray array;
  visualization_msgs::Marker nodes;
  nodes.header.frame_id = cfg_.frames.map;
  nodes.header.stamp = ros::Time::now();
  nodes.ns = "loop_nodes";
  nodes.id = 0;
  nodes.type = visualization_msgs::Marker::SPHERE_LIST;
  nodes.action = visualization_msgs::Marker::ADD;
  nodes.pose.orientation.w = 1.0;
  nodes.scale.x = nodes.scale.y = nodes.scale.z = 0.3;
  nodes.color.r = 1.0f;
  nodes.color.g = 0.8f;
  nodes.color.b = 0.0f;
  nodes.color.a = 1.0f;
  visualization_msgs::Marker edges = nodes;
  edges.ns = "loop_edges";
  edges.id = 1;
  edges.type = visualization_msgs::Marker::LINE_LIST;
  edges.scale.x = 0.1;
  edges.color.r = 0.0f;
  edges.color.g = 0.8f;
  edges.color.b = 0.0f;
  for (const auto& lp : loop_pairs) {
    if (lp.first < 0 || lp.second < 0 ||
        lp.first >= static_cast<int>(keyframes.size()) ||
        lp.second >= static_cast<int>(keyframes.size())) {
      continue;
    }
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

void Publishers::publishKeyframeMarkers(const std::vector<Keyframe>& keyframes) {
  if (!keyframe_markers_pub_) return;
  visualization_msgs::MarkerArray array;
  visualization_msgs::Marker marker;
  marker.header.frame_id = cfg_.frames.map;
  marker.header.stamp = ros::Time::now();
  marker.ns = "keyframes";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = marker.scale.y = marker.scale.z = 0.3;
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

}  // namespace slam_mapping_v2_ros
