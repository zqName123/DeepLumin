#include "slam_mapping/ros/visualization_publisher.hpp"

#include "slam_mapping/ros/ros_converters.hpp"

#include <geometry_msgs/Point.h>
#include <std_msgs/ColorRGBA.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <functional>
#include <unordered_map>

namespace slam_mapping_ros {

namespace {
std::string resolveFrameId(const localization::TimestampedPointCloud& cloud,
                           const std::string& fallback_frame_id) {
  return cloud.frame_id.empty() ? fallback_frame_id : cloud.frame_id;
}
}  // namespace

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
  loop_current_submap_pub_ = nh.advertise<sensor_msgs::PointCloud2>(topics_.loop_current_submap_topic, 10);
  loop_history_submap_pub_ = nh.advertise<sensor_msgs::PointCloud2>(topics_.loop_history_submap_topic, 10);
  map_preview_pub_ = nh.advertise<sensor_msgs::PointCloud2>(topics_.map_preview_topic, 1);
  keyframe_markers_pub_ = nh.advertise<visualization_msgs::MarkerArray>(topics_.keyframe_markers_topic, 10);
  loop_markers_pub_ = nh.advertise<visualization_msgs::MarkerArray>(topics_.loop_markers_topic, 10);
  if (cfg_.publish_gnss_markers) {
    gnss_markers_pub_ = nh.advertise<visualization_msgs::MarkerArray>(topics_.gnss_markers_topic, 10);
  }
  if (cfg_.publish_gnss_edges) {
    gnss_factor_edges_pub_ =
        nh.advertise<visualization_msgs::MarkerArray>(topics_.gnss_factor_edges_topic, 10);
  }
  status_pub_ = nh.advertise<deeplumin_msgs::MappingStatus>(topics_.status_topic, 10);
}

void VisualizationPublisher::publishOdom(const localization::OdomResult& odom) {
  if (odom_pub_) odom_pub_.publish(toRosOdom(odom, frames_.odom, frames_.base_link));
}

void VisualizationPublisher::publishPath(const std::vector<slam_mapping::MappingKeyframe>& keyframes) {
  if (!cfg_.publish_path || !path_pub_) return;
  if (keyframes.size() < path_count_) {
    path.poses.clear();
    path_count_ = 0;
  }
  // nav_msgs::Path path;
  path.header.frame_id = frames_.map;
  path.header.stamp = ros::Time::now();
  for (std::size_t i = path_count_; i < keyframes.size(); ++i) {
    const auto& kf = keyframes[i];
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
  path_count_ = keyframes.size();
  path_pub_.publish(path);

}

void VisualizationPublisher::publishCurrentScan(const localization::TimestampedPointCloud& scan,
                                                const std::string& frame_id) {
  if (!cfg_.publish_current_scan || !current_scan_pub_ || !scan.cloud) return;
  auto msg = toRosPointCloud(scan);
  msg.header.frame_id = resolveFrameId(scan, frame_id);
  current_scan_pub_.publish(msg);
}

void VisualizationPublisher::publishLocalMap(const localization::TimestampedPointCloud& map,
                                             const std::string& frame_id) {
  if (!cfg_.publish_local_map || !local_map_pub_ || !map.cloud) return;
  auto msg = toRosPointCloud(map);
  msg.header.frame_id = resolveFrameId(map, frame_id);
  local_map_pub_.publish(msg);
}

void VisualizationPublisher::publishLoopCurrentSubmap(const localization::TimestampedPointCloud& map,
                                                        const std::string& frame_id) {
  if (!cfg_.publish_local_map || !loop_current_submap_pub_ || !map.cloud || map.cloud->empty()) return;
  auto msg = toRosPointCloud(map);
  msg.header.frame_id = resolveFrameId(map, frame_id);
  loop_current_submap_pub_.publish(msg);
}

void VisualizationPublisher::publishLoopHistorySubmap(const localization::TimestampedPointCloud& map,
                                                        const std::string& frame_id) {
  if (!cfg_.publish_local_map || !loop_history_submap_pub_ || !map.cloud || map.cloud->empty()) return;
  auto msg = toRosPointCloud(map);
  msg.header.frame_id = resolveFrameId(map, frame_id);
  loop_history_submap_pub_.publish(msg);
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

/**
 * @brief 发布 GNSS 因子可视化 marker
 *
 * 绿色球 = 已添加的 GNSS 因子；红色球 = 被空间门控（间距/一致性）拒绝的位置。
 * 仅在 visualize 层生效，不影响优化结果。
 */
void VisualizationPublisher::publishGnssMarkers(const std::vector<Eigen::Vector3d>& accepted,
                                                const std::vector<Eigen::Vector3d>& rejected,
                                                const std::string& frame_id) {
  if (!cfg_.publish_gnss_markers || !gnss_markers_pub_) return;

  visualization_msgs::MarkerArray msg;
  if (accepted.empty() && rejected.empty()) {
    visualization_msgs::Marker clear;
    clear.action = visualization_msgs::Marker::DELETEALL;
    msg.markers.push_back(clear);
    gnss_markers_pub_.publish(msg);
    return;
  }

  auto make_sphere = [&](int id, const Eigen::Vector3d& p, const std::string& ns,
                         float r, float g, float b) {
    visualization_msgs::Marker m;
    m.header.frame_id = frame_id;
    m.header.stamp = ros::Time::now();
    m.ns = ns;
    m.id = id;
    m.type = visualization_msgs::Marker::SPHERE;
    m.action = visualization_msgs::Marker::ADD;
    m.pose.position.x = p.x();
    m.pose.position.y = p.y();
    m.pose.position.z = p.z();
    m.pose.orientation.w = 1.0;
    m.scale.x = 0.4;
    m.scale.y = 0.4;
    m.scale.z = 0.4;
    m.color.r = r;
    m.color.g = g;
    m.color.b = b;
    m.color.a = 1.0;
    return m;
  };

  int id = 0;
  for (const auto& p : accepted) {
    msg.markers.push_back(make_sphere(id++, p, "gnss_accepted", 0.1f, 1.0f, 0.1f));
  }
  for (const auto& p : rejected) {
    msg.markers.push_back(make_sphere(id++, p, "gnss_rejected", 1.0f, 0.2f, 0.2f));
  }
  gnss_markers_pub_.publish(msg);
}

/**
 * @brief 发布 GNSS 球-关键帧残差连线
 *
 * 所有已画出的 GNSS 球（绿=已接受因子，红=被拒）都画一条到其对应关键帧当前
 * pose_map 的线段，线长即当前残差：
 * - 已接受边按长度着色：<1m 绿、<3m 黄、其余红；
 * - 被拒边固定品红色（长度从几何上直接可读，用于区分两类边）。
 */
void VisualizationPublisher::publishGnssFactorEdges(
    const std::vector<std::pair<int, Eigen::Vector3d>>& accepted,
    const std::vector<std::pair<int, Eigen::Vector3d>>& rejected,
    const std::vector<slam_mapping::MappingKeyframe>& keyframes,
    const std::string& frame_id) {
  if (!cfg_.publish_gnss_edges || !gnss_factor_edges_pub_) return;

  visualization_msgs::MarkerArray msg;
  if (accepted.empty() && rejected.empty()) {
    visualization_msgs::Marker clear;
    clear.action = visualization_msgs::Marker::DELETEALL;
    msg.markers.push_back(clear);
    gnss_factor_edges_pub_.publish(msg);
    return;
  }

  std::unordered_map<int, const slam_mapping::MappingKeyframe*> id_to_kf;
  id_to_kf.reserve(keyframes.size());
  for (const auto& kf : keyframes) {
    id_to_kf[kf.id] = &kf;
  }

  const auto color_by_len = [](double len) {
    std_msgs::ColorRGBA c;
    if (len < 1.0) {
      c.r = 0.2f; c.g = 1.0f; c.b = 0.2f;
    } else if (len < 3.0) {
      c.r = 1.0f; c.g = 1.0f; c.b = 0.2f;
    } else {
      c.r = 1.0f; c.g = 0.2f; c.b = 0.2f;
    }
    c.a = 1.0f;
    return c;
  };

  auto build_lines = [&](const std::vector<std::pair<int, Eigen::Vector3d>>& factors,
                         const std::string& ns,
                         const std::function<std_msgs::ColorRGBA(double)>& color_fn,
                         double width) {
    visualization_msgs::Marker lines;
    lines.header.frame_id = frame_id;
    lines.header.stamp = ros::Time::now();
    lines.ns = ns;
    lines.id = 0;
    lines.type = visualization_msgs::Marker::LINE_LIST;
    lines.action = visualization_msgs::Marker::ADD;
    lines.pose.orientation.w = 1.0;
    lines.scale.x = width;
    for (const auto& factor : factors) {
      const auto it = id_to_kf.find(factor.first);
      if (it == id_to_kf.end()) continue;
      const Eigen::Vector3d& kf_pos = it->second->pose_map.position;
      const Eigen::Vector3d& gnss_pos = factor.second;
      const double len = (kf_pos - gnss_pos).norm();
      const auto color = color_fn(len);
      geometry_msgs::Point p1, p2;
      p1.x = kf_pos.x(); p1.y = kf_pos.y(); p1.z = kf_pos.z();
      p2.x = gnss_pos.x(); p2.y = gnss_pos.y(); p2.z = gnss_pos.z();
      lines.points.push_back(p1);
      lines.points.push_back(p2);
      lines.colors.push_back(color);
      lines.colors.push_back(color);
    }
    return lines;
  };

  const std_msgs::ColorRGBA magenta = [] {
    std_msgs::ColorRGBA c;
    c.r = 1.0f; c.g = 0.0f; c.b = 1.0f; c.a = 1.0f;
    return c;
  }();

  auto accepted_lines = build_lines(accepted, "gnss_accepted_edges", color_by_len, 0.05);
  if (!accepted_lines.points.empty()) {
    msg.markers.push_back(accepted_lines);
  }
  auto rejected_lines = build_lines(
      rejected, "gnss_rejected_edges",
      [&magenta](double) { return magenta; }, 0.03);
  if (!rejected_lines.points.empty()) {
    msg.markers.push_back(rejected_lines);
  }

  if (!msg.markers.empty()) {
    gnss_factor_edges_pub_.publish(msg);
  }
}

void VisualizationPublisher::publishStatus(const slam_mapping::MappingRuntimeStatus& status) {
  if (status_pub_) status_pub_.publish(toRosStatus(status, ros::Time::now()));
}

}  // namespace slam_mapping_ros
