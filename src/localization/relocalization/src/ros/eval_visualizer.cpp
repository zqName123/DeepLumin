/**
 * @file eval_visualizer.cpp
 * @brief 离线评估 RViz 可视化实现
 */

#include <relocalization/ros/eval_visualizer.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace relocalization {

namespace {

double radToDeg(double rad) {
  return rad * 180.0 / M_PI;
}

}  // namespace

EvalVisualizer::EvalVisualizer(ros::NodeHandle& nh, const EvalVisualizationConfig& config)
    : config_(config) {
  if (!config_.enable) {
    return;
  }

  global_map_pub_   = nh.advertise<sensor_msgs::PointCloud2>("global_map", 1, true);
  truth_cloud_pub_  = nh.advertise<sensor_msgs::PointCloud2>("query_truth_aligned", 1, true);
  reloc_cloud_pub_  = nh.advertise<sensor_msgs::PointCloud2>("query_reloc_aligned", 1, true);
  sc_initial_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("query_sc_initial_aligned", 1, true);
  raw_cloud_pub_    = nh.advertise<sensor_msgs::PointCloud2>("query_raw", 1, true);
  target_submap_pub_= nh.advertise<sensor_msgs::PointCloud2>("gicp_target_submap", 1, true);
  marker_pub_       = nh.advertise<visualization_msgs::MarkerArray>("pose_markers", 1, true);
  status_pub_       = nh.advertise<std_msgs::String>("status", 1, true);
  focus_pose_pub_   = nh.advertise<geometry_msgs::PoseStamped>("focus_pose", 1, true);

  ROS_INFO("Visualization topics under /offline_eval/:");
  ROS_INFO("  global_map          -> gray   map background");
  ROS_INFO("  query_raw           -> orange pre-reloc cloud (LiDAR frame, at map origin)");
  ROS_INFO("  query_sc_initial_aligned -> cyan SC initial guess cloud (map frame)");
  ROS_INFO("  query_reloc_aligned -> red    GICP result (map frame)");
  ROS_INFO("  gicp_target_submap  -> blue   GICP target submap");
  ROS_INFO("  pose_markers        -> cyan=SC initial guess, red=GICP result");
}

// ──────────────────────────────────────────────────────
// 全局地图背景发布
// ──────────────────────────────────────────────────────

void EvalVisualizer::publishGlobalMap(const KeyframeDatabase& database,
                                      const Pose& center, double crop_radius) {
  if (!config_.enable) {
    return;
  }

  CloudPtr cropped;
  if (config_.global_map_source == "keyframes") {
    cropped = database.buildKeyframeMapAround(center, crop_radius, config_.max_keyframes_for_map);
  } else {
    cropped = database.cropGlobalMap(center, crop_radius);
  }
  if (!cropped || cropped->empty()) {
    ROS_WARN_THROTTLE(5.0, "[viz] global_map empty (%s). Check global_map_pcd / keyframe_dir.",
                      config_.global_map_source.c_str());
    return;
  }
  publishCloud(global_map_pub_, cropped);
  global_map_published_ = true;
  ROS_INFO_STREAM("[viz] global_map (" << config_.global_map_source << "): "
                  << cropped->size() << " pts at ("
                  << center.t.x() << ", " << center.t.y() << ", " << center.t.z() << ")");
}

// ──────────────────────────────────────────────────────
// 工具函数
// ──────────────────────────────────────────────────────

sensor_msgs::PointCloud2 EvalVisualizer::toMsg(const CloudConstPtr& cloud) const {
  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(*cloud, msg);
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = config_.frame_id;
  return msg;
}

void EvalVisualizer::publishCloud(const ros::Publisher& pub,
                                  const CloudConstPtr& cloud) const {
  if (!cloud || cloud->empty() || !pub) {
    return;
  }
  pub.publish(toMsg(cloud));
}

void EvalVisualizer::publishCloudInFrame(const ros::Publisher& pub,
                                         const CloudConstPtr& cloud,
                                         const std::string& frame_id) const {
  if (!cloud || cloud->empty() || !pub) {
    return;
  }
  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(*cloud, msg);
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = frame_id;
  pub.publish(msg);
}

Pose EvalVisualizer::poseFromTransform(const Eigen::Matrix4d& transform) {
  Pose pose;
  pose.t = transform.block<3, 1>(0, 3);
  pose.q = Eigen::Quaterniond(transform.block<3, 3>(0, 0)).normalized();
  return pose;
}

// ──────────────────────────────────────────────────────
// 位姿 Marker 发布
// ──────────────────────────────────────────────────────

void EvalVisualizer::publishPoseMarkers(const Pose& truth,
                                        const MatchResult& result) const {
  visualization_msgs::MarkerArray array;

  auto make_arrow = [&](int id, const Eigen::Vector3d& pos, const Eigen::Quaterniond& q,
                        float r, float g, float b, const std::string& ns) {
    visualization_msgs::Marker m;
    m.header.frame_id = config_.frame_id;
    m.header.stamp    = ros::Time::now();
    m.ns = ns; m.id = id;
    m.type   = visualization_msgs::Marker::ARROW;
    m.action = visualization_msgs::Marker::ADD;
    m.pose.position.x = pos.x();
    m.pose.position.y = pos.y();
    m.pose.position.z = pos.z();
    m.pose.orientation.x = q.x();
    m.pose.orientation.y = q.y();
    m.pose.orientation.z = q.z();
    m.pose.orientation.w = q.w();
    m.scale.x = 4.0; m.scale.y = 0.4; m.scale.z = 0.4;
    m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = 1.0f;
    return m;
  };

  // 绿色：真值位姿
  array.markers.push_back(
      make_arrow(0, truth.t, truth.q.normalized(), 0.1f, 0.9f, 0.1f, "truth_pose"));

  if (result.success || result.converged) {
    const Eigen::Vector3d reloc_t = result.transform.block<3, 1>(0, 3);
    const Eigen::Quaterniond reloc_q(result.transform.block<3, 3>(0, 0));
    // 红色：重定位位姿
    array.markers.push_back(
        make_arrow(1, reloc_t, reloc_q.normalized(), 0.9f, 0.2f, 0.1f, "reloc_pose"));

    // 黄色线段：误差连线
    visualization_msgs::Marker line;
    line.header.frame_id = config_.frame_id;
    line.header.stamp    = ros::Time::now();
    line.ns = "pose_error"; line.id = 2;
    line.type   = visualization_msgs::Marker::LINE_STRIP;
    line.action = visualization_msgs::Marker::ADD;
    line.scale.x = 0.15f;
    line.color.r = 1.0f; line.color.g = 1.0f; line.color.a = 1.0f;
    geometry_msgs::Point p0, p1;
    p0.x = truth.t.x(); p0.y = truth.t.y(); p0.z = truth.t.z();
    p1.x = reloc_t.x(); p1.y = reloc_t.y(); p1.z = reloc_t.z();
    line.points.push_back(p0); line.points.push_back(p1);
    array.markers.push_back(line);
  }

  marker_pub_.publish(array);
}

void EvalVisualizer::publishRelocPoseMarker(const MatchResult& result,
                                            const Pose& sc_candidate) const {
  visualization_msgs::MarkerArray array;

  auto make_arrow = [&](int id, const Eigen::Vector3d& pos, const Eigen::Quaterniond& q,
                        float r, float g, float b, const std::string& ns) {
    visualization_msgs::Marker m;
    m.header.frame_id = config_.frame_id;
    m.header.stamp    = ros::Time::now();
    m.ns = ns; m.id = id;
    m.type   = visualization_msgs::Marker::ARROW;
    m.action = visualization_msgs::Marker::ADD;
    m.pose.position.x = pos.x();
    m.pose.position.y = pos.y();
    m.pose.position.z = pos.z();
    m.pose.orientation.x = q.x();
    m.pose.orientation.y = q.y();
    m.pose.orientation.z = q.z();
    m.pose.orientation.w = q.w();
    m.scale.x = 4.0; m.scale.y = 0.4; m.scale.z = 0.4;
    m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = 1.0f;
    return m;
  };

  // 青色箭头：实际送入 GICP 的 SC 初值（candidate pose + SC yaw_diff）
  if (result.candidate_id >= 0) {
    const Pose initial_pose = poseFromTransform(result.initial_guess);
    array.markers.push_back(
        make_arrow(0, initial_pose.t, initial_pose.q.normalized(),
                   0.0f, 0.9f, 0.9f, "initial_guess"));
  } else if (sc_candidate.id >= 0) {
    array.markers.push_back(
        make_arrow(0, sc_candidate.t, sc_candidate.q.normalized(),
                   0.0f, 0.9f, 0.9f, "initial_guess"));
  }

  // 红色箭头：GICP 输出位姿
  if (result.success || result.converged) {
    const Eigen::Vector3d reloc_t = result.transform.block<3, 1>(0, 3);
    const Eigen::Quaterniond reloc_q(result.transform.block<3, 3>(0, 0));
    array.markers.push_back(
        make_arrow(1, reloc_t, reloc_q.normalized(), 0.9f, 0.2f, 0.1f, "reloc_pose"));
  }

  if (!array.markers.empty()) {
    marker_pub_.publish(array);
  }
}

// ──────────────────────────────────────────────────────
// 有真值对比（offline_eval 模式）
// ──────────────────────────────────────────────────────

void EvalVisualizer::publishComparison(int query_id,
                                       const CloudConstPtr& source,
                                       const Pose& truth,
                                       const MatchResult& result,
                                       bool topk_hit,
                                       bool success,
                                       double translation_error,
                                       double yaw_error_deg) {
  if (!config_.enable || !source || source->empty()) {
    return;
  }

  // 绿色：真值对齐
  if (config_.publish_truth_aligned) {
    publishCloud(truth_cloud_pub_, transformCloud(source, truth.matrix()));
  }

  // 红色：GICP 对齐
  if (config_.publish_reloc_aligned) {
    if (result.aligned && !result.aligned->empty()) {
      publishCloud(reloc_cloud_pub_, result.aligned);
    } else if (result.converged || result.success) {
      publishCloud(reloc_cloud_pub_, transformCloud(source, result.transform));
    }
  }

  // 蓝色：GICP target
  if (config_.publish_target_submap && result.target && !result.target->empty()) {
    publishCloud(target_submap_pub_, result.target);
  }

  publishPoseMarkers(truth, result);

  geometry_msgs::PoseStamped focus;
  focus.header.stamp    = ros::Time::now();
  focus.header.frame_id = config_.frame_id;
  focus.pose.position.x = truth.t.x();
  focus.pose.position.y = truth.t.y();
  focus.pose.position.z = truth.t.z();
  focus.pose.orientation.x = truth.q.x();
  focus.pose.orientation.y = truth.q.y();
  focus.pose.orientation.z = truth.q.z();
  focus.pose.orientation.w = truth.q.w();
  focus_pose_pub_.publish(focus);

  std_msgs::String status;
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(3)
     << "query=" << query_id
     << " topk_hit=" << topk_hit << " success=" << success
     << " candidate=" << result.candidate_id
     << " fitness=" << result.fitness
     << " inlier_ratio=" << result.inlier_ratio
     << " inlier_rmse=" << result.inlier_rmse
     << " inliers=" << result.inlier_count << "/" << result.correspondence_count
     << " sc_yaw=" << radToDeg(result.scan_context_yaw_diff)
     << " init_yaw=" << radToDeg(result.initial_yaw)
     << " final_yaw=" << radToDeg(result.final_yaw)
     << " gicp_dt=" << result.delta_translation
     << " gicp_dyaw=" << radToDeg(result.delta_yaw)
     << " gicp_pts=" << result.gicp_source_points << "/" << result.gicp_target_points
     << " terr=" << translation_error << " yaw=" << yaw_error_deg
     << " center=(" << truth.t.x() << "," << truth.t.y() << "," << truth.t.z() << ")";
  status.data = ss.str();
  status_pub_.publish(status);

  ROS_INFO_STREAM("[viz] " << status.data
                  << " | pts reloc/target="
                  << (result.aligned ? result.aligned->size() : 0) << "/"
                  << (result.target  ? result.target->size()  : 0));
  ros::spinOnce();
}

// ──────────────────────────────────────────────────────
// 无真值（bag_eval 模式）
// ──────────────────────────────────────────────────────

void EvalVisualizer::publishRelocResult(int query_id,
                                        const CloudConstPtr& source,
                                        const MatchResult& result,
                                        bool success,
                                        const Pose& sc_candidate) {
  if (!config_.enable || !source || source->empty()) {
    return;
  }

  // 橙色：重定位前原始点云（LiDAR 坐标系，发布到 map frame 但坐标在原点附近）
  // 注意：该点云以 LiDAR 传感器为中心（0,0,0），用于与重定位后结果的结构对比
  publishCloudInFrame(raw_cloud_pub_, source, config_.frame_id);

  // 青色：Scan Context 初值对齐后的点云（map 坐标系，GICP 前）
  if (result.candidate_id >= 0) {
    publishCloud(sc_initial_cloud_pub_, transformCloud(source, result.initial_guess));
  }

  // 红色：GICP 对齐后的点云（map 坐标系）
  if (config_.publish_reloc_aligned) {
    if (result.aligned && !result.aligned->empty()) {
      publishCloud(reloc_cloud_pub_, result.aligned);
    } else if (result.converged || result.success) {
      publishCloud(reloc_cloud_pub_, transformCloud(source, result.transform));
    }
  }

  // 蓝色：GICP target 子图
  if (config_.publish_target_submap && result.target && !result.target->empty()) {
    publishCloud(target_submap_pub_, result.target);
  }

  publishRelocPoseMarker(result, sc_candidate);

  // focus_pose：GICP 结果位置，供 RViz 聚焦
  const Pose reloc_pose = poseFromTransform(result.transform);
  geometry_msgs::PoseStamped focus;
  focus.header.stamp    = ros::Time::now();
  focus.header.frame_id = config_.frame_id;
  focus.pose.position.x    = reloc_pose.t.x();
  focus.pose.position.y    = reloc_pose.t.y();
  focus.pose.position.z    = reloc_pose.t.z();
  focus.pose.orientation.x = reloc_pose.q.x();
  focus.pose.orientation.y = reloc_pose.q.y();
  focus.pose.orientation.z = reloc_pose.q.z();
  focus.pose.orientation.w = reloc_pose.q.w();
  focus_pose_pub_.publish(focus);

  const Pose initial_pose = poseFromTransform(result.initial_guess);

  std_msgs::String status;
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(3)
     << "query=" << query_id << " mode=map_align"
     << " success=" << success << " converged=" << result.converged
     << " candidate=" << result.candidate_id << " fitness=" << result.fitness
     << " inlier_ratio=" << result.inlier_ratio
     << " inlier_rmse=" << result.inlier_rmse
     << " inliers=" << result.inlier_count << "/" << result.correspondence_count
     << " sc_yaw=" << radToDeg(result.scan_context_yaw_diff)
     << " init_yaw=" << radToDeg(result.initial_yaw)
     << " final_yaw=" << radToDeg(result.final_yaw)
     << " gicp_dt=" << result.delta_translation
     << " gicp_dyaw=" << radToDeg(result.delta_yaw)
     << " gicp_pts=" << result.gicp_source_points << "/" << result.gicp_target_points
     << " center=(" << reloc_pose.t.x() << "," << reloc_pose.t.y() << "," << reloc_pose.t.z() << ")";
  if (result.candidate_id >= 0) {
    ss << " sc_init=(" << initial_pose.t.x() << "," << initial_pose.t.y() << ")";
  } else if (sc_candidate.id >= 0) {
    ss << " sc_init=(" << sc_candidate.t.x() << "," << sc_candidate.t.y() << ")";
  }
  status.data = ss.str();
  status_pub_.publish(status);

  ROS_INFO_STREAM("[viz] " << status.data
                  << " | raw_pts=" << source->size()
                  << " sc_init_pts=" << (result.candidate_id >= 0 ? source->size() : 0)
                  << " reloc_pts=" << (result.aligned ? result.aligned->size() : 0)
                  << " target_pts=" << (result.target ? result.target->size() : 0));
  ros::spinOnce();
}

// ──────────────────────────────────────────────────────
// 交互等待
// ──────────────────────────────────────────────────────

void EvalVisualizer::waitForNext() {
  if (!config_.enable) {
    return;
  }
  if (config_.interactive) {
    std::cout << "[viz] Press Enter for next query (Ctrl+C to quit)... " << std::flush;
    std::string line;
    std::getline(std::cin, line);
    return;
  }
  if (config_.wait_sec > 0.0) {
    ros::Duration(config_.wait_sec).sleep();
  }
}

}  // namespace relocalization
