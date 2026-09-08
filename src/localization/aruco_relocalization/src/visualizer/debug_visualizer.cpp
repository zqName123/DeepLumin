/**
 * @file debug_visualizer.cpp
 * @brief 调试可视化器实现
 */

#include <aruco_relocalization/visualizer/debug_visualizer.hpp>
#include <aruco_relocalization/verifier/pointcloud_verifier.hpp>

#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <iomanip>
#include <sstream>

namespace aruco_reloc {

bool DebugVisualizer::initialize(const DebugVisualizerConfig& config,
                                 const cv::Mat& camera_matrix,
                                 const cv::Mat& dist_coeffs,
                                 const std::string& dictionary_name) {
  config_ = config;
  camera_matrix_ = camera_matrix.clone();
  dist_coeffs_ = dist_coeffs.clone();

  // 初始化 ArUco 字典
  auto dict_id = cv::aruco::DICT_5X5_1000;
  if (dictionary_name == "DICT_4X4_50") dict_id = cv::aruco::DICT_4X4_50;
  else if (dictionary_name == "DICT_4X4_100") dict_id = cv::aruco::DICT_4X4_100;
  else if (dictionary_name == "DICT_4X4_250") dict_id = cv::aruco::DICT_4X4_250;
  else if (dictionary_name == "DICT_4X4_1000") dict_id = cv::aruco::DICT_4X4_1000;
  else if (dictionary_name == "DICT_5X5_50") dict_id = cv::aruco::DICT_5X5_50;
  else if (dictionary_name == "DICT_5X5_100") dict_id = cv::aruco::DICT_5X5_100;
  else if (dictionary_name == "DICT_5X5_250") dict_id = cv::aruco::DICT_5X5_250;
  else if (dictionary_name == "DICT_5X5_1000") dict_id = cv::aruco::DICT_5X5_1000;
  dictionary_ = cv::aruco::getPredefinedDictionary(dict_id);

  // 创建发布者
  ros::NodeHandle nh;
  debug_image_pub_ = nh.advertise<sensor_msgs::Image>(config_.debug_image_topic, 1);
  source_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>(config_.source_cloud_topic, 1);
  submap_pub_ = nh.advertise<sensor_msgs::PointCloud2>(config_.submap_topic, 1);
  aligned_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>(config_.aligned_cloud_topic, 1);

  ROS_INFO("[aruco_reloc] debug visualizer initialized: image=%s cloud=%s",
           config_.debug_image_topic.c_str(), config_.source_cloud_topic.c_str());
  return true;
}

void DebugVisualizer::publishDebugImage(const cv::Mat& image,
                                        const FrameDetection& detection,
                                        const SolveResult& solve_result,
                                        double reproj_error) {
  if (!config_.enabled) return;

  cv::Mat vis_image = image.clone();

  // 绘制每个检测到的 marker
  for (const auto& obs : detection.markers) {
    // 绘制角点
    for (int i = 0; i < 4; ++i) {
      cv::circle(vis_image, obs.corners[i], 5, cv::Scalar(0, 255, 0), -1);
      cv::line(vis_image, obs.corners[i], obs.corners[(i + 1) % 4],
               cv::Scalar(0, 255, 0), 2);
    }

    // 绘制 marker ID
    cv::Point2f center(0, 0);
    for (const auto& corner : obs.corners) {
      center += corner;
    }
    center *= 0.25f;

    std::ostringstream id_text;
    id_text << "ID:" << obs.id;
    cv::putText(vis_image, id_text.str(), cv::Point(center.x - 20, center.y),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 0), 2);

    // 绘制坐标系
    drawAxes(vis_image, obs.T_cam_marker, camera_matrix_, dist_coeffs_);
  }

  // 绘制重投影误差和位姿信息
  std::ostringstream info;
  info << std::fixed << std::setprecision(2);
  info << "Markers: " << detection.markers.size();
  if (solve_result.success) {
    info << " | Reproj: " << reproj_error << "px";
    auto t = solve_result.T_map_base.translation();
    info << " | Pos: [" << t.x() << ", " << t.y() << ", " << t.z() << "]";
  }
  cv::putText(vis_image, info.str(), cv::Point(10, 30),
              cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);

  // 发布
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", vis_image).toImageMsg();
  msg->header.stamp = ros::Time::now();
  debug_image_pub_.publish(msg);
}

void DebugVisualizer::publishPointClouds(const PointCloudVerifyResult& pc_result) {
  if (!config_.enabled) return;

  // 发布变换到 map 系的源点云
  if (pc_result.source_in_map && !pc_result.source_in_map->empty()) {
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*pc_result.source_in_map, cloud_msg);
    cloud_msg.header.stamp = ros::Time::now();
    cloud_msg.header.frame_id = "map";
    source_cloud_pub_.publish(cloud_msg);
  }

  // 发布子图
  if (pc_result.submap && !pc_result.submap->empty()) {
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*pc_result.submap, cloud_msg);
    cloud_msg.header.stamp = ros::Time::now();
    cloud_msg.header.frame_id = "map";
    submap_pub_.publish(cloud_msg);
  }

  // 发布 VGICP 匹配后的点云
  if (pc_result.aligned_source && !pc_result.aligned_source->empty()) {
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*pc_result.aligned_source, cloud_msg);
    cloud_msg.header.stamp = ros::Time::now();
    cloud_msg.header.frame_id = "map";
    aligned_cloud_pub_.publish(cloud_msg);
  }
}

void DebugVisualizer::drawAxes(cv::Mat& image,
                               const Eigen::Isometry3d& T_cam_marker,
                               const cv::Mat& camera_matrix,
                               const cv::Mat& dist_coeffs) {
  double len = 0.1;  // 坐标轴长度（米）
  std::vector<Eigen::Vector3d> axes_pts = {
      Eigen::Vector3d(len, 0, 0),   // X 轴
      Eigen::Vector3d(0, len, 0),   // Y 轴
      Eigen::Vector3d(0, 0, len)    // Z 轴
  };

  Eigen::Vector3d origin_3d(0, 0, 0);
  cv::Point2f origin_2d = projectPoint(T_cam_marker * origin_3d, camera_matrix);

  if (origin_2d.x < 0 || origin_2d.y < 0) return;

  cv::Scalar colors[3] = {cv::Scalar(0, 0, 255), cv::Scalar(0, 255, 0), cv::Scalar(255, 0, 0)};

  for (int i = 0; i < 3; ++i) {
    Eigen::Vector3d pt_3d = T_cam_marker * axes_pts[i];
    cv::Point2f pt_2d = projectPoint(pt_3d, camera_matrix);

    if (pt_2d.x < 0 || pt_2d.y < 0) continue;

    cv::arrowedLine(image, origin_2d, pt_2d, colors[i], 2, cv::LINE_AA, 0, 0.1);
  }
}

cv::Point2f DebugVisualizer::projectPoint(const Eigen::Vector3d& point_3d,
                                          const cv::Mat& camera_matrix) {
  double fx = camera_matrix.at<double>(0, 0);
  double fy = camera_matrix.at<double>(1, 1);
  double cx = camera_matrix.at<double>(0, 2);
  double cy = camera_matrix.at<double>(1, 2);

  if (point_3d.z() <= 0.001) return cv::Point2f(-1, -1);

  double x = point_3d.x() / point_3d.z();
  double y = point_3d.y() / point_3d.z();

  cv::Point2f pt;
  pt.x = fx * x + cx;
  pt.y = fy * y + cy;

  if (std::isnan(pt.x) || std::isnan(pt.y)) return cv::Point2f(-1, -1);

  return pt;
}

}  // namespace aruco_reloc
