#pragma once

/**
 * @file debug_visualizer.hpp
 * @brief 调试可视化器：发布调试图像和点云
 *
 * 功能：
 *   - 发布处理后的图像（marker 角点、坐标系、ID、重投影误差）
 *   - 发布变换到 map 系的当前帧点云
 *   - 发布裁剪的子图
 *   - 发布 VGICP 匹配后的点云
 *
 * 所有功能可独立开关。
 */

#include <aruco_relocalization/common/types.hpp>
#include <aruco_relocalization/ros/param_loader.hpp>
#include <aruco_relocalization/solver/relocalization_solver.hpp>

#include <cv_bridge/cv_bridge.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <string>

namespace aruco_reloc {

// 前向声明
struct PointCloudVerifyResult;

class DebugVisualizer {
 public:
  /**
   * @brief 初始化可视化器
   */
  bool initialize(const DebugVisualizerConfig& config,
                  const cv::Mat& camera_matrix,
                  const cv::Mat& dist_coeffs,
                  const std::string& dictionary_name);

  /**
   * @brief 发布调试图像（marker 角点、坐标系、ID、重投影误差）
   */
  void publishDebugImage(const cv::Mat& image,
                         const FrameDetection& detection,
                         const SolveResult& solve_result,
                         double reproj_error);

  /**
   * @brief 发布点云可视化
   * @param pc_result 点云验证结果（含 source_in_map、submap、aligned_source）
   */
  void publishPointClouds(const PointCloudVerifyResult& pc_result);

  bool isEnabled() const { return config_.enabled; }

 private:
  void drawAxes(cv::Mat& image,
                const Eigen::Isometry3d& T_cam_marker,
                const cv::Mat& camera_matrix,
                const cv::Mat& dist_coeffs);

  cv::Point2f projectPoint(const Eigen::Vector3d& point_3d,
                           const cv::Mat& camera_matrix);

  DebugVisualizerConfig config_;
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
  cv::Ptr<cv::aruco::Dictionary> dictionary_;

  ros::Publisher debug_image_pub_;
  ros::Publisher source_cloud_pub_;
  ros::Publisher submap_pub_;
  ros::Publisher aligned_cloud_pub_;
};

}  // namespace aruco_reloc
