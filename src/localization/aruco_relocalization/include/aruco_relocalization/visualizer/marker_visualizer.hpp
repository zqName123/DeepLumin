#pragma once

/**
 * @file marker_visualizer.hpp
 * @brief ArUco Marker 可视化器（绘制角点、坐标系、位姿信息）
 *
 * 设计目标：
 * - 解耦设计：独立于离线建图和在线重定位
 * - 订阅检测结果 topic，绘制可视化图像
 * - 可用于调试和效果验证
 */

#include <aruco_relocalization/common/types.hpp>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <mutex>
#include <string>

namespace aruco_reloc {

/// 可视化配置
struct VisualizerConfig {
  bool enabled = true;                // 是否启用可视化
  std::string input_image_topic = "/camera/li_image";
  std::string output_image_topic = "/aruco_reloc/debug_image";

  // ArUco 配置
  double marker_size = 0.2;           // marker 物理尺寸（米）

  // 绘制选项
  bool draw_corners = true;           // 绘制角点
  bool draw_ids = true;               // 绘制 marker ID
  bool draw_axes = true;              // 绘制坐标系
  bool draw_pose_text = true;         // 绘制位姿文本
  bool draw_cube = false;             // 绘制立方体（3D 效果）

  // 绘制样式
  int corner_radius = 5;              // 角点半径
  int line_thickness = 2;             // 线条粗细
  double axis_length = 0.1;           // 坐标轴长度（米）
  double text_scale = 0.6;            // 文字大小
  int text_thickness = 2;             // 文字粗细

  // 颜色 (BGR)
  cv::Scalar corner_color = cv::Scalar(0, 255, 0);      // 绿色
  cv::Scalar id_color = cv::Scalar(255, 255, 0);        // 青色
  cv::Scalar x_axis_color = cv::Scalar(0, 0, 255);      // 红色
  cv::Scalar y_axis_color = cv::Scalar(0, 255, 0);      // 绿色
  cv::Scalar z_axis_color = cv::Scalar(255, 0, 0);      // 蓝色
  cv::Scalar text_color = cv::Scalar(255, 255, 255);    // 白色
  cv::Scalar bg_color = cv::Scalar(0, 0, 0);            // 黑色背景
};

/// 检测结果（用于可视化输入）
struct VisualDetection {
  double timestamp = 0.0;
  std::vector<MarkerObservation> markers;
  Eigen::Isometry3d T_map_base = Eigen::Isometry3d::Identity();  // 可选：map 下的位姿
  bool has_pose = false;
};

/**
 * @brief ArUco Marker 可视化器
 *
 * 功能：
 * - 订阅原始图像
 * - 独立进行 ArUco 检测（解耦设计）
 * - 绘制 marker 角点、ID、坐标系、位姿信息
 * - 发布可视化图像
 *
 * 使用方式：
 * 1. 离线建图：直接订阅原始图像 topic
 * 2. 在线重定位：直接订阅原始图像 topic
 * 3. 外部输入：调用 processImage() 方法
 */
class MarkerVisualizer {
 public:
  /**
   * @brief 初始化可视化器
   * @param config 可视化配置
   * @param camera_matrix 相机内参矩阵
   * @param dist_coeffs 畸变系数
   * @param dictionary_name ArUco 字典名称（如 "DICT_4X4_50"）
   */
  bool initialize(const VisualizerConfig& config,
                  const cv::Mat& camera_matrix,
                  const cv::Mat& dist_coeffs,
                  const std::string& dictionary_name = "DICT_4X4_50");

  /**
   * @brief 处理图像并返回可视化结果（不发布）
   * @param image 输入图像（BGR格式）
   * @return 可视化后的图像
   */
  cv::Mat processImage(const cv::Mat& image) const;

  /**
   * @brief 绘制可视化图像（不发布，用于调试）
   * @param image 输入图像
   * @param detection 检测结果
   * @return 可视化后的图像
   */
  cv::Mat drawVisualization(const cv::Mat& image,
                            const VisualDetection& detection) const;

 private:
  /**
   * @brief 绘制 marker 角点
   */
  void drawCorners(cv::Mat& image,
                   const std::array<cv::Point2f, 4>& corners) const;

  /**
   * @brief 绘制 marker ID
   */
  void drawMarkerId(cv::Mat& image,
                    const std::array<cv::Point2f, 4>& corners,
                    int id) const;

  /**
   * @brief 绘制坐标系
   */
  void drawAxes(cv::Mat& image,
                const Eigen::Isometry3d& T_cam_marker) const;

  /**
   * @brief 绘制位姿文本信息
   */
  void drawPoseText(cv::Mat& image,
                    const Eigen::Isometry3d& T_cam_marker,
                    int id,
                    const cv::Point& position) const;

  /**
   * @brief 绘制立方体
   */
  void drawCube(cv::Mat& image,
                const Eigen::Isometry3d& T_cam_marker,
                double marker_size) const;

  /**
   * @project 3D 点到图像平面
   */
  cv::Point2f projectPoint(const Eigen::Vector3d& point_3d) const;

  /**
   * @brief 获取字典 ID
   */
  static cv::aruco::PREDEFINED_DICTIONARY_NAME getDictionaryId(const std::string& name);

  VisualizerConfig config_;
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
  cv::Ptr<cv::aruco::Dictionary> detector_;
  cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
  bool initialized_ = false;
};

}  // namespace aruco_reloc
