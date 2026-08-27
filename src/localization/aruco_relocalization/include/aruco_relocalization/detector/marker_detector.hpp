#pragma once

/**
 * @file marker_detector.hpp
 * @brief ArUco marker 检测器封装
 */

#include <aruco_relocalization/common/types.hpp>

#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>

#include <string>

namespace aruco_reloc {

struct DetectorConfig {
  std::string dictionary = "DICT_4X4_50";  // ArUco 字典类型
  double marker_size = 0.2;                // marker 物理边长（米）
  bool corner_subpix = true;               // 是否亚像素精化
  double corner_refinement_win_size = 5;   // 角点精化窗口
};

class MarkerDetector {
 public:
  bool initialize(const DetectorConfig& config, const cv::Mat& camera_matrix,
                  const cv::Mat& dist_coeffs);

  /// 检测一帧图像中的所有 marker
  FrameDetection detect(const cv::Mat& image, double timestamp) const;

  const cv::Mat& cameraMatrix() const { return camera_matrix_; }
  const cv::Mat& distCoeffs() const { return dist_coeffs_; }
  const DetectorConfig& config() const { return config_; }

 private:
  /// 从字典名称获取 OpenCV 字典 ID
  static cv::aruco::PREDEFINED_DICTIONARY_NAME getDictionaryId(const std::string& name);

  /// 将 rvec/tvec 转换为 Isometry3d
  static Eigen::Isometry3d rvecTvecToIsometry(const cv::Vec3d& rvec, const cv::Vec3d& tvec);

  DetectorConfig config_;
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
};

}  // namespace aruco_reloc
