#include <aruco_relocalization/detector/marker_detector.hpp>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <ros/ros.h>

namespace aruco_reloc {

bool MarkerDetector::initialize(const DetectorConfig& config, const cv::Mat& camera_matrix,
                                const cv::Mat& dist_coeffs) {
  config_ = config;
  camera_matrix_ = camera_matrix.clone();
  dist_coeffs_ = dist_coeffs.clone();

  // 获取字典
  auto dict_id = getDictionaryId(config_.dictionary);
  dictionary_ = cv::aruco::getPredefinedDictionary(dict_id);

  // 检测参数
  detector_params_ = cv::aruco::DetectorParameters::create();
  if (config_.corner_subpix) {
    detector_params_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
    detector_params_->cornerRefinementWinSize = static_cast<int>(config_.corner_refinement_win_size);
  }

  ROS_INFO("[aruco_reloc] detector initialized: dict=%s marker_size=%.3f",
           config_.dictionary.c_str(), config_.marker_size);
  return true;
}

FrameDetection MarkerDetector::detect(const cv::Mat& image, double timestamp) const {
  FrameDetection result;
  result.timestamp = timestamp;

  if (image.empty() || camera_matrix_.empty()) {
    return result;
  }

  // 检测 marker
  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> corners;
  cv::aruco::detectMarkers(image, dictionary_, corners, ids, detector_params_);

  if (ids.empty()) {
    return result;
  }

  // 估计每个 marker 的位姿
  std::vector<cv::Vec3d> rvecs, tvecs;
  cv::aruco::estimatePoseSingleMarkers(corners, static_cast<float>(config_.marker_size),
                                       camera_matrix_, dist_coeffs_, rvecs, tvecs);

  // 调试：检查位姿估计结果
  for (size_t i = 0; i < rvecs.size(); ++i) {
    if (std::isnan(rvecs[i][0]) || std::isnan(tvecs[i][0])) {
      ROS_WARN("[aruco_detector] marker %d: pose estimation failed (nan), corners=[%.1f,%.1f %.1f,%.1f %.1f,%.1f %.1f,%.1f]",
               ids[i],
               corners[i][0].x, corners[i][0].y,
               corners[i][1].x, corners[i][1].y,
               corners[i][2].x, corners[i][2].y,
               corners[i][3].x, corners[i][3].y);
    }
  }

  // 组装结果
  for (size_t i = 0; i < ids.size(); ++i) {
    MarkerObservation obs;
    obs.id = ids[i];
    for (int j = 0; j < 4; ++j) {
      obs.corners[j] = corners[i][j];
    }
    obs.T_cam_marker = rvecTvecToIsometry(rvecs[i], tvecs[i]);

    // 计算重投影误差
    std::vector<cv::Point3d> obj_points = {
      {-config_.marker_size / 2, -config_.marker_size / 2, 0},
      { config_.marker_size / 2, -config_.marker_size / 2, 0},
      { config_.marker_size / 2,  config_.marker_size / 2, 0},
      {-config_.marker_size / 2,  config_.marker_size / 2, 0}
    };
    std::vector<cv::Point2d> projected;
    cv::projectPoints(obj_points, rvecs[i], tvecs[i], camera_matrix_, dist_coeffs_, projected);

    double total_error = 0.0;
    for (int j = 0; j < 4; ++j) {
      double dx = projected[j].x - corners[i][j].x;
      double dy = projected[j].y - corners[i][j].y;
      total_error += std::sqrt(dx * dx + dy * dy);
    }
    obs.reprojection_error = total_error / 4.0;

    result.markers.push_back(obs);
  }

  return result;
}

cv::aruco::PREDEFINED_DICTIONARY_NAME MarkerDetector::getDictionaryId(const std::string& name) {
  static const std::unordered_map<std::string, cv::aruco::PREDEFINED_DICTIONARY_NAME> dict_map = {
    {"DICT_4X4_50", cv::aruco::DICT_4X4_50},
    {"DICT_4X4_100", cv::aruco::DICT_4X4_100},
    {"DICT_4X4_250", cv::aruco::DICT_4X4_250},
    {"DICT_4X4_1000", cv::aruco::DICT_4X4_1000},
    {"DICT_5X5_50", cv::aruco::DICT_5X5_50},
    {"DICT_5X5_100", cv::aruco::DICT_5X5_100},
    {"DICT_5X5_250", cv::aruco::DICT_5X5_250},
    {"DICT_5X5_1000", cv::aruco::DICT_5X5_1000},
    {"DICT_6X6_50", cv::aruco::DICT_6X6_50},
    {"DICT_6X6_100", cv::aruco::DICT_6X6_100},
    {"DICT_6X6_250", cv::aruco::DICT_6X6_250},
    {"DICT_6X6_1000", cv::aruco::DICT_6X6_1000},
    {"DICT_7X7_50", cv::aruco::DICT_7X7_50},
    {"DICT_7X7_100", cv::aruco::DICT_7X7_100},
    {"DICT_7X7_250", cv::aruco::DICT_7X7_250},
    {"DICT_7X7_1000", cv::aruco::DICT_7X7_1000},
    {"DICT_ARUCO_ORIGINAL", cv::aruco::DICT_ARUCO_ORIGINAL},
  };

  auto it = dict_map.find(name);
  if (it != dict_map.end()) {
    return it->second;
  }
  ROS_WARN("[aruco_reloc] unknown dictionary '%s', using DICT_4X4_50", name.c_str());
  return cv::aruco::DICT_4X4_50;
}

Eigen::Isometry3d MarkerDetector::rvecTvecToIsometry(const cv::Vec3d& rvec,
                                                     const cv::Vec3d& tvec) {
  cv::Mat rmat;
  cv::Rodrigues(rvec, rmat);

  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T(0, 3) = tvec[0];
  T(1, 3) = tvec[1];
  T(2, 3) = tvec[2];

  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      T(i, j) = rmat.at<double>(i, j);
    }
  }

  return T;
}

}  // namespace aruco_reloc
