#pragma once

/**
 * @file types.hpp
 * @brief ArUco 重定位核心数据结构
 */

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>

#include <array>
#include <string>
#include <unordered_map>
#include <vector>

namespace aruco_reloc {

/// 单个 marker 检测结果
struct MarkerObservation {
  int id = -1;                                    // marker ID
  std::array<cv::Point2f, 4> corners;             // 四个角点像素坐标
  Eigen::Isometry3d T_cam_marker = Eigen::Isometry3d::Identity();  // 相机到 marker 的变换
  double reprojection_error = 0.0;                // 角点重投影误差
};

/// 一帧检测结果
struct FrameDetection {
  double timestamp = 0.0;
  std::vector<MarkerObservation> markers;
};

/// 单个 marker 地图项
struct MarkerItem {
  int id = -1;
  Eigen::Isometry3d T_map_marker = Eigen::Isometry3d::Identity();  // map 坐标系下 marker 位姿
  double size = 0.2;                              // marker 物理尺寸（米）
  int observation_count = 0;                      // 观测帧数
  double mean_reproj_error = 0.0;                 // 平均重投影误差
};

/// marker 地图
using MarkerMap = std::unordered_map<int, MarkerItem>;

/// 关键帧位姿缓存（用于离线建图）
struct KeyframePose {
  double timestamp = 0.0;
  Eigen::Isometry3d T_map_base = Eigen::Isometry3d::Identity();
};

/// marker 聚合观测（用于离线建图）
struct MarkerAggregateObs {
  Eigen::Isometry3d T_map_marker = Eigen::Isometry3d::Identity();
  double reproj_error = 0.0;
};

}  // namespace aruco_reloc
