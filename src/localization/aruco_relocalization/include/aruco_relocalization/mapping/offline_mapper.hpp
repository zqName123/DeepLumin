#pragma once

/**
 * @file offline_mapper.hpp
 * @brief 离线建图器（订阅 slam_mapping_v2 位姿，在线聚合 marker 位姿）
 */

#include <aruco_relocalization/common/types.hpp>
#include <aruco_relocalization/detector/marker_detector.hpp>
#include <aruco_relocalization/ros/param_loader.hpp>

#include <nav_msgs/Path.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <Eigen/Dense>

#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace aruco_reloc {

class OfflineMapper {
 public:
  bool initialize(const OfflineMappingConfig& config);

  /// 接收关键帧轨迹
  void onPath(const nav_msgs::Path::ConstPtr& msg);

  /// 接收图像
  void onImage(const sensor_msgs::Image::ConstPtr& msg);

  /// 接收相机内参
  void onCameraInfo(const sensor_msgs::CameraInfo::ConstPtr& msg);

  /// 保存 marker 地图
  void saveMarkerMap();

  /// 获取当前统计信息
  size_t getMarkerCount() const;
  size_t getObservationCount() const;

 private:
  /// 找最近的关键帧位姿
  const KeyframePose* findNearestKeyframe(double timestamp) const;

  /// 聚合 marker 观测
  void aggregateMarker(int id, const Eigen::Isometry3d& T_map_marker, double reproj_error);

  /// 从 MarkerObservation 计算 T_cam_marker
  Eigen::Isometry3d computeTcamMarker(const MarkerObservation& obs) const;

  OfflineMappingConfig config_;
  MarkerDetector detector_;
  Eigen::Isometry3d T_base_cam_ = Eigen::Isometry3d::Identity();

  mutable std::mutex mutex_;
  std::vector<KeyframePose> keyframe_poses_;
  std::unordered_map<int, std::vector<MarkerAggregateObs>> marker_observations_;

  bool camera_info_received_ = false;
  bool initialized_ = false;
};

}  // namespace aruco_reloc
