#pragma once

/**
 * @file offline_mapping_v2.hpp
 * @brief 离线建图 V2：读取关键帧图像和位姿，遍历检测 marker，输出 marker map
 *
 * 设计思路：
 * 1. 读取 slam_mapping_v2 保存的关键帧位姿（CSV）
 * 2. 读取对应的关键帧图像（images/ 文件夹）
 * 3. 遍历每个关键帧，检测 ArUco marker
 * 4. 使用优化后的位姿计算 marker 在 map 坐标系下的位置
 * 5. 聚合观测，输出 marker_map.yaml
 */

#include <aruco_relocalization/common/types.hpp>
#include <aruco_relocalization/detector/marker_detector.hpp>

#include <Eigen/Dense>
#include <opencv2/core.hpp>

#include <string>
#include <unordered_map>
#include <vector>

namespace aruco_reloc {

/// 离线建图 V2 配置
struct OfflineMappingV2Config {
  // 输入路径
  std::string keyframe_poses_csv;  // 关键帧位姿 CSV 文件
  std::string keyframe_images_dir; // 关键帧图像目录

  // 输出路径
  std::string output_marker_map;   // 输出 marker map 文件

  // 外参（分离 base->lidar 和 lidar->camera）
  Eigen::Isometry3d T_base_lidar = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d T_lidar_camera = Eigen::Isometry3d::Identity();

  // 图像处理
  int image_rotation = 0;  // 0: 不旋转, 1: 顺时针90°, 2: 180°, 3: 逆时针90°

  // 检测器配置
  DetectorConfig detector;

  // 校验参数
  double max_reproj_error = 5.0;  // pixels
  int min_observations = 3;       // 每个 marker 至少被观测到的次数
};

/// 关键帧数据
struct KeyframeData {
  int id;
  double timestamp;
  Eigen::Isometry3d T_map_base;  // 优化后的位姿
};

/// Marker 观测
struct MarkerObservationV2 {
  int keyframe_id;
  Eigen::Isometry3d T_map_marker;  // marker 在 map 下的位姿
  double reprojection_error;
};

/**
 * @brief 离线建图 V2 类
 *
 * 读取关键帧图像和位姿，遍历检测 marker，输出 marker map
 */
class OfflineMappingV2 {
 public:
  /**
   * @brief 初始化
   * @param config 配置
   * @param camera_matrix 相机内参矩阵
   * @param dist_coeffs 畸变系数
   */
  bool initialize(const OfflineMappingV2Config& config,
                  const cv::Mat& camera_matrix,
                  const cv::Mat& dist_coeffs);

  /**
   * @brief 处理所有关键帧
   * @return 是否成功
   */
  bool process();

  /**
   * @brief 保存 marker map
   * @return 是否成功
   */
  bool saveMarkerMap();

  /**
   * @brief 获取统计信息
   */
  int getKeyframeCount() const { return keyframes_.size(); }
  int getMarkerCount() const { return marker_aggregates_.size(); }

 private:
  /**
   * @brief 读取关键帧位姿 CSV
   */
  bool loadKeyframePoses(const std::string& csv_path);

  /**
   * @brief 读取关键帧图像
   */
  cv::Mat loadKeyframeImage(int keyframe_id);

  /**
   * @brief 处理单个关键帧
   */
  bool processKeyframe(const KeyframeData& kf);

  /**
   * @brief 聚合 marker 观测
   */
  bool aggregateMarkerObservations();

  OfflineMappingV2Config config_;
  MarkerDetector detector_;
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;

  // 关键帧数据
  std::vector<KeyframeData> keyframes_;

  // Marker 聚合结果
  struct MarkerAggregate {
    int id;
    std::vector<MarkerObservationV2> observations;
    Eigen::Isometry3d T_map_marker;
    double mean_reproj_error;
    int observation_count;
  };
  std::unordered_map<int, MarkerAggregate> marker_aggregates_;
};

}  // namespace aruco_reloc
