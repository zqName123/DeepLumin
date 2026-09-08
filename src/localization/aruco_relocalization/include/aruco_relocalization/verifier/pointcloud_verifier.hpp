#pragma once

/**
 * @file pointcloud_verifier.hpp
 * @brief 点云验证器：用 VGICP 匹配验证并精化 ArUco 重定位位姿
 *
 * 流程：
 *   1. 订阅当前帧点云（lidar 系）
 *   2. 旋转到 base_link 系（-90°）
 *   3. 用 ArUco 位姿作为初始猜测，从全局地图裁剪局部子图
 *   4. VGICP 两级配准（粗配准 + 精匹配）
 *   5. 最近邻搜索，统计 inlier_ratio / inlier_rmse
 *   6. 输出精化后的位姿和质量指标
 */

#include <aruco_relocalization/ros/param_loader.hpp>

#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>

#include <Eigen/Dense>
#include <deque>
#include <mutex>
#include <string>

namespace aruco_reloc {

using PointT = pcl::PointXYZI;
using CloudT = pcl::PointCloud<PointT>;
using CloudPtr = CloudT::Ptr;
using CloudConstPtr = CloudT::ConstPtr;

/// 点云验证结果
struct PointCloudVerifyResult {
  bool valid = false;
  bool vgicp_converged = false;
  Eigen::Isometry3d T_map_base_refined = Eigen::Isometry3d::Identity();  // VGICP 精化后的位姿
  double inlier_ratio = 0.0;
  double inlier_rmse = 1e9;
  int inlier_count = 0;
  int correspondence_count = 0;
  double delta_translation = 0.0;  // VGICP 修正的平移量
  double delta_rotation_deg = 0.0; // VGICP 修正的旋转量
  double elapsed_ms = 0.0;
  std::string reject_reason;

  // 可视化数据（供 DebugVisualizer 使用）
  CloudPtr source_in_map;      // 变换到 map 系的当前帧点云
  CloudPtr submap;             // 裁剪的子图
  CloudPtr aligned_source;     // VGICP 匹配后的点云（仅 use_vgicp=true）
};

class PointCloudVerifier {
 public:
  /**
   * @brief 初始化验证器
   * @param config 点云验证配置
   * @return true 初始化成功
   */
  bool initialize(const PointCloudVerifyConfig& config);

  /**
   * @brief 点云回调（订阅调用）
   */
  void onCloud(const sensor_msgs::PointCloud2::ConstPtr& msg);

  /**
   * @brief 验证并精化位姿
   * @param T_map_base ArUco 求解的 map→base_link 位姿（作为初始猜测）
   * @param image_timestamp 图像时间戳（用于同步点云）
   * @return 验证结果（含 VGICP 精化后的位姿和质量指标）
   */
  PointCloudVerifyResult verify(const Eigen::Isometry3d& T_map_base,
                                double image_timestamp = 0.0);

  /**
   * @brief 是否已初始化（全局地图已加载）
   */
  bool isInitialized() const { return map_loaded_; }

 private:
  /**
   * @brief 加载全局地图
   */
  bool loadGlobalMap(const std::string& pcd_path);

  /**
   * @brief 旋转点云（绕 Z 轴）
   */
  CloudPtr rotateCloudYaw(const CloudConstPtr& cloud, double yaw_rad) const;

  /**
   * @brief 变换点云到 map 系
   */
  CloudPtr transformToMap(const CloudConstPtr& cloud_base,
                          const Eigen::Isometry3d& T_map_base) const;

  /**
   * @brief 从全局地图裁剪局部子图
   */
  CloudPtr cropLocalSubmap(const Eigen::Isometry3d& T_map_base) const;

  /**
   * @brief 点云预处理（距离裁剪 + z 过滤 + 体素下采样）
   */
  CloudPtr preprocessCloud(const CloudConstPtr& cloud) const;

  /**
   * @brief VGICP 两级配准
   * @param source 当前帧点云（base_link 系）
   * @param target 局部子图（map 系）
   * @param initial_guess 初始位姿（T_map_base）
   * @return 精化后的位姿和收敛信息
   */
  struct VgicpResult {
    bool converged = false;
    Eigen::Isometry3d T_map_base = Eigen::Isometry3d::Identity();
    double delta_translation = 0.0;
    double delta_rotation_deg = 0.0;
    double elapsed_ms = 0.0;
    CloudPtr aligned_source;  // 匹配后的点云
  };
  VgicpResult runVgicp(const CloudConstPtr& source, const CloudConstPtr& target,
                       const Eigen::Isometry3d& initial_guess) const;

  PointCloudVerifyConfig config_;
  CloudPtr global_map_;
  bool map_loaded_ = false;

  // 点云缓冲（用于时间戳同步）
  struct TimestampedCloud {
    double timestamp;
    sensor_msgs::PointCloud2::ConstPtr msg;
  };
  std::deque<TimestampedCloud> cloud_buffer_;
  static constexpr size_t kMaxCloudBufferSize = 20;
  static constexpr double kMaxCloudTimeDiff = 0.1;  // 最大允许时间差（秒）
  std::mutex cloud_mutex_;
};

}  // namespace aruco_reloc
