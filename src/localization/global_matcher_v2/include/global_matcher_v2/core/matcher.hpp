#pragma once

/**
 * @file matcher.hpp
 * @brief 全局匹配器：统一基于 small_gicp 的 icp / gicp / vgicp，
 *        并提供 fast_gicp 粗配准 + vgicp 精细配准的混合分支
 *
 * - icp/gicp 使用 small_gicp 点云 + KDTree；
 * - vgicp 使用 small_gicp 预构建的 GaussianVoxelMap（由 MapManager 按活动
 *   分片集合缓存，避免每帧重建）；
 * - gicp / vgicp / icp 可配置 Huber / Cauchy 鲁棒核，抑制离群对应点；
 * - 粗/细两阶段与原始 global_matcher 流程一致（scale 控制体素与对应距离）。
 */

#include <limits>
#include <string>

#include <Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <small_gicp/ann/gaussian_voxelmap.hpp>
#include <small_gicp/ann/kdtree.hpp>
#include <small_gicp/points/point_cloud.hpp>

namespace global_matcher_v2 {

struct MatcherParams {
  /// icp | gicp | vgicp | fast_gicp | fast_gicp_vgicp（fast_gicp 粗 + vgicp 细）
  std::string type = "fast_gicp_vgicp";
  /// true: 匹配前按 FOV（crop.*）裁剪 target（v1 行为，fast_gicp 推荐）；false: 使用整图/活动分片缓存结构
  bool use_crop = false;

  // 两阶段体素尺度（与原始一致：体素 = leaf * scale）
  double source_voxel_leaf = 0.08;
  double target_voxel_leaf = 0.08;
  double coarse_scale = 6.0;
  double fine_scale = 3.0;

  int max_iterations = 30;
  double transformation_epsilon = 1e-3;

  // 对应距离门限：粗 = coarse_override 或 max*coarse_scale；细同理
  double max_correspondence_distance = 3.5;
  double coarse_max_correspondence_distance = 0.0;
  double fine_max_correspondence_distance = 0.0;

  // 鲁棒核：none | huber | cauchy
  std::string robust_kernel = "huber";
  double robust_kernel_scale = 1.0;

  int num_threads = 4;                        ///< small_gicp 线程数
  int cov_num_neighbors = 20;                 ///< source 协方差估计近邻数
  int cov_num_threads = 4;
  int fast_gicp_num_threads = 4;              ///< fast_gicp 线程数
  int fast_gicp_correspondence_randomness = 20;

  // VGICP 体素分辨率（与 MapManager 预构建保持一致，仅记录/校验用）
  double vgicp_voxel_resolution = 0.5;
  double vgicp_coarse_scale = 10.0;
  double vgicp_fine_scale = 1.0;
};

struct MatchOutput {
  Eigen::Matrix4f coarse = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f final = Eigen::Matrix4f::Identity();
  double coarse_fitness = std::numeric_limits<double>::infinity();
  double fine_fitness = std::numeric_limits<double>::infinity();
  bool coarse_converged = false;
  bool fine_converged = false;
};

/**
 * @brief 匹配器（无状态，线程由调用方保证串行）
 */
class PointCloudMatcher {
 public:
  void configure(const MatcherParams& params) { params_ = params; }
  const MatcherParams& params() const { return params_; }

  /**
   * @brief 执行粗+细两阶段配准
   * @param source_pcl      源点云（局部/odom 系，未降采样）
   * @param target          target 点云（map 系，带协方差，由 MapManager 缓存）
   * @param target_pcl      target 的 PCL 表示（fast_gicp 使用）
   * @param target_kdtree   target 的 KDTree（icp/gicp 使用）
   * @param coarse_voxelmap VGICP 粗体素地图（可为空）
   * @param fine_voxelmap   VGICP 细体素地图（可为空）
   * @param init_T          T_map_source 初始猜测
   */
  MatchOutput match(const pcl::PointCloud<pcl::PointXYZ>& source_pcl,
                    const small_gicp::PointCloud& target,
                    const pcl::PointCloud<pcl::PointXYZ>& target_pcl,
                    const small_gicp::KdTree<small_gicp::PointCloud>& target_kdtree,
                    const small_gicp::GaussianVoxelMap* coarse_voxelmap,
                    const small_gicp::GaussianVoxelMap* fine_voxelmap,
                    const Eigen::Matrix4f& init_T) const;

 private:
  MatcherParams params_;
};

}  // namespace global_matcher_v2
