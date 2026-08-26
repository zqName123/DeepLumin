/**
 * @file eval_visualizer.hpp
 * @brief 离线评估 RViz 可视化模块
 *
 * 话题（/offline_eval/）：
 *   灰色 global_map           : query 附近裁剪的全局地图（参考背景）
 *   绿色 query_truth_aligned  : 用真值位姿变换 query（仅 truth_compare 模式）
 *   橙色 query_raw            : 重定位前的原始点云（LiDAR 坐标系，中心在 map 原点）
 *   青色 query_sc_initial_aligned : 用 Scan Context 初值变换 query（map 坐标系）
 *   红色 query_reloc_aligned  : 用 GICP 输出变换 query（map 坐标系）
 *   蓝色 gicp_target_submap   : GICP 配准目标子图
 *   青色箭头 pose_markers/initial_guess : SC 候选位姿（GICP 初值）
 *   红色箭头 pose_markers/reloc_pose    : GICP 输出位姿
 */

#pragma once

#include <relocalization/core/common.hpp>
#include <relocalization/interface/i_global_matcher.hpp>
#include <relocalization/core/keyframe_database.hpp>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>

namespace relocalization {

struct EvalVisualizationConfig {
  bool enable = false;
  std::string frame_id = "map";
  double crop_radius = 80.0;
  double wait_sec = 2.0;
  bool interactive = false;
  bool publish_truth_aligned = true;
  bool publish_reloc_aligned = true;
  bool publish_target_submap = true;
  /// global_map 来源：pcd=裁剪 global_map_pcd；keyframes=拼接附近关键帧（3D）
  std::string global_map_source = "pcd";
  int max_keyframes_for_map = 200;
};

class EvalVisualizer {
 public:
  explicit EvalVisualizer(ros::NodeHandle& nh, const EvalVisualizationConfig& config);

  /// 发布 query 附近裁剪的全局地图（灰色背景）
  void publishGlobalMap(const KeyframeDatabase& database, const Pose& center, double crop_radius);

  /**
   * @brief 有真值位姿的对比可视化（offline_eval 模式）
   *
   * 绿色=真值对齐，红色=重定位结果，蓝色=GICP target
   */
  void publishComparison(int query_id,
                         const CloudConstPtr& source,
                         const Pose& truth,
                         const MatchResult& result,
                         bool topk_hit,
                         bool success,
                         double translation_error,
                         double yaw_error_deg);

  /**
   * @brief 无真值位姿的可视化（bag_eval 推荐）
   *
   * 橙色 query_raw          : 重定位前原始点云（LiDAR 坐标系，显示在 map 原点）
   * 青色 query_sc_initial_aligned: SC 初值对齐后的点云（map 坐标系）
   * 红色 query_reloc_aligned: GICP 对齐后的点云（map 坐标系）
   * 蓝色 gicp_target_submap : GICP 配准目标子图
   * 青色箭头 initial_guess  : SC 候选位姿（GICP 初值）
   * 红色箭头 reloc_pose     : GICP 输出位姿
   *
   * @param sc_candidate Scan Context Top-1 候选位姿（id=-1 时不绘制初值箭头）
   */
  void publishRelocResult(int query_id,
                          const CloudConstPtr& source,
                          const MatchResult& result,
                          bool success,
                          const Pose& sc_candidate = Pose{});

  /// 等待用户观察（交互模式按 Enter，否则 sleep wait_sec）
  void waitForNext();

 private:
  sensor_msgs::PointCloud2 toMsg(const CloudConstPtr& cloud) const;
  /// 发布非空点云（frame_id = config_.frame_id）
  void publishCloud(const ros::Publisher& pub, const CloudConstPtr& cloud) const;
  /// 发布非空点云（使用指定 frame_id，用于 LiDAR 系原始点云）
  void publishCloudInFrame(const ros::Publisher& pub,
                           const CloudConstPtr& cloud,
                           const std::string& frame_id) const;
  void publishPoseMarkers(const Pose& truth, const MatchResult& result) const;
  void publishRelocPoseMarker(const MatchResult& result,
                              const Pose& sc_candidate) const;
  static Pose poseFromTransform(const Eigen::Matrix4d& transform);

  EvalVisualizationConfig config_;

  ros::Publisher global_map_pub_;       ///< /offline_eval/global_map
  ros::Publisher truth_cloud_pub_;      ///< /offline_eval/query_truth_aligned
  ros::Publisher reloc_cloud_pub_;      ///< /offline_eval/query_reloc_aligned
  ros::Publisher sc_initial_cloud_pub_; ///< /offline_eval/query_sc_initial_aligned
  ros::Publisher raw_cloud_pub_;        ///< /offline_eval/query_raw（重定位前 LiDAR 系）
  ros::Publisher target_submap_pub_;    ///< /offline_eval/gicp_target_submap
  ros::Publisher marker_pub_;           ///< /offline_eval/pose_markers
  ros::Publisher status_pub_;           ///< /offline_eval/status
  ros::Publisher focus_pose_pub_;       ///< /offline_eval/focus_pose
  bool global_map_published_ = false;
};

}  // namespace relocalization
