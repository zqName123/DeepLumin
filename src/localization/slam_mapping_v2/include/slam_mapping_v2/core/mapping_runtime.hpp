#pragma once

/**
 * @file mapping_runtime.hpp
 * @brief slam_mapping_v2 运行时：消息订阅驱动的多线程建图
 *
 * 线程模型：
 * - 主线程：消费 odom/scan → 全局地图增量插入 + 关键帧判定（仅写 pending）
 * - 后端线程：合并关键帧 → GNSS 因子 + 回环检测 + ISAM2 优化
 * - 可视化线程（节点侧）：取关键帧/LoopData 快照发布
 */

#include "slam_mapping_v2/common/config.hpp"
#include "slam_mapping_v2/common/types.hpp"
#include "slam_mapping_v2/core/global_map.hpp"
#include "slam_mapping_v2/core/gnss_optimizer.hpp"
#include "slam_mapping_v2/core/i_loop_detector.hpp"
#include "slam_mapping_v2/core/i_visual_feature_provider.hpp"
#include "slam_mapping_v2/core/keyframe_manager.hpp"
#include "slam_mapping_v2/core/map_saver.hpp"
#include "slam_mapping_v2/core/pose_graph_optimizer.hpp"
#include "slam_mapping_v2/core/visual_feature_buffer.hpp"

#include <opencv2/core.hpp>

#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <thread>
#include <utility>
#include <vector>
#include <chrono>

namespace slam_mapping_v2 {

/// 后端/可视化线程间共享的只读结果快照
struct LoopData {
  std::vector<std::pair<int, int>> loop_pairs;
  PointCloudPtr current_submap;
  PointCloudPtr history_submap;
  std::vector<Vec3d> gnss_factor_positions;
  std::vector<Vec3d> gnss_rejected_positions;
  std::vector<std::pair<int, Vec3d>> gnss_accepted_factors;
  std::vector<std::pair<int, Vec3d>> gnss_rejected_factors;
  bool map_enu_valid = false;
  Mat3d map_enu_rot = Mat3d::Identity();
  Vec3d map_enu_trans = Vec3d::Zero();
  bool map_odom_valid = false;
  Eigen::Matrix4d t_map_odom = Eigen::Matrix4d::Identity();
};

class MappingRuntime {
 public:
  MappingRuntime() = default;
  ~MappingRuntime();

  bool initialize(const MappingConfig& cfg);

  bool start(const StartRequest& req, double now_sec);
  bool stop(const StopRequest& req);
  bool save(const SaveRequest& req);
  void reset();

  void feedOdom(const FrameInput& odom);
  void feedScan(const FrameInput& scan);
  void feedGnss(const GnssObservation& gnss);
  void feedImage(const cv::Mat& image, double timestamp);

  bool processOnce();

  void startThreads();
  void stopThreads();
  void shutdown();

  RuntimeStatus status() const;
  std::vector<Keyframe> keyframes() const;
  LoopData loopData() const;
  FrameInput latestFrame() const;
  /// 全局地图导出（map 系降采样缓存，回环优化后自动重建）
  PointCloudPtr mapCloud() const;
  bool mapOdomTransform(Eigen::Matrix4d* t_map_odom) const;

 private:
  bool popFrame(FrameInput* odom, FrameInput* scan);
  void associateGnssToKeyframe(Keyframe* kf);
  void backendLoop();
  void visualFeatLoop();
  void updateLoopData(const Keyframe* latest_kf, const LoopResult& r);
  void updateStatusFromLoop(const LoopResult& r);
  RuntimeStatus statusSnapshot() const;

  MappingConfig cfg_;
  bool initialized_ = false;
  bool stop_threads_ = false;

  KeyframeManager kf_manager_;
  GlobalMap global_map_;
  GnssOptimizer gnss_;
  PoseGraphOptimizer optimizer_;
  std::unique_ptr<ILoopDetector> detector_;
  MapSaver map_saver_;

  mutable std::shared_mutex kf_mutex_;
  std::vector<Keyframe> keyframes_;
  std::mutex pending_mutex_;
  std::condition_variable pending_cv_;
  std::vector<Keyframe> pending_;
  Keyframe last_keyframe_;  ///< 主线程私有：上一关键帧（判定用）

  std::mutex input_mutex_;
  std::deque<FrameInput> odom_queue_;
  std::deque<FrameInput> scan_queue_;
  std::deque<GnssObservation> gnss_queue_;
  std::deque<GnssObservation> gnss_buffer_;  ///< 主线程私有

  mutable std::mutex latest_mutex_;
  mutable std::mutex map_mutex_;  ///< 保护 global_map_（主线程写，可视化线程读）
  FrameInput latest_frame_;

  mutable std::mutex loop_mutex_;
  LoopData loop_data_;

  mutable std::mutex status_mutex_;
  RuntimeStatus status_;

  std::thread backend_thread_;
  std::uint64_t frame_count_ = 0;
  bool loop_enabled_ = false;

  // ===== 视觉回环相关 =====
  std::thread visual_feat_thread_;
  std::deque<std::pair<cv::Mat, double>> image_queue_;
  std::mutex image_mutex_;
  std::condition_variable image_cv_;
  std::shared_ptr<VisualFeatureBuffer> feat_buffer_;
  std::unique_ptr<IVisualFeatureProvider> feat_provider_;
};

}  // namespace slam_mapping_v2