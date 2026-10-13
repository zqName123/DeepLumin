/**
 * @file localization_v3_node.cpp
 * @brief localization_v3 定位融合节点（薄编排层）
 *
 * 职责边界：本文件只做 ROS 消息 <-> 纯数据转换、调用 core 模块、发布结果。
 * 业务判定全部在 core：
 *   fusion_core      位姿写入策略（人工 / 验证 / jump guard / 恢复）
 *   quality_gate     质量三级分级
 *   jump_guard       TRACKING 跳变防护
 *   failure_detector 快速失效信号（出图 / 看门狗 / 超时）
 *   state_machine    五态转移与恢复确认
 *   init_arbiter     初始化来源择优
 *
 * 数据流：
 *   slam_odometry_v2   /slam_odometry/odom                  odom->base_link
 *   global_matcher_v2  /global_matcher/result      map->odom
 *   relocalization     /pc_reloc/result      map->base_link
 *   /initialpose                                          人工初值（最高优先级）
 *
 * 融合公式（不变）：
 *   map_odom = map_base ⊕ (odom_base)⁻¹
 *   map_base = map_odom ⊕ odom_base
 */

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <deeplumin_msgs/FusionStatus.h>
#include <deeplumin_msgs/GlobalMatchResult.h>
#include <deeplumin_msgs/LocPoint.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_broadcaster.h>

#include <deeplumin_health_checker/health_checker/health_checker.h>
#include <syslog/logger.h>

#include "localization_v3/core/failure_detector.hpp"
#include "localization_v3/core/fusion_core.hpp"
#include "localization_v3/core/init_arbiter.hpp"
#include "localization_v3/core/quality_gate.hpp"
#include "localization_v3/core/state_machine.hpp"
#include "localization_v3/ros/localization_v3_config.hpp"
#include "localization_v3/ros/relocalization_ros_adapter.hpp"
#include "localization_v3/ros/status_publisher.hpp"

namespace localization_v3 {

namespace {

using State = LocalizationStateMachine::State;

/// 每行号独立的节流器（替代 ROS_INFO_THROTTLE）
template <int Line>
struct ThrottleByLine {
  static bool shouldLog(double rate) {
    static std::chrono::steady_clock::time_point last_log_time;
    const auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration<double>(now - last_log_time).count() >= rate) {
      last_log_time = now;
      return true;
    }
    return false;
  }
};
#define SHOULD_LOG(rate) ThrottleByLine<__LINE__>::shouldLog(rate)

Pose3d poseFromRos(const geometry_msgs::Pose& msg) {
  Pose3d pose;
  pose.position << msg.position.x, msg.position.y, msg.position.z;
  pose.orientation = Quatd(msg.orientation.w, msg.orientation.x, msg.orientation.y,
                           msg.orientation.z);
  if (!isFinite(pose.orientation)) {
    pose.orientation = Quatd::Identity();
  } else {
    pose.orientation.normalize();
  }
  return pose;
}

void fillPose(const Pose3d& pose, geometry_msgs::Pose* msg) {
  msg->position.x = pose.position.x();
  msg->position.y = pose.position.y();
  msg->position.z = pose.position.z();
  const Quatd q = pose.orientation.normalized();
  msg->orientation.x = q.x();
  msg->orientation.y = q.y();
  msg->orientation.z = q.z();
  msg->orientation.w = q.w();
}

geometry_msgs::TransformStamped toTf(const Pose3d& pose, const std::string& parent,
                                     const std::string& child, const ros::Time& stamp) {
  geometry_msgs::TransformStamped msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = parent;
  msg.child_frame_id = child;
  msg.transform.translation.x = pose.position.x();
  msg.transform.translation.y = pose.position.y();
  msg.transform.translation.z = pose.position.z();
  const Quatd q = pose.orientation.normalized();
  msg.transform.rotation.x = q.x();
  msg.transform.rotation.y = q.y();
  msg.transform.rotation.z = q.z();
  msg.transform.rotation.w = q.w();
  return msg;
}

Eigen::Matrix4f toMatrix4f(const Pose3d& pose) {
  Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();
  mat.block<3, 3>(0, 0) = pose.orientation.normalized().toRotationMatrix().cast<float>();
  mat.block<3, 1>(0, 3) = pose.position.cast<float>();
  return mat;
}

}  // namespace

class LocalizationV3Node {
 public:
  LocalizationV3Node(ros::NodeHandle& nh, ros::NodeHandle& pnh)
      : nh_(nh), pnh_(pnh), config_(loadLocalizationV3Config(pnh_)) {
    // ---- 健康监测 ----
    health_checker_ = std::make_shared<deeplumin_health_checker::HealthChecker>(nh_, pnh_);
    health_checker_->ENABLE();
    health_checker_->NODE_ACTIVATE();

    // ---- 轨迹落盘（可选） ----
    std::string traj_path;
    pnh_.param<std::string>("traj_path", traj_path, "");
    if (!traj_path.empty()) {
      traj_file_.open(traj_path);
      if (traj_file_.is_open()) {
        traj_file_ << "timestamp, x, y, z, roll, pitch, yaw\n";
        SYS_LOG(syslog::LogSeverity::INFO, true) << "trajectory saving enabled -> " << traj_path;
      } else {
        SYS_LOG(syslog::LogSeverity::WARNING, true)
            << "failed to open trajectory file: " << traj_path;
      }
    }

    // ---- core 模块配置 ----
    fusion_.setConfig(config_.fusion);
    state_machine_.configure(config_.init, config_.reloc_policy, config_.tracking);
    failure_detector_.configure(config_.failure);

    // ---- 重定位接口 ----
    relocalization_ = std::make_unique<RelocalizationRosAdapter>(
        nh_, config_.relocalization_result_topic, config_.relocalization_command_topic);
    relocalization_->setResultCallback(
        [this](const RelocalizationSignal& signal) { onRelocalizationResult(signal); });

    // ---- 订阅 ----
    slam_sub_ = nh_.subscribe(config_.slam_odom_topic, 100, &LocalizationV3Node::onSlamOdom, this);
    gm_sub_ = nh_.subscribe(config_.global_matcher_result_topic, 5,
                            &LocalizationV3Node::onGlobalMatch, this);
    initial_pose_sub_ = nh_.subscribe(config_.initial_pose_topic, 5,
                                      &LocalizationV3Node::onInitialPose, this);
    if (config_.publish_current_scan_in_map) {
      cloud_sub_ = nh_.subscribe(config_.source_cloud_topic, 5,
                                 &LocalizationV3Node::onSourceCloud, this);
    }

    // ---- 发布 ----
    fused_odom_pub_ = nh_.advertise<nav_msgs::Odometry>(config_.fused_odom_topic, 50);
    map_to_odom_pub_ = nh_.advertise<nav_msgs::Odometry>(config_.map_to_odom_topic, 10);
    path_pub_ = nh_.advertise<nav_msgs::Path>(config_.fused_path_topic, 5);
    status_pub_ = nh_.advertise<deeplumin_msgs::FusionStatus>(config_.status_topic, 10);
    gm_anchor_pub_ =
        nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(config_.gm_anchor_topic, 1, true);
    loc_point_pub_ = nh_.advertise<deeplumin_msgs::LocPoint>(config_.loc_point_topic, 10);
    if (config_.publish_current_scan_in_map) {
      current_scan_map_pub_ =
          nh_.advertise<sensor_msgs::PointCloud2>(config_.current_scan_in_map_topic, 2);
    }
    path_.header.frame_id = config_.map_frame;

    // ---- 定时器 ----
    timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(1.0, config_.output_rate_hz)),
                             &LocalizationV3Node::onTimer, this);

    SYS_LOG(syslog::LogSeverity::INFO, true)
        << "localization_v3 started slam=" << config_.slam_odom_topic
        << " GM=" << config_.global_matcher_result_topic
        << " reloc_result=" << config_.relocalization_result_topic
        << " reloc_cmd=" << config_.relocalization_command_topic
        << " fused=" << config_.fused_odom_topic;
  }

  ~LocalizationV3Node() {
    if (traj_file_.is_open()) {
      traj_file_.close();
    }
  }

 private:
  // ===================================================================
  // 回调
  // ===================================================================

  void onSlamOdom(const nav_msgs::Odometry::ConstPtr& msg) {
    const double now = ros::Time::now().toSec();
    fusion_.setOdomBase(poseFromRos(msg->pose.pose));
    latest_twist_ = msg->twist;
    last_slam_stamp_ = now;
    failure_detector_.onSlamUpdate(now);

    state_machine_.onSlamOdom();
    if (state_machine_.state() == State::kRelocalizing && relocalizing_since_ <= 0.0) {
      relocalizing_since_ = now;
    }
    dispatchRelocalizationCommand();
    syncFailureDetectorState(now);

    publishCurrentScanInMap();
    publishOutput(now);
  }

  void onGlobalMatch(const deeplumin_msgs::GlobalMatchResult::ConstPtr& msg) {
    const double now = ros::Time::now().toSec();
    last_gm_stamp_ = now;

    // ---- 失败结果：仅喂失效判定，不写位姿 ----
    if (!msg->success) {
      failure_detector_.onGmResult(false, QualityLevel::kBad, msg->reject_reason, now);
      handleGmFailure(msg->reject_reason, now);
      return;
    }

    // ---- 候选 map->odom（统一转换，一次计算） ----
    const Pose3d pose = poseFromRos(msg->pose.pose);
    const bool pose_is_map_base =
        (config_.gm_result_pose_frame == "base" || config_.gm_result_pose_frame == "base_link");
    if (pose_is_map_base) {
      if (!fusion_.hasSlam()) {
        return;
      }
      gm_candidate_map_odom_ = compose(pose, inverse(fusion_.odomBase()));
    } else {
      gm_candidate_map_odom_ = pose;
    }

    // ---- 质量分级（只做一次，多处共用） ----
    const QualityLevel quality =
        classifyQuality(msg->inlier_ratio, msg->inlier_rmse, config_.quality);
    failure_detector_.onGmResult(true, quality, msg->reject_reason, now);

    const State state = state_machine_.state();
    if (state == State::kUninitialized || state == State::kRelocalizing) {
      cacheGmCandidate(*msg);
      tryCompleteInitialization(now);
      return;
    }
    if (state == State::kTracking) {
      handleGmTracking(quality, *msg, now);
      return;
    }
    handleGmRecovery(quality, *msg, now);
  }

  /// TRACKING 时 GM 失败：快速计数 / 出图快路径
  void handleGmFailure(const std::string& reject_reason, double now) {
    const State state = state_machine_.state();
    if (state != State::kTracking) {
      if (state == State::kTrackingLost) {
        state_machine_.onRecoveryAttemptFailed();  // 恢复尝试失败，打断确认窗口
      }
      return;
    }

    // 人工验证期：失败只计"验证失败"（独立判据），不计入跟踪失败计数
    if (fusion_.manualVerifyPending()) {
      if (failure_detector_.mapMissExceeded()) {
        failManualVerify("out of map (" + reject_reason + ")", now);
        return;
      }
      onVerifyFailure(reject_reason, now);
      return;
    }

    if (failure_detector_.mapMissExceeded()) {
      // 出图快路径：直接判定跟踪丢失
      state_machine_.forceTrackingLost();
      if (state_machine_.state() == State::kTrackingLost) {
        enterTrackingLost("out of map (" + reject_reason + ")", now);
      }
      return;
    }

    state_machine_.onTrackingFailure();
    if (state_machine_.state() == State::kTrackingLost) {
      enterTrackingLost("GM consecutive failures=" +
                            std::to_string(state_machine_.getGmConsecutiveFailures()),
                        now);
    }
  }

  /// TRACKING：质量计数 + 位姿写入（settle / 人工验证 / jump guard 均在 FusionCore）
  void handleGmTracking(QualityLevel quality, const deeplumin_msgs::GlobalMatchResult& msg,
                        double now) {
    // ---- 人工验证期：非 good 结果不写位姿，只计验证失败（与跟踪失败计数隔离） ----
    if (fusion_.manualVerifyPending()) {
      if (quality != QualityLevel::kGood) {
        onVerifyFailure(std::string("quality=") + qualityName(quality) +
                            " inlier=" + std::to_string(msg.inlier_ratio) +
                            " rmse=" + std::to_string(msg.inlier_rmse),
                        now);
        return;
      }
      // good 结果落到下方 applyGlobalMatch：验证采纳（不经 jump guard）
    } else if (quality == QualityLevel::kBad) {
      // ---- 质量计数（bad/退化达阈值 → 跟踪丢失） ----
      state_machine_.onTrackingFailure();
      state_machine_.resetDegradedCounter();
      if (state_machine_.state() == State::kTrackingLost) {
        enterTrackingLost("GM quality bad: inlier=" + std::to_string(msg.inlier_ratio) +
                              " rmse=" + std::to_string(msg.inlier_rmse),
                          now);
      }
      return;  // bad 结果不应用位姿
    } else if (quality == QualityLevel::kDegraded) {
      state_machine_.onTrackingDegraded();
      if (state_machine_.state() == State::kTrackingLost) {
        enterTrackingLost("GM quality degraded for " +
                              std::to_string(state_machine_.getGmDegradedCount()) + " frames",
                          now);
      }
    } else {
      state_machine_.resetGmCounter();
      state_machine_.resetDegradedCounter();
    }

    // ---- 位姿写入：settle → 人工验证 → jump guard ----
    const PoseUpdateResult result = fusion_.applyGlobalMatch(gm_candidate_map_odom_, quality, now);
    switch (result) {
      case PoseUpdateResult::kAcceptedVerified:
        failure_detector_.onCorrectionAccepted(now);
        failure_reason_.clear();
        latest_map_id_ = msg.map_id;
        if (config_.publish_gm_anchor) {
          publishGmAnchor(fusion_.fusedPose(), now);  // 验证后位姿锚定 GM
        }
        publishMapToOdom();
        SYS_LOG(syslog::LogSeverity::INFO, true)
            << "manual pose verified by global matcher map_id=" << msg.map_id
            << " inlier=" << msg.inlier_ratio << " rmse=" << msg.inlier_rmse
            << " pos=(" << fusion_.fusedPose().position.x() << ", "
            << fusion_.fusedPose().position.y() << ")";
        break;

      case PoseUpdateResult::kAccepted:
        failure_detector_.onCorrectionAccepted(now);
        latest_map_id_ = msg.map_id;
        publishMapToOdom();
        break;

      case PoseUpdateResult::kRejectedStale:
        // settle 窗口内：等待 GM 以人工锚点算出的新鲜结果
        break;

      case PoseUpdateResult::kRejectedQuality:
        break;

      case PoseUpdateResult::kRejectedGuard: {
        const Pose3d old_fused = fusion_.fusedPose();
        const Pose3d new_fused = compose(gm_candidate_map_odom_, fusion_.odomBase());
        const double jump_dx = translationDiff(old_fused, new_fused);
        const double jump_dyaw = yawDiffDeg(old_fused, new_fused);
        SYS_LOG(syslog::LogSeverity::WARNING, false)
            << "reject global matcher result (jump guard): trans=" << jump_dx << " (limit "
            << config_.fusion.jump_guard.hard_max_translation << "), yaw=" << jump_dyaw
            << " (limit " << config_.fusion.jump_guard.hard_max_yaw_deg << ")";
        break;
      }

      case PoseUpdateResult::kConfirmPending:
        if (SHOULD_LOG(2.0)) {
          SYS_LOG(syslog::LogSeverity::INFO, false)
              << "large jump pending confirmation: " << fusion_.jumpGuard().pendingCount()
              << "/" << config_.fusion.jump_guard.confirm_frames;
        }
        break;
    }
  }

  /// TRACKING_LOST / LOCALIZATION_FAILURE：恢复写入（无 jump guard）+ 确认窗口
  void handleGmRecovery(QualityLevel quality, const deeplumin_msgs::GlobalMatchResult& msg,
                        double now) {
    if (quality != QualityLevel::kGood) {
      state_machine_.onRecoveryAttemptFailed();
      return;
    }
    // 恢复期允许大跳变：无 jump guard，直接采纳（位姿已不可信，恢复结果是更优估计）
    if (!fusion_.applyRecoveryMatch(gm_candidate_map_odom_, now)) {
      state_machine_.onRecoveryAttemptFailed();
      return;
    }
    failure_detector_.onCorrectionAccepted(now);
    state_machine_.onGlobalMatchSuccess();
    if (state_machine_.state() == State::kTracking) {
      recoveredFrom("global matcher", now);
      SYS_LOG(syslog::LogSeverity::INFO, true)
          << "recovered via global matcher map_id=" << msg.map_id
          << " inlier=" << msg.inlier_ratio << " rmse=" << msg.inlier_rmse;
    } else if (SHOULD_LOG(1.0)) {
      SYS_LOG(syslog::LogSeverity::INFO, false)
          << "GM recovery accumulating, waiting for confirmation window";
    }
  }

  void onInitialPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    if (!config_.init.use_manual_initialpose) {
      return;
    }
    const double now = ros::Time::now().toSec();

    Pose3d map_base = poseFromRos(msg->pose.pose);
    // 兼容 RViz 固定坐标系被设为 odom/base_link 的情况
    const std::string pose_frame = msg->header.frame_id;
    if (!pose_frame.empty() && pose_frame != config_.map_frame) {
      if (pose_frame == config_.odom_frame) {
        map_base = compose(fusion_.mapOdom(), map_base);
      } else if (pose_frame == config_.base_frame) {
        map_base = compose(fusion_.mapOdom(), fusion_.odomBase());
      } else {
        SYS_LOG(syslog::LogSeverity::WARNING, true)
            << "unknown initial pose frame '" << pose_frame << "', assume " << config_.map_frame;
      }
    }

    // z 补全（人工位姿附近地图点云高度范围中值）
    double init_z = 0.0;
    if (computeInitPoseZ(map_base.position, &init_z)) {
      map_base.position.z() = init_z;
    }

    // 人工位姿：无条件写入（不经 jump guard），开启 settle + 人工验证
    if (!fusion_.applyManualPose(map_base, /*require_slam=*/false, now)) {
      SYS_LOG(syslog::LogSeverity::WARNING, true) << "invalid manual initial pose, ignored";
      return;
    }
    state_machine_.onManualPose();
    failure_detector_.onCorrectionAccepted(now);
    manual_verify_deadline_ = now + config_.manual_verify_timeout_sec;
    verify_fail_count_ = 0;
    last_manual_stamp_ = now;
    failure_reason_.clear();
    syncFailureDetectorState(now);
    dispatchRelocalizationCommand();

    if (config_.publish_gm_anchor) {
      publishGmAnchor(map_base, now);  // 通知 GM 以人工位姿重锚
    }
    publishMapToOdom();
    SYS_LOG(syslog::LogSeverity::INFO, true)
        << "reset map->odom from manual initial pose frame=" << pose_frame
        << " z=" << map_base.position.z() << " (verify within "
        << config_.manual_verify_timeout_sec << "s)";
  }

  void onRelocalizationResult(const RelocalizationSignal& signal) {
    const State state = state_machine_.state();
    if (state != State::kRelocalizing && state != State::kTrackingLost) {
      return;  // 其他状态不接受重定位结果（人工/跟踪优先）
    }
    const double now = ros::Time::now().toSec();
    last_reloc_stamp_ = now;

    if (!signal.success) {
      if (state == State::kTrackingLost) {
        state_machine_.onRecoveryFailed();
        SYS_LOG(syslog::LogSeverity::WARNING, false)
            << "relocalization failed, consecutive="
            << state_machine_.getRelocConsecutiveFailures() << "/"
            << config_.tracking.reloc_failure_threshold;
        if (state_machine_.state() == State::kLocalizationFailure) {
          declareFailure("relocalization recovery failed (" +
                             std::to_string(state_machine_.getRelocConsecutiveFailures()) +
                             " consecutive failures)",
                         now);
        }
      }
      return;
    }
    if (config_.relocalization_require_accepted && !signal.accepted) {
      SYS_LOG(syslog::LogSeverity::WARNING, false) << "relocalization result not accepted";
      return;
    }
    if (signal.inlier_ratio < config_.relocalization_min_inlier_ratio) {
      SYS_LOG(syslog::LogSeverity::WARNING, false)
          << "relocalization inlier ratio " << signal.inlier_ratio << " below limit "
          << config_.relocalization_min_inlier_ratio;
      return;
    }
    if (signal.inlier_rmse > config_.relocalization_max_inlier_rmse) {
      SYS_LOG(syslog::LogSeverity::WARNING, false)
          << "relocalization rmse " << signal.inlier_rmse << " above limit "
          << config_.relocalization_max_inlier_rmse;
      return;
    }

    // 暂存结果（初始化仲裁用）
    reloc_result_pending_ = true;
    reloc_result_map_base_ = signal.map_base;
    reloc_result_inlier_ratio_ = signal.inlier_ratio;
    reloc_result_inlier_rmse_ = signal.inlier_rmse;
    reloc_result_map_id_ = signal.map_id;
    reloc_result_method_ = signal.method;
    reloc_result_stamp_ = now;

    if (state == State::kTrackingLost) {
      // P5 修复：applyRelocalization 内部做 map_base → map_odom 反推
      if (!fusion_.applyRelocalization(signal.map_base, /*require_slam=*/false, now)) {
        return;
      }
      failure_detector_.onCorrectionAccepted(now);
      state_machine_.onRelocalizationResult(true);
      if (state_machine_.state() == State::kTracking) {
        recoveredFrom("relocalization", now);
        SYS_LOG(syslog::LogSeverity::INFO, true)
            << "recovered from tracking lost via relocalization map_id=" << signal.map_id
            << " inlier=" << signal.inlier_ratio << " rmse=" << signal.inlier_rmse;
      } else if (SHOULD_LOG(1.0)) {
        SYS_LOG(syslog::LogSeverity::INFO, false)
            << "relocalization recovery accumulating, waiting for confirmation window";
      }
      return;
    }
    tryCompleteInitialization(now);
  }

  void onSourceCloud(const sensor_msgs::PointCloud2::ConstPtr& msg) { latest_cloud_ = msg; }

  // ===================================================================
  // 初始化仲裁
  // ===================================================================

  void cacheGmCandidate(const deeplumin_msgs::GlobalMatchResult& msg) {
    have_gm_candidate_ = true;
    gm_candidate_inlier_ratio_ = msg.inlier_ratio;
    gm_candidate_inlier_rmse_ = msg.inlier_rmse;
    gm_candidate_map_id_ = msg.map_id;
  }

  void tryCompleteInitialization(double now) {
    if (state_machine_.state() != State::kRelocalizing) {
      return;
    }
    const bool reloc_fresh =
        reloc_result_pending_ &&
        (now - reloc_result_stamp_) <= config_.init_arbiter.reloc_result_max_age_sec;
    const bool gm_ready = have_gm_candidate_;
    const double elapsed = relocalizing_since_ > 0.0 ? (now - relocalizing_since_) : 0.0;

    const InitSource source =
        selectInitSource(config_.init_arbiter, reloc_fresh, gm_ready,
                         initScore(reloc_result_inlier_ratio_, reloc_result_inlier_rmse_),
                         initScore(gm_candidate_inlier_ratio_, gm_candidate_inlier_rmse_),
                         elapsed);
    if (source == InitSource::kRelocalization) {
      completeWithReloc(now);
    } else if (source == InitSource::kGlobalMatcher) {
      completeWithGm(now);
    }
  }

  void completeWithReloc(double now) {
    if (!fusion_.applyRelocalization(reloc_result_map_base_, /*require_slam=*/false, now)) {
      SYS_LOG(syslog::LogSeverity::WARNING, false) << "invalid relocalization pose, ignored";
      reloc_result_pending_ = false;
      return;
    }
    failure_detector_.onCorrectionAccepted(now);
    latest_map_id_ = reloc_result_map_id_;
    state_machine_.onRelocalizationResult(true);
    dispatchRelocalizationCommand();
    publishMapToOdom();
    if (config_.publish_gm_anchor) {
      publishGmAnchor(reloc_result_map_base_, now);  // 初始化仲裁重定位胜出：GM 重锚
    }
    SYS_LOG(syslog::LogSeverity::INFO, true)
        << "initialized from relocalization method=" << reloc_result_method_ << " map_id="
        << reloc_result_map_id_ << " (reloc inlier=" << reloc_result_inlier_ratio_ << " rmse="
        << reloc_result_inlier_rmse_ << " vs gm inlier=" << gm_candidate_inlier_ratio_
        << " rmse=" << gm_candidate_inlier_rmse_ << ")";
  }

  void completeWithGm(double now) {
    if (!fusion_.applyRecoveryMatch(gm_candidate_map_odom_, now)) {
      SYS_LOG(syslog::LogSeverity::WARNING, false) << "invalid global matcher candidate, ignored";
      return;
    }
    failure_detector_.onCorrectionAccepted(now);
    latest_map_id_ = gm_candidate_map_id_;
    state_machine_.onRelocalizationResult(true);
    dispatchRelocalizationCommand();
    publishMapToOdom();
    reloc_result_pending_ = false;
    SYS_LOG(syslog::LogSeverity::INFO, true)
        << "initialized from global matcher map_id=" << gm_candidate_map_id_
        << " (gm inlier=" << gm_candidate_inlier_ratio_ << " rmse=" << gm_candidate_inlier_rmse_
        << " vs reloc inlier=" << reloc_result_inlier_ratio_
        << " rmse=" << reloc_result_inlier_rmse_ << ")";
  }

  // ===================================================================
  // 定时输出 / 失效超时
  // ===================================================================

  void onTimer(const ros::TimerEvent&) {
    const double now = ros::Time::now().toSec();

    if (!fusion_.hasSlam()) {
      publishIdentityMapOdom();
      if (SHOULD_LOG(10.0)) {
        SYS_LOG(syslog::LogSeverity::WARNING, false)
            << "waiting for slam odom: " << config_.slam_odom_topic;
      }
      publishStatus(now);
      return;
    }

    checkFailureSignals(now);

    tryCompleteInitialization(now);
    publishCurrentScanInMap();
    publishOutput(now);
  }

  /// 快速失效判定：SLAM 断流 / 无修正看门狗 / 恢复超时 / 人工验证超时
  void checkFailureSignals(double now) {
    const State state = state_machine_.state();

    // 1) SLAM 断流：定位输出已冻结，立即判失效
    if (failure_detector_.slamTimeout(now) && state_machine_.initialized() &&
        state != State::kLocalizationFailure) {
      state_machine_.forceLocalizationFailure();
      declareFailure("slam odometry timeout", now);
      return;
    }

    // 2) 人工验证超时：位姿不可信，交恢复流程（reloc/GM 全局搜索）
    //    仅在 TRACKING 处理：进 LOST 时 enterTrackingLost 已结束验证期，不会重复触发
    if (state == State::kTracking && fusion_.manualVerifyPending() &&
        now > manual_verify_deadline_) {
      failManualVerify("manual pose not verified within " +
                           std::to_string(config_.manual_verify_timeout_sec) + "s",
                       now);
      return;
    }

    // 3) 无修正看门狗：长时间无任何被采纳的修正（被 jump guard 连续拒绝 / GM 静默）
    if (state == State::kTracking && !fusion_.manualVerifyPending() &&
        failure_detector_.noCorrectionTimeout(now)) {
      state_machine_.forceTrackingLost();
      if (state_machine_.state() == State::kTrackingLost) {
        enterTrackingLost("no accepted pose correction within " +
                              std::to_string(config_.failure.no_correction_timeout_sec) + "s",
                          now);
      }
      return;
    }

    // 4) 恢复预算耗尽仍无确认恢复 → 定位失效（代表无法恢复，需人工）
    if (state == State::kTrackingLost && failure_detector_.lostTimeout(now)) {
      state_machine_.forceLocalizationFailure();
      declareFailure("recovery budget exhausted (" +
                         std::to_string(config_.failure.lost_timeout_sec) + "s): " +
                         (failure_reason_.empty() ? "no confirmed recovery" : failure_reason_),
                     now);
    }
  }

  // ===================================================================
  // 状态流转辅助
  // ===================================================================

  void enterTrackingLost(const std::string& reason, double now) {
    // 进入恢复流程：结束人工验证期（后续由 reloc/GM 恢复写位姿，无 jump guard）
    fusion_.cancelManualVerify();
    verify_fail_count_ = 0;
    failure_detector_.onTrackingLost(now);
    failure_reason_ = reason;
    dispatchRelocalizationCommand();
    SYS_LOG(syslog::LogSeverity::WARNING, true)
        << "tracking lost: " << reason << ", starting relocalization recovery";
  }

  /// 人工验证期的失败：独立计数（不计入跟踪失败），达阈值交恢复流程
  void onVerifyFailure(const std::string& detail, double now) {
    ++verify_fail_count_;
    if (verify_fail_count_ < config_.manual_verify_fail_threshold) {
      return;
    }
    failManualVerify("manual pose unverified (" + detail + ")", now);
  }

  /// 验证失败定论：退出验证期并转恢复流程（reason 由调用方给）
  void failManualVerify(const std::string& reason, double now) {
    fusion_.cancelManualVerify();
    verify_fail_count_ = 0;
    state_machine_.forceTrackingLost();
    if (state_machine_.state() == State::kTrackingLost) {
      enterTrackingLost(reason, now);
    }
  }

  void declareFailure(const std::string& reason, double now) {
    failure_reason_ = reason;
    dispatchRelocalizationCommand();
    SYS_LOG(syslog::LogSeverity::ERROR, true) << "localization failure: " << reason;
    (void)now;
  }

  void recoveredFrom(const std::string& source, double now) {
    failure_detector_.onTrackingRecovered(now);
    failure_reason_.clear();
    dispatchRelocalizationCommand();
    publishMapToOdom();
    SYS_LOG(syslog::LogSeverity::INFO, true) << "tracking recovered via " << source;
  }

  /// 状态机转移后同步失效判定器的 lost 计时
  void syncFailureDetectorState(double now) {
    const bool lost = state_machine_.state() == State::kTrackingLost;
    if (lost && !lost_synced_) {
      failure_detector_.onTrackingLost(now);
      lost_synced_ = true;
    } else if (!lost && lost_synced_) {
      failure_detector_.onTrackingRecovered(now);
      lost_synced_ = false;
    }
    if (state_machine_.state() == State::kTracking) {
      failure_detector_.onTrackingRecovered(now);
    }
  }

  void dispatchRelocalizationCommand() {
    const RelocCommand cmd = state_machine_.takeCommand();
    if (cmd == RelocCommand::kStart) {
      relocalization_->start();
    } else if (cmd == RelocCommand::kStop) {
      relocalization_->stop();
    }
    syncFailureDetectorState(ros::Time::now().toSec());
  }

  // ===================================================================
  // 输出
  // ===================================================================

  void publishOutput(double now) {
    if (!fusion_.hasSlam()) {
      return;
    }
    const bool output_ready = fusion_.isInitialized() || !config_.init.require_initial_pose;
    const Pose3d map_base = fusion_.fusedPose();
    const ros::Time stamp = ros::Time::now();

    if (output_ready) {
      nav_msgs::Odometry odom;
      odom.header.stamp = stamp;
      odom.header.frame_id = config_.map_frame;
      odom.child_frame_id = config_.base_frame;
      fillPose(map_base, &odom.pose.pose);
      odom.twist = latest_twist_;
      fused_odom_pub_.publish(odom);

      if (traj_file_.is_open() && state_machine_.state() == State::kTracking) {
        const Eigen::Matrix3d R = map_base.orientation.normalized().toRotationMatrix();
        const double roll = std::atan2(R(2, 1), R(2, 2)) * 180.0 / M_PI;
        const double pitch = std::asin(-R(2, 0)) * 180.0 / M_PI;
        const double yaw = std::atan2(R(1, 0), R(0, 0)) * 180.0 / M_PI;
        traj_file_ << std::fixed << std::setprecision(6) << stamp.toSec() << "," << stamp.sec
                   << "," << stamp.nsec << "," << map_base.position.x() << ","
                   << map_base.position.y() << "," << map_base.position.z() << "," << roll << ","
                   << pitch << "," << yaw << "\n";
      }

      // 下游规划订阅的位姿
      deeplumin_msgs::LocPoint loc_point;
      loc_point.header.stamp = stamp;
      loc_point.header.frame_id = config_.map_frame;
      loc_point.x = map_base.position.x();
      loc_point.y = map_base.position.y();
      const Eigen::Matrix3d R = map_base.orientation.normalized().toRotationMatrix();
      loc_point.yaw = std::atan2(R(1, 0), R(0, 0)) * 180.0 / M_PI;
      loc_point.pitch = std::asin(-R(2, 0)) * 180.0 / M_PI;
      loc_point.vec = static_cast<float>(latest_twist_.twist.linear.x);
      loc_point_pub_.publish(loc_point);

      if (config_.publish_path) {
        geometry_msgs::PoseStamped path_pose;
        path_pose.header = odom.header;
        path_pose.pose = odom.pose.pose;
        path_.poses.push_back(path_pose);
        while (static_cast<int>(path_.poses.size()) > config_.max_path_size) {
          path_.poses.erase(path_.poses.begin());
        }
        path_.header.stamp = stamp;
        path_pub_.publish(path_);
      }
    }

    publishMapToOdom();
    publishTf(stamp);
    publishStatus(now);
    (void)now;
  }

  void publishMapToOdom() {
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = config_.map_frame;
    odom.child_frame_id = config_.odom_frame;
    fillPose(fusion_.mapOdom(), &odom.pose.pose);
    map_to_odom_pub_.publish(odom);
  }

  void publishTf(const ros::Time& stamp) {
    if (!config_.publish_tf) {
      return;
    }
    std::vector<geometry_msgs::TransformStamped> transforms;
    if (config_.publish_map_to_odom_tf) {
      transforms.push_back(toTf(fusion_.mapOdom(), config_.map_frame, config_.odom_frame, stamp));
    }
    if (config_.publish_odom_to_base_tf) {
      transforms.push_back(
          toTf(fusion_.odomBase(), config_.odom_frame, config_.base_frame, stamp));
    }
    Pose3d map_base_tf = fusion_.fusedPose();
    if (config_.base_yaw_offset != 0.0) {
      const Eigen::Quaterniond q_off(
          Eigen::AngleAxisd(config_.base_yaw_offset, Eigen::Vector3d::UnitZ()));
      map_base_tf.orientation = (q_off * map_base_tf.orientation.normalized()).normalized();
    }
    transforms.push_back(toTf(map_base_tf, config_.map_frame, config_.base_frame, stamp));
    tf_broadcaster_.sendTransform(transforms);
  }

  void publishIdentityMapOdom() {
    if (config_.publish_tf && config_.publish_map_to_odom_tf) {
      tf_broadcaster_.sendTransform(
          toTf(Pose3d(), config_.map_frame, config_.odom_frame, ros::Time::now()));
    }
  }

  void publishCurrentScanInMap() {
    if (!config_.publish_current_scan_in_map || !latest_cloud_ || !fusion_.hasSlam()) {
      return;
    }
    pcl::PointCloud<pcl::PointXYZI> in;
    pcl::fromROSMsg(*latest_cloud_, in);
    if (in.empty()) {
      return;
    }
    pcl::PointCloud<pcl::PointXYZI> out;
    pcl::transformPointCloud(in, out, toMatrix4f(fusion_.mapOdom()));
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(out, msg);
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = config_.map_frame;
    current_scan_map_pub_.publish(msg);
  }

  void publishStatus(double now) {
    StatusInput in;
    in.state = state_machine_.state();
    in.have_slam = fusion_.hasSlam();
    in.have_map_odom = fusion_.hasMapOdom();
    in.map_id = latest_map_id_;
    in.slam_age = ageOf(last_slam_stamp_, now);
    in.gm_age = ageOf(last_gm_stamp_, now);
    in.reloc_age = ageOf(last_reloc_stamp_, now);
    in.manual_age = ageOf(last_manual_stamp_, now);
    in.gm_failures = state_machine_.getGmConsecutiveFailures();
    in.gm_degraded = state_machine_.getGmDegradedCount();
    in.reloc_failures = state_machine_.getRelocConsecutiveFailures();
    in.live_failure_reason = failure_reason_.empty() ? failure_detector_.failureReason(now)
                                                     : failure_reason_;
    in.reloc_running = state_machine_.relocalizationRunning() ||
                       state_machine_.state() == State::kTrackingLost;
    in.allow_degraded_output = config_.allow_degraded_output;
    status_pub_.publish(makeFusionStatus(in, ros::Time::now(), config_.map_frame));
  }

  void publishGmAnchor(const Pose3d& map_base, double now) {
    geometry_msgs::PoseWithCovarianceStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = config_.map_frame;
    fillPose(map_base, &msg.pose.pose);
    gm_anchor_pub_.publish(msg);
    (void)now;
    SYS_LOG(syslog::LogSeverity::INFO, true) << "published gm anchor";
  }

  // ===================================================================
  // 辅助
  // ===================================================================

  static double ageOf(double stamp, double now) {
    return stamp >= 0.0 ? (now - stamp) : -1.0;
  }

  /// 人工初值 z 补全：取位姿附近地图点云高度范围中值
  bool computeInitPoseZ(const Vec3d& position, double* z_out) {
    if (!config_.initial_pose_z_completion || config_.initial_pose_map_pcd_path.empty()) {
      return false;
    }
    if (!init_z_map_) {
      init_z_map_.reset(new pcl::PointCloud<pcl::PointXYZI>());
      if (pcl::io::loadPCDFile<pcl::PointXYZI>(config_.initial_pose_map_pcd_path, *init_z_map_) !=
              0 ||
          init_z_map_->empty()) {
        SYS_LOG(syslog::LogSeverity::ERROR, true)
            << "failed to load map for z completion: " << config_.initial_pose_map_pcd_path;
        init_z_map_.reset();
        return false;
      }
      std::vector<int> valid_indices;
      pcl::removeNaNFromPointCloud(*init_z_map_, *init_z_map_, valid_indices);
      SYS_LOG(syslog::LogSeverity::INFO, true)
          << "loaded map for initial pose z completion: " << config_.initial_pose_map_pcd_path
          << " (" << init_z_map_->size() << " pts)";
    }
    if (init_z_map_->empty()) {
      return false;
    }

    const double r2 = config_.initial_pose_z_search_radius * config_.initial_pose_z_search_radius;
    bool found = false;
    double z_min = std::numeric_limits<double>::max();
    double z_max = std::numeric_limits<double>::lowest();
    for (const auto& p : init_z_map_->points) {
      const double dx = p.x - position.x();
      const double dy = p.y - position.y();
      if (dx * dx + dy * dy <= r2) {
        found = true;
        z_min = std::min(z_min, static_cast<double>(p.z));
        z_max = std::max(z_max, static_cast<double>(p.z));
      }
    }
    if (!found) {
      return false;
    }
    *z_out = 0.5 * (z_min + z_max);
    return true;
  }

  // ===================================================================
  // 成员
  // ===================================================================
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  LocalizationV3Config config_;

  // ---- core 模块 ----
  FusionCore fusion_;
  LocalizationStateMachine state_machine_;
  FailureDetector failure_detector_;
  std::unique_ptr<IRelocalizationClient> relocalization_;

  // ---- ROS IO ----
  ros::Subscriber slam_sub_;
  ros::Subscriber gm_sub_;
  ros::Subscriber initial_pose_sub_;
  ros::Subscriber cloud_sub_;
  ros::Publisher fused_odom_pub_;
  ros::Publisher map_to_odom_pub_;
  ros::Publisher path_pub_;
  ros::Publisher status_pub_;
  ros::Publisher gm_anchor_pub_;
  ros::Publisher current_scan_map_pub_;
  ros::Publisher loc_point_pub_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  ros::Timer timer_;

  // ---- 运行时数据 ----
  sensor_msgs::PointCloud2::ConstPtr latest_cloud_;
  geometry_msgs::TwistWithCovariance latest_twist_;
  nav_msgs::Path path_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr init_z_map_;
  std::string latest_map_id_;
  std::string failure_reason_;  ///< 失效/丢失原因（进入 TRACKING 后清空）
  bool lost_synced_ = false;

  // ---- 数据新鲜度时间戳 ----
  double last_slam_stamp_ = -1.0;
  double last_gm_stamp_ = -1.0;
  double last_reloc_stamp_ = -1.0;
  double last_manual_stamp_ = -1.0;

  // ---- 人工验证 ----
  double manual_verify_deadline_ = -1.0;
  int verify_fail_count_ = 0;  ///< 验证期连续未通过计数（独立于跟踪失败计数）

  // ---- 初始化仲裁缓存 ----
  double relocalizing_since_ = -1.0;
  bool reloc_result_pending_ = false;
  Pose3d reloc_result_map_base_;
  double reloc_result_inlier_ratio_ = 0.0;
  double reloc_result_inlier_rmse_ = std::numeric_limits<double>::infinity();
  std::string reloc_result_map_id_;
  std::string reloc_result_method_;
  double reloc_result_stamp_ = -1.0;
  bool have_gm_candidate_ = false;
  double gm_candidate_inlier_ratio_ = 0.0;
  double gm_candidate_inlier_rmse_ = std::numeric_limits<double>::infinity();
  Pose3d gm_candidate_map_odom_;
  std::string gm_candidate_map_id_;

  // ---- 健康监测 / 轨迹 ----
  std::shared_ptr<deeplumin_health_checker::HealthChecker> health_checker_;
  std::ofstream traj_file_;
};

}  // namespace localization_v3

int main(int argc, char** argv) {
  ros::init(argc, argv, "localization_v3");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::string log_dir;
  pnh.param<std::string>("log_dir", log_dir,
                         "/home/hhy/2004ros/DeepLumin_zq/src/localization/loclog");
  SYS_LOG_INIT_FULL(log_dir, syslog::LogSeverity::INFO, 50, "localization_v3");

  localization_v3::LocalizationV3Node node(nh, pnh);
  ros::spin();
  return 0;
}
