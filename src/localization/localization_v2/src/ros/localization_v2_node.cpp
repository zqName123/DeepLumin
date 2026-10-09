/**
 * @file localization_v2_node.cpp
 * @brief localization_v2 定位融合节点
 *
 * 数据流：
 *   slam_odometry_v2   /localization/slam_odom            odom->base_link（高频）
 *   global_matcher     /localization/global_matcher/result map->odom（低频修正）
 *   relocalization     /localization/relocalization/result map->base_link（启动初值）
 *   /initialpose                                          人工初值（map 2D/3D）
 *
 * 融合公式（与 ysw_style_fusion 一致）：
 *   map_odom = map_base ⊕ (odom_base)⁻¹
 *   map_base = map_odom ⊕ odom_base
 *
 * 启动初始化状态机：
 *   未初始化时（可选）自动请求 relocalization 获取初始位姿；
 *   重定位成功 / 人工初值 / global_matcher 自身成功 任一发生时
 *   完成初始化并停止 relocalization。
 */

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <fstream>
#include <iomanip>
#include <cmath>

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

#include "localization_v2/core/fusion_core.hpp"
#include "localization_v2/core/initialization.hpp"
#include "localization_v2/ros/localization_v2_config.hpp"
#include "localization_v2/ros/relocalization_ros_adapter.hpp"

// 节点健康检测
#include <deeplumin_health_checker/health_checker/health_checker.h>
// 日志
#include <syslog/logger.h>

namespace localization_v2 {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kRelocResultMaxAge = 3.0;  ///< 初始化仲裁中重定位结果的时效窗（s）

/// 每行号独立的节流器（替代 ROS_INFO_THROTTLE）
template<int Line>
struct ThrottleByLine {
  static bool shouldLog(double rate) {
    static std::chrono::steady_clock::time_point last_log_time;
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration<double>(now - last_log_time).count();
    if (elapsed >= rate) {
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

class LocalizationV2Node {
 public:
  ~LocalizationV2Node() {
    if (traj_file_.is_open()) {
      traj_file_.close();
    }
  }


  LocalizationV2Node(ros::NodeHandle& nh, ros::NodeHandle& pnh)
      : nh_(nh), pnh_(pnh), config_(loadLocalizationV2Config(pnh_)) {
      
    //---- 健康监测------
    health_checker_ = std::make_shared<deeplumin_health_checker::HealthChecker>(nh_, pnh_);
    health_checker_->ENABLE(); // 启动后台发布线程
    health_checker_->NODE_ACTIVATE(); // 节点激活
    std::cout << "LocalizationV2Node: HealthChecker enabled and activated." << std::endl;

    std::string traj_path;
    pnh_.param<std::string>("traj_path", traj_path,"");
    if (!traj_path.empty()) {
      traj_file_.open(traj_path);
      if (traj_file_.is_open()) {
        traj_file_ << "timestamp, x, y, z, roll, pitch, yaw\n";
        SYS_LOG(syslog::LogSeverity::INFO, true) 
          << "trajectory saving enabled -> " << traj_path;
      } else {
        SYS_LOG(syslog::LogSeverity::WARNING, true) 
          << "failed to open trajectory file: " << traj_path;
      }
    }

    // ---- 核心与状态机配置 ----
    JumpGuardConfig jump_guard;
    jump_guard.enabled = config_.enable_jump_guard;
    jump_guard.max_translation = config_.max_jump_translation;
    jump_guard.max_yaw_deg = config_.max_jump_yaw_deg;
    fusion_.setJumpGuard(jump_guard);

    InitConfig init;
    init.require_initial_pose = config_.require_initial_pose;
    init.use_relocalization_init = config_.use_relocalization_init;
    init.use_manual_initialpose = config_.use_manual_initialpose;
    init.use_global_matcher_init = config_.use_global_matcher_init;

    RelocalizationPolicy policy;
    policy.enabled = config_.relocalization_enabled;
    policy.auto_start = config_.relocalization_auto_start;
    policy.stop_on_success = config_.relocalization_stop_on_success;
    policy.stop_on_manual_pose = config_.relocalization_stop_on_manual_pose;
    policy.stop_on_global_match = config_.relocalization_stop_on_global_match;

    TrackingFailureConfig tracking_config;
    tracking_config.gm_failure_threshold = config_.gm_failure_threshold;
    tracking_config.reloc_failure_threshold = config_.reloc_failure_threshold;
    tracking_config.reloc_timeout_sec = config_.reloc_timeout_sec;
    tracking_config.gm_recovery_threshold = config_.gm_recovery_threshold;
    tracking_config.reloc_recovery_threshold = config_.reloc_recovery_threshold;
    tracking_config.tracking_min_dwell_sec = config_.tracking_min_dwell_sec;
    tracking_config.tracking_lost_min_dwell_sec = config_.tracking_lost_min_dwell_sec;
    tracking_config.gm_degraded_threshold = config_.gm_degraded_threshold;

    state_machine_.configure(init, policy, tracking_config);

    


    // ---- 重定位接口（默认 ROS 适配层，可替换其他实现） ----
    relocalization_ = std::make_unique<RelocalizationRosAdapter>(
        nh_, config_.relocalization_result_topic, config_.relocalization_command_topic);
    relocalization_->setResultCallback(
        [this](const RelocalizationSignal& signal) { onRelocalizationResult(signal); });

    // ---- 订阅 ----
    slam_sub_ = nh_.subscribe(config_.slam_odom_topic, 100, &LocalizationV2Node::onSlamOdom, this);
    gm_sub_ = nh_.subscribe(config_.global_matcher_result_topic, 1,
                            &LocalizationV2Node::onGlobalMatch, this);
    initial_pose_sub_ = nh_.subscribe(config_.initial_pose_topic, 5,
                                      &LocalizationV2Node::onInitialPose, this);
    if (config_.publish_current_scan_in_map) {
      cloud_sub_ = nh_.subscribe(config_.source_cloud_topic, 5,
                                 &LocalizationV2Node::onSourceCloud, this);
    }

    // ---- 发布 ----
    fused_odom_pub_ = nh_.advertise<nav_msgs::Odometry>(config_.fused_odom_topic, 50);
    map_to_odom_pub_ = nh_.advertise<nav_msgs::Odometry>(config_.map_to_odom_topic, 10);
    path_pub_ = nh_.advertise<nav_msgs::Path>(config_.fused_path_topic, 5);
    status_pub_ = nh_.advertise<deeplumin_msgs::FusionStatus>(config_.status_topic, 10);
    gm_anchor_pub_ =
        nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(config_.gm_anchor_topic, 1, true);
    if (config_.publish_current_scan_in_map) {
      current_scan_map_pub_ =
          nh_.advertise<sensor_msgs::PointCloud2>(config_.current_scan_in_map_topic, 2);
    }

    loc_point_pub_ = nh_.advertise<deeplumin_msgs::LocPoint>(config_.loc_point_topic, 10);

    path_.header.frame_id = config_.map_frame;

    // ---- 定时器 ----
    timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(1.0, config_.output_rate_hz)),
                             &LocalizationV2Node::onTimer, this);

    // ROS_INFO("[localization_v2] started slam=%s gm=%s reloc_result=%s reloc_cmd=%s fused=%s",
    //          config_.slam_odom_topic.c_str(), config_.global_matcher_result_topic.c_str(),
    //          config_.relocalization_result_topic.c_str(),
    //          config_.relocalization_command_topic.c_str(), config_.fused_odom_topic.c_str());
    SYS_LOG(syslog::LogSeverity::INFO, true)
               << "slam=" << config_.slam_odom_topic << " "
               << "GM=" << config_.global_matcher_result_topic << " "
               << "reloc_result=" << config_.relocalization_result_topic << " "
               << "reloc_cmd=" << config_.relocalization_command_topic << " "
               << "fused=" << config_.fused_odom_topic;

  }

 private:
  // ===================================================================
  // 回调
  // ===================================================================

  void onSlamOdom(const nav_msgs::Odometry::ConstPtr& msg) {
    fusion_.setOdomBase(poseFromRos(msg->pose.pose));
    latest_twist_ = msg->twist;

    // slam 到达：若处于未初始化状态，触发（可选）自动启动重定位
    state_machine_.onSlamOdom();
    if (state_machine_.state() == LocalizationStateMachine::State::kRelocalizing &&
        relocalizing_since_.isZero()) {
      relocalizing_since_ = ros::Time::now();
    }
    dispatchRelocalizationCommand();

    publishCurrentScanInMap();
    publishOutput();
  }

  void onGlobalMatch(const deeplumin_msgs::GlobalMatchResult::ConstPtr& msg) {
    GlobalMatchUpdate update;
    update.success = msg->success;
    update.has_pose = true;
    update.pose = poseFromRos(msg->pose.pose);
    update.pose_is_map_base = (config_.gm_result_pose_frame == "base" ||
                               config_.gm_result_pose_frame == "base_link");
    update.map_id = msg->map_id;
    update.fitness_score = msg->fitness_score;
    update.inlier_ratio = msg->inlier_ratio;

    // [DEBUG] 打印 GM 原始结果
    // ROS_INFO("[LOC_DEBUG] GM raw result: pose(%.3f, %.3f, %.3f) pose_is_map_base=%d "
    //          "success=%d inlier=%.3f",
    //          update.pose.position.x(), update.pose.position.y(), update.pose.position.z(),
    //          update.pose_is_map_base, update.success, update.inlier_ratio);

    if (!update.success) {
      // 跟踪失败检测：计数器由状态机统一管理
      if (state_machine_.state() == LocalizationStateMachine::State::kTracking) {
        state_machine_.onTrackingFailure();
        // ROS_WARN_THROTTLE(1.0, "[localization_v2] GM consecutive failures=%d/%d",
        //                   state_machine_.getGmConsecutiveFailures(),
        //                   config_.gm_failure_threshold);
        SYS_LOG(syslog::LogSeverity::WARNING, true) 
                << "GM consecutive failures=" << state_machine_.getGmConsecutiveFailures() << "/" << config_.gm_failure_threshold;

        if (state_machine_.state() == LocalizationStateMachine::State::kTrackingLost) {
          dispatchRelocalizationCommand();
          tracking_lost_since_ = ros::Time::now();
          // ROS_WARN("[localization_v2] tracking lost, starting relocalization recovery");
          SYS_LOG(syslog::LogSeverity::WARNING, true) 
                << "tracking lost, starting relocalization recovery";
        }
      }
      return;
    }

    // 缓存 GM 候选：初始化阶段仅缓存供仲裁，TRACKING 阶段才应用
    have_gm_candidate_ = true;
    gm_candidate_inlier_ratio_ = msg->inlier_ratio;
    gm_candidate_inlier_rmse_ = msg->inlier_rmse;
    gm_candidate_map_id_ = msg->map_id;

    // --- GM 质量三级分级 ---
    // 好（good）：inlier_ratio >= good_min 且 inlier_rmse <= max_rmse → level 0
    // 退化（degraded）：inlier_ratio >= degraded_min 且 inlier_rmse <= max_rmse → level 1
    // 差（bad）：inlier_ratio < degraded_min 或 inlier_rmse > max_rmse → 失败计数
    bool gm_quality_good = false;
    bool gm_quality_degraded = false;
    if (msg->inlier_rmse > config_.gm_max_inlier_rmse) {
      // RMSE 超标 → 差（无论 inlier_ratio 多少）
      // ROS_WARN_THROTTLE(2.0, "[localization_v2] GM bad: inlier_rmse=%.3f above limit=%.3f",
      //                   msg->inlier_rmse, config_.gm_max_inlier_rmse);
      SYS_LOG(syslog::LogSeverity::WARNING, true) 
                << "GM bad: inlier_rmse=" << msg->inlier_rmse << " above limit=" << config_.gm_max_inlier_rmse;
    } else if (msg->inlier_ratio >= config_.gm_good_min_inlier_ratio) {
      // 好
      gm_quality_good = true;
    } else if (msg->inlier_ratio >= config_.gm_degraded_min_inlier_ratio) {
      // 退化
      gm_quality_degraded = true;
      // ROS_WARN_THROTTLE(2.0, "[localization_v2] GM degraded: inlier_ratio=%.3f "
      //                   "(good_thresh=%.3f degraded_thresh=%.3f)",
      //                   msg->inlier_ratio,
      //                   config_.gm_good_min_inlier_ratio, config_.gm_degraded_min_inlier_ratio);
      SYS_LOG(syslog::LogSeverity::WARNING, true) 
                << "GM degraded: inlier_ratio=" << msg->inlier_ratio << " (good_thresh=" << config_.gm_good_min_inlier_ratio << ", degraded_thresh=" << config_.gm_degraded_min_inlier_ratio << ")";
    } else {
      // inlier_ratio 太低 → 差
      // ROS_WARN_THROTTLE(2.0, "[localization_v2] GM bad: inlier_ratio=%.3f below degraded_thresh=%.3f",
      //                   msg->inlier_ratio, config_.gm_degraded_min_inlier_ratio);
      SYS_LOG(syslog::LogSeverity::WARNING, true) 
                << "GM bad: inlier_ratio=" << msg->inlier_ratio << " below degraded_thresh=" << config_.gm_degraded_min_inlier_ratio;
    }

    // TRACKING 状态下：三级质量判断
    if (state_machine_.state() == LocalizationStateMachine::State::kTracking) {
      if (gm_quality_good) {
        // 好：重置所有计数器
        state_machine_.resetGmCounter();
        state_machine_.resetDegradedCounter();
      } else if (gm_quality_degraded) {
        // 退化：增加退化计数，仍应用位姿（level 会标记为 WARNING）
        state_machine_.onTrackingDegraded();
        if (state_machine_.state() == LocalizationStateMachine::State::kTrackingLost) {
          dispatchRelocalizationCommand();
          tracking_lost_since_ = ros::Time::now();
          // ROS_WARN("[localization_v2] tracking lost due to prolonged degraded GM quality "
          //          "(degraded=%d, threshold=%d)",
          //          state_machine_.getGmDegradedCount(), config_.gm_degraded_threshold);
          SYS_LOG(syslog::LogSeverity::WARNING, true) 
                << "tracking lost due to prolonged degraded GM quality (degraded=" << state_machine_.getGmDegradedCount() << ", threshold=" << config_.gm_degraded_threshold << ")";
        }
      } else {
        // 差：增加失败计数，重置退化计数（打断退化连续性）
        state_machine_.onTrackingFailure();
        state_machine_.resetDegradedCounter();
        if (state_machine_.state() == LocalizationStateMachine::State::kTrackingLost) {
          dispatchRelocalizationCommand();
          tracking_lost_since_ = ros::Time::now();
          // ROS_WARN("[localization_v2] tracking lost due to poor GM quality "
          //          "(failures=%d/%d)",
          //          state_machine_.getGmConsecutiveFailures(), config_.gm_failure_threshold);
          SYS_LOG(syslog::LogSeverity::WARNING, true) 
                << "tracking lost due to poor GM quality (failures=" << state_machine_.getGmConsecutiveFailures() << ", threshold=" << config_.gm_failure_threshold << ")";
        }
        return;  // 差结果不应用位姿
      }
    }
    if (update.pose_is_map_base) {
      if (!fusion_.hasSlam()) {
        return;
      }
      gm_candidate_map_odom_ = compose(update.pose, inverse(fusion_.odomBase()));
    } else {
      gm_candidate_map_odom_ = update.pose;
    }
    // [DEBUG] 打印 gm_candidate_map_odom_ 计算结果
    // ROS_INFO("[LOC_DEBUG] gm_candidate_map_odom computed: (%.3f, %.3f, %.3f) "
    //          "odom_base(%.3f, %.3f, %.3f) pose_is_map_base=%d",
    //          gm_candidate_map_odom_.position.x(), gm_candidate_map_odom_.position.y(), gm_candidate_map_odom_.position.z(),
    //          fusion_.odomBase().position.x(), fusion_.odomBase().position.y(), fusion_.odomBase().position.z(),
    //          update.pose_is_map_base);

    // 未进入 TRACKING：不应用 jump guard（门限仅 TRACKING 生效）
    if (state_machine_.state() != LocalizationStateMachine::State::kTracking) {
      // LOCALIZATION_FAILURE 状态：仅 GM 质量好时累积恢复（确认窗口）
      if (state_machine_.state() == LocalizationStateMachine::State::kLocalizationFailure) {
        if (!gm_quality_good) {
          state_machine_.onRecoveryAttemptFailed();
          // ROS_WARN_THROTTLE(2.0,
          //                   "[localization_v2] skip GM recovery from localization failure: quality too low "
          //                   "inlier=%.3f rmse=%.3f",
          //                   msg->inlier_ratio, msg->inlier_rmse);
          SYS_LOG(syslog::LogSeverity::WARNING, true) 
                << "skip GM recovery from localization failure: quality too low inlier_ratio=" << msg->inlier_ratio << " rmse=" << msg->inlier_rmse;
          return;
        }
        if (!fusion_.setMapOdom(gm_candidate_map_odom_)) {
          state_machine_.onRecoveryAttemptFailed();
          return;
        }
        state_machine_.onGlobalMatchSuccess();
        if (state_machine_.state() == LocalizationStateMachine::State::kTracking) {
          dispatchRelocalizationCommand();
          latest_map_id_ = msg->map_id;
          publishMapToOdom();
          // ROS_INFO("[localization_v2] recovered from localization failure via global matcher "
          //          "map_id=%s inlier=%.3f rmse=%.3f",
          //          msg->map_id.c_str(), msg->inlier_ratio, msg->inlier_rmse);
          SYS_LOG(syslog::LogSeverity::INFO, true) 
                << "recovered from localization failure via global matcher map_id=" << msg->map_id << " inlier_ratio=" << msg->inlier_ratio << " rmse=" << msg->inlier_rmse;
        } else {
          // ROS_INFO_THROTTLE(1.0,
          //                   "[localization_v2] GM recovery accumulating for localization failure: "
          //                   "inlier=%.3f rmse=%.3f (waiting for confirmation window)",
          //                   msg->inlier_ratio, msg->inlier_rmse);
          SYS_LOG(syslog::LogSeverity::INFO, true) 
                << "GM recovery accumulating for localization failure inlier_ratio=" << msg->inlier_ratio << " rmse=" << msg->inlier_rmse << " (waiting for confirmation window)";
        }
        return;
      }

      // TRACKING_LOST 状态：GM 成功时累积恢复（确认窗口）
      if (state_machine_.state() == LocalizationStateMachine::State::kTrackingLost) {
        if (!gm_quality_good) {
          state_machine_.onRecoveryAttemptFailed();
          // ROS_DEBUG("[localization_v2] GM quality bad during recovery attempt, reset counter");
          SYS_LOG(syslog::LogSeverity::INFO, true) 
                << "GM quality bad during recovery attempt, reset counter";
          return;
        }
        if (!fusion_.setMapOdom(gm_candidate_map_odom_)) {
          state_machine_.onRecoveryAttemptFailed();
          return;
        }
        state_machine_.onGlobalMatchSuccess();
        if (state_machine_.state() == LocalizationStateMachine::State::kTracking) {
          dispatchRelocalizationCommand();
          latest_map_id_ = msg->map_id;
          publishMapToOdom();
          // ROS_INFO("[localization_v2] recovered from tracking lost via global matcher "
          //          "map_id=%s inlier=%.3f rmse=%.3f",
          //          msg->map_id.c_str(), msg->inlier_ratio, msg->inlier_rmse);
          SYS_LOG(syslog::LogSeverity::INFO, true) 
                << "recovered from tracking lost via global matcher map_id=" << msg->map_id << " inlier_ratio=" << msg->inlier_ratio << " rmse=" << msg->inlier_rmse;
        } else {
          // ROS_INFO_THROTTLE(1.0,
          //                   "[localization_v2] GM recovery accumulating for tracking lost: "
          //                   "inlier=%.3f rmse=%.3f (waiting for confirmation window)",
          //                   msg->inlier_ratio, msg->inlier_rmse);
          SYS_LOG(syslog::LogSeverity::INFO, true) 
                << "GM recovery accumulating for tracking lost inlier_ratio=" << msg->inlier_ratio << " rmse=" << msg->inlier_rmse << " (waiting for confirmation window)";
        }
        return;
      }

      if (config_.use_global_matcher_init && !config_.use_relocalization_init) {
        // 仅 GM 初始化：GM 成功即完成初始化
        if (!fusion_.setMapOdom(gm_candidate_map_odom_)) {
          return;
        }
        state_machine_.onGlobalMatchSuccess();
        dispatchRelocalizationCommand();
        latest_map_id_ = msg->map_id;
        publishMapToOdom();
        // ROS_INFO("[localization_v2] initialized from global matcher map_id=%s inlier=%.3f rmse=%.3f",
        //          msg->map_id.c_str(), msg->inlier_ratio, msg->inlier_rmse);
        SYS_LOG(syslog::LogSeverity::INFO, true) 
                << "initialized from global matcher map_id=" << msg->map_id << " inlier_ratio=" << msg->inlier_ratio << " rmse=" << msg->inlier_rmse;

      } else if (config_.use_global_matcher_init && config_.use_relocalization_init) {
        // 双源初始化：缓存候选，等待重定位结果仲裁
        tryCompleteInitialization();
      } else {
        // ROS_INFO_THROTTLE(1.0,
        //                   "[localization_v2] cached global matcher candidate during initializing "
        //                   "inlier=%.3f rmse=%.3f",
        //                   msg->inlier_ratio, msg->inlier_rmse);
        SYS_LOG(syslog::LogSeverity::INFO, true) 
                << "cached global matcher candidate during initializing inlier_ratio=" << msg->inlier_ratio << " rmse=" << msg->inlier_rmse;
      }
      return;
    }

    // TRACKING：jump guard / bypass 校验后应用
    // [DEBUG] 打印 jump guard 比较的值（fused_pose 变化量）
    {
      const Pose3d old_fused = compose(fusion_.mapOdom(), fusion_.odomBase());
      const Pose3d new_fused = compose(gm_candidate_map_odom_, fusion_.odomBase());
      // ROS_INFO("[LOC_DEBUG] jump guard check: "
      //          "old_fused(%.3f, %.3f) new_fused(%.3f, %.3f) fused_diff=%.3fm "
      //          "map_odom(%.3f, %.3f) gm_candidate(%.3f, %.3f)",
      //          old_fused.position.x(), old_fused.position.y(),
      //          new_fused.position.x(), new_fused.position.y(),
      //          translationDiff(old_fused, new_fused),
      //          fusion_.mapOdom().position.x(), fusion_.mapOdom().position.y(),
      //          gm_candidate_map_odom_.position.x(), gm_candidate_map_odom_.position.y());
    }
    const bool accepted = fusion_.updateFromGlobalMatch(update);
    if (!accepted) {
      // 计算 fused_pose 变化量（与 jump guard 内部逻辑一致）
      const Pose3d old_fused = compose(fusion_.mapOdom(), fusion_.odomBase());
      const Pose3d new_fused = compose(gm_candidate_map_odom_, fusion_.odomBase());
      const double jump_dx = translationDiff(old_fused, new_fused);
      const double jump_dyaw = yawDiffDeg(old_fused, new_fused);
      const bool dx_exceed = jump_dx > config_.max_jump_translation;
      const bool yaw_exceed = std::abs(jump_dyaw) > config_.max_jump_yaw_deg;
      // ROS_WARN_THROTTLE(
      //     1.0,
      //     "[localization_v2] reject global matcher result (jump guard): trans=%.2fm "
      //     "(limit %.2fm) %s, yaw=%.1fdeg (limit %.1fdeg) %s",
      //     jump_dx, config_.max_jump_translation, dx_exceed ? "EXCEED" : "ok", jump_dyaw,
      //     config_.max_jump_yaw_deg, yaw_exceed ? "EXCEED" : "ok");
      SYS_LOG(syslog::LogSeverity::WARNING, true) 
                << "reject global matcher result (jump guard): trans=" << jump_dx << " (limit " << config_.max_jump_translation << ") " << (dx_exceed ? "EXCEED" : "ok") << ", yaw=" << jump_dyaw << " (limit " << config_.max_jump_yaw_deg << ") " << (yaw_exceed ? "EXCEED" : "ok");
      return;
    }
    latest_map_id_ = msg->map_id;
    publishMapToOdom();
    // ROS_INFO("[localization_v2] updated map->odom from global matcher map_id=%s fitness=%.4f inlier=%.3f rmse=%.3f",
    //          msg->map_id.c_str(), msg->fitness_score, msg->inlier_ratio, msg->inlier_rmse);
    SYS_LOG(syslog::LogSeverity::INFO, false) 
                << "updated map->odom from global matcher map_id=" << msg->map_id << " fitness=" << msg->fitness_score << " inlier_ratio=" << msg->inlier_ratio << " rmse=" << msg->inlier_rmse;
  }

  void onInitialPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    if (!config_.use_manual_initialpose) {
      return;
    }

    Pose3d map_base = poseFromRos(msg->pose.pose);
    // 与 ysw 一致：兼容 RViz 固定坐标系被设为 odom/base_link 的情况
    const std::string pose_frame = msg->header.frame_id;
    if (!pose_frame.empty() && pose_frame != config_.map_frame) {
      if (pose_frame == config_.odom_frame) {
        map_base = compose(fusion_.mapOdom(), map_base);
      } else if (pose_frame == config_.base_frame) {
        map_base = compose(fusion_.mapOdom(), fusion_.odomBase());
      } else {
        // ROS_WARN_THROTTLE(2.0, "[localization_v2] unknown initial pose frame '%s', assume %s",
        //                   pose_frame.c_str(), config_.map_frame.c_str());
        SYS_LOG(syslog::LogSeverity::WARNING, true) 
                << "unknown initial pose frame '" << pose_frame << "', assume " << config_.map_frame;
      }
    }

    // z 补全（取人工位姿附近地图点云高度范围中值）
    double init_z = 0.0;
    if (computeInitPoseZ(map_base.position, &init_z)) {
      map_base.position.z() = init_z;
    }

    if (!fusion_.updateMapOdomFromMapBase(map_base, /*require_slam=*/false)) {
      // ROS_WARN("[localization_v2] invalid manual initial pose, ignored");
      SYS_LOG(syslog::LogSeverity::WARNING, true) 
                << "invalid manual initial pose, ignored";
      return;
    }
    fusion_.requestJumpGuardBypass();
    state_machine_.onManualPose();
    dispatchRelocalizationCommand();
    publishMapToOdom();
    // ROS_INFO("[localization_v2] reset map->odom from manual initial pose frame=%s z=%.3f",
    //          pose_frame.c_str(), map_base.position.z());
    SYS_LOG(syslog::LogSeverity::INFO, false) 
                << "reset map->odom from manual initial pose frame=" << pose_frame << " z=" << map_base.position.z();
  }

  void onRelocalizationResult(const RelocalizationSignal& signal) {
    // 允许在 RELOCALIZING 和 TRACKING_LOST 状态下处理重定位结果
    const auto state = state_machine_.state();
    if (state != LocalizationStateMachine::State::kRelocalizing &&
        state != LocalizationStateMachine::State::kTrackingLost) {
      return;
    }
    if (!signal.success) {
      // TRACKING_LOST 状态下重定位失败，计数器由状态机统一管理
      if (state == LocalizationStateMachine::State::kTrackingLost) {
        state_machine_.onRecoveryFailed();
        // ROS_WARN_THROTTLE(2.0, "[localization_v2] relocalization failed, consecutive=%d/%d",
        //                   state_machine_.getRelocConsecutiveFailures(),
        //                   config_.reloc_failure_threshold);
        SYS_LOG(syslog::LogSeverity::WARNING, true) 
                << "relocalization failed, consecutive=" << state_machine_.getRelocConsecutiveFailures() << "/" << config_.reloc_failure_threshold;
        if (state_machine_.state() == LocalizationStateMachine::State::kLocalizationFailure) {
          dispatchRelocalizationCommand();
          // ROS_ERROR("[localization_v2] recovery failed, localization failure");
          SYS_LOG(syslog::LogSeverity::ERROR, true) 
                    << "recovery failed, localization failure";
        }
      }
      return;
    }
    if (config_.relocalization_require_accepted && !signal.accepted) {
      // ROS_WARN_THROTTLE(2.0, "[localization_v2] relocalization result not accepted");
      SYS_LOG(syslog::LogSeverity::WARNING, true) 
                << "relocalization result not accepted";
      return;
    }
    // 质量门限（只使用 inlier_ratio 和 inlier_rmse，不使用 fitness_score）
    if (signal.inlier_ratio < config_.relocalization_min_inlier_ratio) {
      // ROS_WARN_THROTTLE(2.0, "[localization_v2] relocalization inlier ratio %.3f below limit %.3f",
      //                   signal.inlier_ratio, config_.relocalization_min_inlier_ratio);
      SYS_LOG(syslog::LogSeverity::WARNING, true) 
                << "relocalization inlier ratio " << signal.inlier_ratio << " below limit " << config_.relocalization_min_inlier_ratio;
      return;
    }
    if (signal.inlier_rmse > config_.relocalization_max_inlier_rmse) {
      // ROS_WARN_THROTTLE(2.0, "[localization_v2] relocalization rmse %.3f above limit %.3f",
      //                   signal.inlier_rmse, config_.relocalization_max_inlier_rmse);
      SYS_LOG(syslog::LogSeverity::WARNING, true) 
                << "relocalization rmse " << signal.inlier_rmse << " above limit " << config_.relocalization_max_inlier_rmse;
      return;
    }

    // 通过门限：暂存重定位结果，交由初始化仲裁（双源）或直接完成（仅重定位源）
    reloc_result_pending_ = true;
    reloc_result_map_base_ = signal.map_base;
    reloc_result_inlier_ratio_ = signal.inlier_ratio;
    reloc_result_inlier_rmse_ = signal.inlier_rmse;
    reloc_result_map_id_ = signal.map_id;
    reloc_result_method_ = signal.method;
    reloc_result_stamp_ = ros::Time::now();

    // TRACKING_LOST 状态下重定位成功：使用状态机确认窗口
    if (state == LocalizationStateMachine::State::kTrackingLost) {
      state_machine_.onRelocalizationResult(true);
      // 即使确认窗口未满，也先应用位姿（避免等待期间位姿缺失，让GM能在正确位姿上工作）
      if (!fusion_.setMapOdom(reloc_result_map_base_)) {
        return;
      }
      if (state_machine_.state() == LocalizationStateMachine::State::kTracking) {
        // 恢复成功，发布最终结果
        dispatchRelocalizationCommand();
        publishMapToOdom();
        // ROS_INFO("[localization_v2] recovered from tracking lost via relocalization "
        //          "map_id=%s inlier=%.3f rmse=%.3f",
        //          reloc_result_map_id_.c_str(), reloc_result_inlier_ratio_,
        //          reloc_result_inlier_rmse_);
        SYS_LOG(syslog::LogSeverity::INFO, true) 
                << "recovered from tracking lost via relocalization map_id=" << reloc_result_map_id_ << " inlier=" << reloc_result_inlier_ratio_ << " rmse=" << reloc_result_inlier_rmse_;
      } else {
        // ROS_INFO_THROTTLE(1.0,
        //                   "[localization_v2] relocalization recovery accumulating: "
        //                   "inlier=%.3f rmse=%.3f (waiting for confirmation window)",
        //                   reloc_result_inlier_ratio_, reloc_result_inlier_rmse_);
        SYS_LOG(syslog::LogSeverity::INFO, false) 
                << "relocalization recovery accumulating: inlier=" << reloc_result_inlier_ratio_ << " rmse=" << reloc_result_inlier_rmse_ << " (waiting for confirmation window)";
      }
      return;
    }

    tryCompleteInitialization();
  }

  // ===================================================================
  // 初始化仲裁
  // ===================================================================

  void tryCompleteInitialization() {
    if (state_machine_.state() != LocalizationStateMachine::State::kRelocalizing) {
      return;
    }
    const bool reloc_fresh =
        reloc_result_pending_ && !relocalizing_since_.isZero() &&
        (ros::Time::now() - reloc_result_stamp_).toSec() <= kRelocResultMaxAge;
    const bool both_mode = config_.use_global_matcher_init && config_.use_relocalization_init;
    const bool deadline = !relocalizing_since_.isZero() &&
                          (ros::Time::now() - relocalizing_since_).toSec() >=
                              config_.relocalization_fallback_timeout;
    if (!both_mode) {
      if (reloc_fresh) {
        completeWithReloc();
      } else if (have_gm_candidate_ && config_.use_global_matcher_init) {
        completeWithGm();
      }
      return;
    }
    if (reloc_fresh && have_gm_candidate_) {
      arbitrateAndComplete();
      return;
    }
    if (!deadline) {
      return;
    }
    if (reloc_fresh) {
      completeWithReloc();
    } else if (have_gm_candidate_) {
      completeWithGm();
    } else {
      // ROS_WARN_THROTTLE(5.0,
      //                   "[localization_v2] no init source ready within %.1fs, keep initializing",
      //                   config_.relocalization_fallback_timeout);
      SYS_LOG(syslog::LogSeverity::WARNING, false) 
                << "no init source ready within " << config_.relocalization_fallback_timeout << "s, keep initializing";
    }
  }

  void arbitrateAndComplete() {
    // 综合评分仲裁：重定位是初始化主源，使用加权评分综合考虑内点率和 RMSE
    // 内点率权重 1.0，RMSE 权重 -0.3（RMSE 单位米，数值较小，适当加权）
    // 当重定位综合评分不低于 GM 时，优先选择重定位（重定位初始化效果更好）
    const double reloc_score = reloc_result_inlier_ratio_ - 0.3 * reloc_result_inlier_rmse_;
    const double gm_score = gm_candidate_inlier_ratio_ - 0.3 * gm_candidate_inlier_rmse_;
    const bool reloc_better = reloc_score >= gm_score;

    // ROS_INFO("[localization_v2] init arbitration: reloc(score=%.4f ratio=%.3f rmse=%.3f) vs "
    //          "gm(score=%.4f ratio=%.3f rmse=%.3f) => %s",
    //          reloc_score, reloc_result_inlier_ratio_, reloc_result_inlier_rmse_,
    //          gm_score, gm_candidate_inlier_ratio_, gm_candidate_inlier_rmse_,
    //          reloc_better ? "RELOC" : "GM");
    SYS_LOG(syslog::LogSeverity::INFO, true) 
                << "init arbitration: reloc(score=" << reloc_score << " ratio=" << reloc_result_inlier_ratio_ << " rmse=" << reloc_result_inlier_rmse_ << ") vs gm(score=" << gm_score << " ratio=" << gm_candidate_inlier_ratio_ << " rmse=" << gm_candidate_inlier_rmse_ << ") => " << (reloc_better ? "RELOC" : "GM");

    if (reloc_better) {
      completeWithReloc();
    } else {
      completeWithGm();
    }
  }

  void completeWithReloc() {
    if (!fusion_.updateMapOdomFromMapBase(reloc_result_map_base_, /*require_slam=*/false)) {
      // ROS_WARN("[localization_v2] invalid relocalization pose, ignored");
      SYS_LOG(syslog::LogSeverity::WARNING, false) 
                << "invalid relocalization pose, ignored";
      reloc_result_pending_ = false;
      return;
    }
    // 覆盖 GM 上一次结果：仅初始化阶段，向 GM 发布一次性锚点
    publishGmAnchor(reloc_result_map_base_);
    latest_map_id_ = reloc_result_map_id_;
    state_machine_.onRelocalizationResult(/*accepted=*/true);
    dispatchRelocalizationCommand();
    publishMapToOdom();
    // ROS_INFO("[localization_v2] initialized from relocalization method=%s map_id=%s "
    //          "(reloc inlier=%.3f rmse=%.3f vs gm inlier=%.3f rmse=%.3f)",
    //          reloc_result_method_.c_str(), reloc_result_map_id_.c_str(),
    //          reloc_result_inlier_ratio_, reloc_result_inlier_rmse_, gm_candidate_inlier_ratio_,
    //          gm_candidate_inlier_rmse_);
    SYS_LOG(syslog::LogSeverity::INFO, true) 
                << "initialized from relocalization method=" << reloc_result_method_ << " map_id=" << reloc_result_map_id_ << " (reloc inlier=" << reloc_result_inlier_ratio_ << " rmse=" << reloc_result_inlier_rmse_ << " vs gm inlier=" << gm_candidate_inlier_ratio_ << " rmse=" << gm_candidate_inlier_rmse_ << ")";
        
  }

  void completeWithGm() {
    if (!fusion_.setMapOdom(gm_candidate_map_odom_)) {
      // ROS_WARN("[localization_v2] invalid global matcher candidate pose, ignored");
      SYS_LOG(syslog::LogSeverity::WARNING, false) 
                << "invalid global matcher candidate pose, ignored";
      reloc_result_pending_ = false;
      return;
    }
    latest_map_id_ = gm_candidate_map_id_;
    state_machine_.onRelocalizationResult(/*accepted=*/true);
    dispatchRelocalizationCommand();
    publishMapToOdom();
    reloc_result_pending_ = false;
    // ROS_INFO("[localization_v2] initialized from global matcher candidate "
    //          "(gm inlier=%.3f rmse=%.3f vs reloc inlier=%.3f rmse=%.3f)",
    //          gm_candidate_inlier_ratio_, gm_candidate_inlier_rmse_, reloc_result_inlier_ratio_,
    //          reloc_result_inlier_rmse_);
    SYS_LOG(syslog::LogSeverity::INFO, true) 
                << "initialized from global matcher candidate (gm inlier=" << gm_candidate_inlier_ratio_ << " rmse=" << gm_candidate_inlier_rmse_ << " vs reloc inlier=" << reloc_result_inlier_ratio_ << " rmse=" << reloc_result_inlier_rmse_ << ")";
  }

  void publishGmAnchor(const Pose3d& map_base) {
    geometry_msgs::PoseWithCovarianceStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = config_.map_frame;
    fillPose(map_base, &msg.pose.pose);
    gm_anchor_pub_.publish(msg);
    // ROS_INFO("[localization_v2] published gm anchor (relocalization winner)");
    SYS_LOG(syslog::LogSeverity::INFO, true) 
                << "published gm anchor (relocalization winner)";
  }

  void onSourceCloud(const sensor_msgs::PointCloud2::ConstPtr& msg) { latest_cloud_ = msg; }

  // ===================================================================
  // 定时输出
  // ===================================================================

  void onTimer(const ros::TimerEvent&) {
    if (!fusion_.hasSlam()) {
      publishIdentityMapOdom();
      // ROS_WARN_THROTTLE(2.0, "[localization_v2] waiting for slam odom: %s",
      //                   config_.slam_odom_topic.c_str());
      if (SHOULD_LOG(10.0)) {
        SYS_LOG(syslog::LogSeverity::WARNING, false) 
                << "waiting for slam odom: " << config_.slam_odom_topic;
      }

      return;
    }

    // 恢复超时检查：超时是独立的失败判定，直接切换状态不经过计数器
    if (state_machine_.state() == LocalizationStateMachine::State::kTrackingLost) {
      const double elapsed = (ros::Time::now() - tracking_lost_since_).toSec();
      if (elapsed >= config_.reloc_timeout_sec) {
        state_machine_.forceLocalizationFailure();
        dispatchRelocalizationCommand();
        // ROS_ERROR("[localization_v2] recovery timeout (%.1fs >= %.1fs), localization failure",
        //           elapsed, config_.reloc_timeout_sec);
        SYS_LOG(syslog::LogSeverity::ERROR, true) 
                << "recovery timeout (" << elapsed << " >= " << config_.reloc_timeout_sec << "), localization failure";
      }
    }

    // 初始化仲裁：双源等待超时后取已就绪的一源，防止状态切换被阻塞
    tryCompleteInitialization();
    publishCurrentScanInMap();
    publishOutput();
  }

  void publishOutput() {
    if (!fusion_.hasSlam()) {
      return;
    }
    const bool output_ready = fusion_.isInitialized() || !config_.require_initial_pose;
    const Pose3d map_base = fusion_.fusedPose();
    const ros::Time stamp = ros::Time::now();

    if (output_ready) {
      // 融合里程计 map->base_link（保留 slam 速度）
      nav_msgs::Odometry odom;
      odom.header.stamp = stamp;
      odom.header.frame_id = config_.map_frame;
      odom.child_frame_id = config_.base_frame;
      fillPose(map_base, &odom.pose.pose);
      odom.twist = latest_twist_;
      fused_odom_pub_.publish(odom);
      // 保存轨迹

      if (traj_file_.is_open()) {
        if (state_machine_.state() == LocalizationStateMachine::State::kTracking) {
          Eigen::Matrix3d R_traj = map_base.orientation.normalized().toRotationMatrix();
          double roll  = std::atan2(R_traj(2,1), R_traj(2,2)) * 180.0 / M_PI;
          double pitch = std::asin(-R_traj(2,0)) * 180.0 / M_PI;
          double yaw   = std::atan2(R_traj(1,0), R_traj(0,0)) * 180.0 / M_PI;

          traj_file_ << std::fixed << std::setprecision(6)
                    << stamp.toSec() << ","
                    << stamp.sec << "," << stamp.nsec << ","
                    << map_base.position.x() << ","
                    << map_base.position.y() << ","
                    << map_base.position.z() << ","
                    << roll << "," << pitch << "," << yaw << "\n";
        }
      }


      // 构建loc_point 下游订阅
      deeplumin_msgs::LocPoint loc_point;
      loc_point.header.stamp = stamp;
      loc_point.header.frame_id = config_.map_frame;
      loc_point.x = map_base.position.x();
      loc_point.y = map_base.position.y();
      // 提取yaw 和 pitch
      Eigen::Matrix3d R = map_base.orientation.normalized().toRotationMatrix();
      // loc_point.yaw = std::atan2(R(1, 0), R(0, 0)); // yaw 角度范围 [-pi, pi]，单位弧度
      // loc_point.pitch = std::asin(-R(2, 0)); // pitch 角度范围 [-pi, pi]，单位弧度
      double yaw_rad = std::atan2(R(1, 0), R(0, 0)); // yaw 角度范围 [-pi, pi]，单位弧度
      if (yaw_rad > M_PI) {
        yaw_rad -= 2 * M_PI; // 角度范围 [0, 2pi]
      } else if (yaw_rad < -M_PI) {
        yaw_rad += 2 * M_PI; // 角度范围 [0, 2pi]
      }
      loc_point.yaw = yaw_rad * 180.0 / M_PI; // 转换为角度
      loc_point.pitch = std::asin(-R(2, 0)) * 180.0 / M_PI; // 转换为角度
      
      const float vx = latest_twist_.twist.linear.x;
      loc_point.vec = static_cast<float>(vx);

      loc_point_pub_.publish(loc_point);


      // 历史轨迹
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
    publishStatus(stamp, output_ready);
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
    // 适配下游，y朝前
    Pose3d map_base_tf = fusion_.fusedPose();
    if (config_.base_yaw_offset != 0.0) {
      Eigen::Quaterniond q_off(Eigen::AngleAxisd(config_.base_yaw_offset, Eigen::Vector3d::UnitZ()));
      map_base_tf.orientation = (q_off * map_base_tf.orientation.normalized()).normalized();

    }
    transforms.push_back(toTf(map_base_tf, config_.map_frame, config_.base_frame, stamp));

    if (!transforms.empty()) {
      tf_broadcaster_.sendTransform(transforms);
    }

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

  void publishStatus(const ros::Time& stamp, bool output_valid) {
    deeplumin_msgs::FusionStatus msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = config_.map_frame;
    msg.current_map_id = latest_map_id_;

    // ========== 计算数据源年龄 ==========
    const double slam_age = fusion_.hasSlam() ? 0.0 : 1e9;  // SLAM 数据实时性
    const double gm_age = have_gm_candidate_ ? 0.0 : 1e9;   // GM 数据实时性
    const double reloc_age = (state_machine_.state() == LocalizationStateMachine::State::kRelocalizing ||
                               state_machine_.state() == LocalizationStateMachine::State::kTrackingLost)
                                  ? (ros::Time::now() - relocalizing_since_).toSec()
                                  : 1e9;

    msg.last_slam_age = slam_age;
    msg.last_global_matcher_age = gm_age;
    msg.last_relocalization_age = reloc_age;
    msg.last_dr_age = 1e9;  // 未使用 DR
    msg.last_gnss_age = 1e9;  // 未使用 GNSS
    msg.last_can_age = 0.0;
    msg.last_imu_age = 0.0;
    msg.last_lidar_age = slam_age;

    // ========== 根据状态机设置 level 和 failure_reason ==========
    switch (state_machine_.state()) {
      case LocalizationStateMachine::State::kUninitialized:
        msg.level = deeplumin_msgs::FusionStatus::LEVEL_WARNING;
        msg.state = "UNINITIALIZED";
        msg.initialized = false;
        msg.output_valid = false;
        msg.failure_reason = "waiting for initial pose";
        msg.position_std = 100.0;
        msg.velocity_std = 10.0;
        msg.roll_std = 90.0;
        msg.pitch_std = 90.0;
        msg.yaw_std = 180.0;
        break;

      case LocalizationStateMachine::State::kRelocalizing:
        msg.level = deeplumin_msgs::FusionStatus::LEVEL_WARNING;
        msg.state = "RELOCALIZING";
        msg.initialized = false;
        msg.output_valid = false;
        msg.failure_reason = "relocalization in progress for initial pose";
        msg.position_std = 50.0;
        msg.velocity_std = 5.0;
        msg.roll_std = 45.0;
        msg.pitch_std = 45.0;
        msg.yaw_std = 90.0;
        break;

      case LocalizationStateMachine::State::kTracking:
        msg.state = "TRACKING";
        msg.initialized = true;
        msg.output_valid = true;
        // 动态计算 level：根据 GM 质量和退化计数器
        {
          int failures = state_machine_.getGmConsecutiveFailures();
          int degraded = state_machine_.getGmDegradedCount();
          if (failures == 0 && degraded == 0) {
            msg.level = deeplumin_msgs::FusionStatus::LEVEL_NORMAL;
            msg.failure_reason = "";
          } else {
            msg.level = deeplumin_msgs::FusionStatus::LEVEL_WARNING;
            msg.failure_reason = "GM quality degraded: consecutive_failures=" +
                                 std::to_string(failures) +
                                 ", degraded_frames=" + std::to_string(degraded);
          }
        }
        msg.position_std = 0.1;
        msg.velocity_std = 0.5;
        msg.roll_std = 0.5;
        msg.pitch_std = 0.5;
        msg.yaw_std = 1.0;
        break;

      case LocalizationStateMachine::State::kTrackingLost:
        msg.level = deeplumin_msgs::FusionStatus::LEVEL_DEGRADED;
        msg.state = "TRACKING_LOST";
        msg.initialized = true;
        msg.output_valid = config_.allow_degraded_output;
        msg.failure_reason = "tracking lost, GM consecutive failures=" +
                             std::to_string(state_machine_.getGmConsecutiveFailures()) +
                             ", relocalization attempting recovery";
        msg.position_std = 5.0;
        msg.velocity_std = 2.0;
        msg.roll_std = 5.0;
        msg.pitch_std = 5.0;
        msg.yaw_std = 10.0;
        break;

      case LocalizationStateMachine::State::kLocalizationFailure:
        msg.level = deeplumin_msgs::FusionStatus::LEVEL_FAILURE;
        msg.state = "LOCALIZATION_FAILURE";
        msg.initialized = true;
        msg.output_valid = false;
        msg.failure_reason = std::string("localization failure: recovery timeout") +
                             ", reloc failures=" + std::to_string(state_machine_.getRelocConsecutiveFailures());
        msg.position_std = 100.0;
        msg.velocity_std = 10.0;
        msg.roll_std = 90.0;
        msg.pitch_std = 90.0;
        msg.yaw_std = 180.0;
        break;
    }

    // ========== 设置 used_sources ==========
    msg.used_sources.clear();
    msg.rejected_sources.clear();
    msg.reject_reasons.clear();

    if (fusion_.hasSlam()) {
      msg.used_sources.push_back("slam_odometry_v2");
    }

    switch (state_machine_.state()) {
      case LocalizationStateMachine::State::kTracking:
        if (fusion_.hasMapOdom()) {
          msg.used_sources.push_back("global_matcher");
        }
        break;

      case LocalizationStateMachine::State::kTrackingLost:
        msg.rejected_sources.push_back("global_matcher");
        msg.reject_reasons.push_back("GM consecutive failures exceeded threshold");
        if (config_.relocalization_enabled) {
          msg.used_sources.push_back("relocalization_attempting");
        }
        break;

      case LocalizationStateMachine::State::kLocalizationFailure:
        msg.rejected_sources.push_back("global_matcher");
        msg.rejected_sources.push_back("relocalization");
        msg.reject_reasons.push_back("GM tracking lost");
        msg.reject_reasons.push_back("relocalization recovery failed");
        break;

      default:
        if (state_machine_.relocalizationRunning()) {
          msg.used_sources.push_back("relocalization");
        }
        break;
    }

    // ========== 数据源可用性 ==========
    msg.is_slam_available = fusion_.hasSlam();
    msg.is_global_matcher_available = gm_age < 10.0;  // GM 数据 10 秒内可用
    msg.is_relocalization_available = config_.relocalization_enabled;
    msg.is_dr_available = false;
    msg.is_gnss_available = false;
    msg.is_can_available = true;
    msg.is_imu_available = true;
    msg.is_lidar_available = fusion_.hasSlam();

    status_pub_.publish(msg);
  }

  // ===================================================================
  // 辅助
  // ===================================================================

  void dispatchRelocalizationCommand() {
    const RelocCommand cmd = state_machine_.takeCommand();
    if (cmd == RelocCommand::kStart) {
      relocalization_->start();
    } else if (cmd == RelocCommand::kStop) {
      relocalization_->stop();
    }
  }

  /// 人工初值 z 补全：取位姿附近地图点云高度范围中值（与 ysw_loc 一致）
  bool computeInitPoseZ(const Vec3d& position, double* z_out) {
    if (!config_.initial_pose_z_completion || config_.initial_pose_map_pcd_path.empty()) {
      return false;
    }
    if (!init_z_map_) {
      init_z_map_.reset(new pcl::PointCloud<pcl::PointXYZI>());
      if (pcl::io::loadPCDFile<pcl::PointXYZI>(config_.initial_pose_map_pcd_path, *init_z_map_) !=
              0 ||
          init_z_map_->empty()) {
        // ROS_ERROR("[localization_v2] failed to load map for z completion: %s",
        //           config_.initial_pose_map_pcd_path.c_str());
        SYS_LOG(syslog::LogSeverity::ERROR, true) 
                << "failed to load map for z completion: " << config_.initial_pose_map_pcd_path;
        init_z_map_.reset();
        return false;
      }
      std::vector<int> valid_indices;
      pcl::removeNaNFromPointCloud(*init_z_map_, *init_z_map_, valid_indices);
      // ROS_INFO("[localization_v2] loaded map for initial pose z completion: %s (%zu pts)",
      //          config_.initial_pose_map_pcd_path.c_str(), init_z_map_->size());
      SYS_LOG(syslog::LogSeverity::INFO, true) 
                << "loaded map for initial pose z completion: " << config_.initial_pose_map_pcd_path << " (" << init_z_map_->size() << " pts)";
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
  LocalizationV2Config config_;

  FusionCore fusion_;
  LocalizationStateMachine state_machine_;
  std::unique_ptr<IRelocalizationClient> relocalization_;

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

  sensor_msgs::PointCloud2::ConstPtr latest_cloud_;
  geometry_msgs::TwistWithCovariance latest_twist_;
  nav_msgs::Path path_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr init_z_map_;
  std::string latest_map_id_;
  ros::Time relocalizing_since_;
  bool reloc_result_pending_ = false;
  Pose3d reloc_result_map_base_ = Pose3d();
  double reloc_result_inlier_ratio_ = 0.0;
  double reloc_result_inlier_rmse_ = std::numeric_limits<double>::infinity();
  std::string reloc_result_map_id_;
  std::string reloc_result_method_;
  ros::Time reloc_result_stamp_;
  bool have_gm_candidate_ = false;
  double gm_candidate_inlier_ratio_ = 0.0;
  double gm_candidate_inlier_rmse_ = std::numeric_limits<double>::infinity();
  Pose3d gm_candidate_map_odom_ = Pose3d();
  std::string gm_candidate_map_id_;

  // 健康监测
  std::shared_ptr<deeplumin_health_checker::HealthChecker> health_checker_;

  std::ofstream traj_file_;

  // 跟踪失效检测
  ros::Time tracking_lost_since_;
};

}  // namespace localization_v2

int main(int argc, char** argv) {
  ros::init(argc, argv, "localization_v2");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  // 日志初始化
  std::string  log_dir;
  pnh.param<std::string>("log_dir", log_dir, "/home/hhy/2004ros/DeepLumin_zq/src/localization/loclog");
  SYS_LOG_INIT_FULL(log_dir,
                    syslog::LogSeverity::INFO,
                    50,
                    "localization_v2");

  localization_v2::LocalizationV2Node node(nh, pnh);
  ros::spin();
  return 0;
}