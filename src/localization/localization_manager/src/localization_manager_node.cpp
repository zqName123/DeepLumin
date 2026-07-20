/**
 * @file localization_manager_node.cpp
 * @brief localization_manager ROS 节点入口：监控各定位子模块 → 输出状态机与观测策略
 *
 * 启动路径：
 *   localization_manager.launch
 *     → rosparam load localization_manager.yaml
 *     → localization_manager_node
 *     → main() → LocalizationManagerNode → ros::spin()
 *
 * 数据流（异步回调更新快照 + 定频决策输出）：
 *   ┌─ DR / SLAM / Fusion / GM / Reloc / GNSS 状态与里程计
 *   ├─ /initialpose / /system/scene_state（人工干预）
 *   │        ↓
 *   │   ManagerCore（缓存 ModuleHealth + 最新消息）
 *   │        ↓
 *   └─ onTimer @ rate_hz
 *         ├─ updateScene   → 场景：INDOOR / OUTDOOR_OPEN / SEMI_OUTDOOR
 *         ├─ updateState   → 状态：WAIT_INITIAL_POSE / LOCALIZATION / DEGRADED / ...
 *         ├─ updateStatus  → ManagerSnapshot
 *         ├─ updatePolicy  → ObserverPolicyData
 *         └─ 发布 manager_status / observer_policy / accepted_relocalization
 *
 * 下游消费者：
 *   pose_fusion 订阅 observer_policy 与 accepted_relocalization
 */

#include "localization_manager/core/manager_core.hpp"
#include "localization_manager/core/map_height_provider.hpp"
#include "localization_manager/ros/ros_adapter.hpp"

#include <deeplumin_msgs/FusionStatus.h>
#include <deeplumin_msgs/GlobalMatchResult.h>
#include <deeplumin_msgs/Gpchc.h>
#include <deeplumin_msgs/LocalizationStatus.h>
#include <deeplumin_msgs/RelocalizationResult.h>
#include <deeplumin_msgs/SlamHealth.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <algorithm>
#include <cmath>
#include <string>

namespace {

/** 消息 stamp 为 0 时回退到 now，避免 age 计算异常 */
template <typename MsgT>
ros::Time msgTime(const MsgT& msg) {
  return msg.header.stamp.isZero() ? ros::Time::now() : msg.header.stamp;
}

std::string normalizeFrameId(const std::string& frame_id) {
  if (!frame_id.empty() && frame_id.front() == '/') {
    return frame_id.substr(1);
  }
  return frame_id;
}

}  // namespace

/**
 * @brief ROS 外壳：订阅多源健康信息，定频驱动 ManagerCore 决策并发布策略。
 *
 * 设计意图：
 *   - core 无 ROS 依赖，可单测状态机与门控逻辑
 *   - 回调只做“快照更新”，状态转移集中在 onTimer → core.update()
 */
class LocalizationManagerNode {
 public:
  LocalizationManagerNode() : nh_(), pnh_("~") {
    // 1) 从私有参数服务器读取 YAML
    config_ = localization_manager_ros::loadConfig(pnh_);
    // 2) 初始化内核：默认 WAIT_INITIAL_POSE
    core_.initialize(config_);
    if (config_.initialization.complete_manual_initialpose_z &&
        config_.initial_pose_completion.enabled &&
        height_provider_.initialize(config_.initial_pose_completion)) {
      ROS_INFO("[localization_manager] map height provider loaded pcd=%s",
               height_provider_.mapPath().c_str());
    } else if (config_.initialization.complete_manual_initialpose_z) {
      ROS_WARN("[localization_manager] map height provider unavailable, manual initialpose will be rejected unless raw fallback is enabled");
    }

    // ---------- 输入订阅：各子模块健康与输出 ----------
    dr_odom_sub_ = nh_.subscribe(config_.topics.dr_odom, 20, &LocalizationManagerNode::onDrOdom, this);
    dr_status_sub_ = nh_.subscribe(config_.topics.dr_status, 20, &LocalizationManagerNode::onDrStatus, this);
    slam_odom_sub_ = nh_.subscribe(config_.topics.slam_odom, 20, &LocalizationManagerNode::onSlamOdom, this);
    slam_health_sub_ = nh_.subscribe(config_.topics.slam_health, 20, &LocalizationManagerNode::onSlamHealth, this);
    fused_odom_sub_ = nh_.subscribe(config_.topics.fused_odom, 20, &LocalizationManagerNode::onFusedOdom, this);
    fusion_status_sub_ = nh_.subscribe(config_.topics.fusion_status, 20,
                                       &LocalizationManagerNode::onFusionStatus, this);
    global_match_result_sub_ = nh_.subscribe(config_.topics.global_matcher_result, 20,
                                             &LocalizationManagerNode::onGlobalMatchResult, this);
    global_match_status_sub_ = nh_.subscribe(config_.topics.global_matcher_status, 20,
                                             &LocalizationManagerNode::onGlobalMatchStatus, this);
    reloc_result_sub_ = nh_.subscribe(config_.topics.relocalization_result, 20,
                                      &LocalizationManagerNode::onRelocalizationResult, this);
    gnss_sub_ = nh_.subscribe(config_.topics.gnss, 20, &LocalizationManagerNode::onGnss, this);
    initialpose_sub_ = nh_.subscribe(config_.topics.initialpose, 5, &LocalizationManagerNode::onInitialPose, this);
    scene_state_sub_ = nh_.subscribe(config_.topics.scene_state, 5, &LocalizationManagerNode::onSceneState, this);

    // ---------- 输出发布 ----------
    manager_status_pub_ = nh_.advertise<deeplumin_msgs::LocalizationStatus>(config_.topics.manager_status, 10);
    observer_policy_pub_ = nh_.advertise<deeplumin_msgs::ObserverPolicy>(config_.topics.observer_policy, 10);
    // latched：新订阅者能拿到最近一次已接受重定位
    accepted_relocalization_pub_ = nh_.advertise<deeplumin_msgs::RelocalizationResult>(
        config_.topics.accepted_relocalization, 5, true);
    manager_initial_pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(
        config_.topics.manager_initial_pose, 5, true);

    // 定频决策节拍：状态机 + 策略输出
    timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(1.0, config_.runtime.rate_hz)),
                             &LocalizationManagerNode::onTimer, this);

    ROS_INFO("[localization_manager] started");
    ROS_INFO("[localization_manager] dr=%s slam=%s fused=%s policy=%s reloc=%s",
             config_.topics.dr_status.c_str(), config_.topics.slam_health.c_str(),
             config_.topics.fusion_status.c_str(), config_.topics.observer_policy.c_str(),
             config_.topics.accepted_relocalization.c_str());
  }

 private:
  /** DR 里程计到达：更新 dr_odom_ 健康快照（仅时间戳/可用性） */
  void onDrOdom(const nav_msgs::Odometry::ConstPtr& msg) {
    core_.onDrOdom(localization_manager_ros::fromOdom(*msg, "dr_odom", ros::Time::now()));
  }

  void onDrStatus(const deeplumin_msgs::LocalizationStatus::ConstPtr& msg) {
    core_.onDrStatus(localization_manager_ros::fromLocalizationStatus(*msg, "dr_status", ros::Time::now()));
  }

  void onSlamOdom(const nav_msgs::Odometry::ConstPtr& msg) {
    core_.onSlamOdom(localization_manager_ros::fromOdom(*msg, "slam_odom", ros::Time::now()));
  }

  void onSlamHealth(const deeplumin_msgs::SlamHealth::ConstPtr& msg) {
    core_.onSlamStatus(localization_manager_ros::fromSlamHealth(*msg, ros::Time::now()));
  }

  /**
   * 融合里程计回调：若尚未收到 fusion_status，用 dummy 状态更新 fusion 健康度。
   * 保证仅有 fused_odom 时 manager 仍能感知融合输出在流动。
   */
  void onFusedOdom(const nav_msgs::Odometry::ConstPtr& msg) {
    if (last_fusion_status_msg_.header.stamp.isZero()) {
      deeplumin_msgs::FusionStatus dummy;
      dummy.header.stamp = msgTime(*msg);
      dummy.initialized = false;
      dummy.output_valid = true;
      core_.onFusionStatus(localization_manager_ros::fromOdom(*msg, "fusion", ros::Time::now()), dummy);
      return;
    }
    core_.onFusionStatus(localization_manager_ros::fromOdom(*msg, "fusion", ros::Time::now()),
                         last_fusion_status_msg_);
  }

  /** 融合状态：缓存完整 FusionStatus 并更新 fusion_status_ 健康度 */
  void onFusionStatus(const deeplumin_msgs::FusionStatus::ConstPtr& msg) {
    last_fusion_status_msg_ = *msg;
    core_.onFusionStatus(localization_manager_ros::fromFusionStatus(*msg, ros::Time::now()), *msg);
  }

  void onGlobalMatchResult(const deeplumin_msgs::GlobalMatchResult::ConstPtr& msg) {
    core_.onGlobalMatchResult(localization_manager_ros::fromOdom(nav_msgs::Odometry(), "global_matcher_result",
                                                                 ros::Time::now()),
                              *msg);
  }

  void onGlobalMatchStatus(const deeplumin_msgs::LocalizationStatus::ConstPtr& msg) {
    core_.onGlobalMatchStatus(localization_manager_ros::fromGlobalMatchStatus(*msg, ros::Time::now()));
  }

  /**
   * 重定位结果：core 内 relocalizationTrusted() 门控通过后写入 accepted_relocalization_，
   * 由 onTimer 发布到 accepted_relocalization 话题供 pose_fusion 消费。
   */
  void onRelocalizationResult(const deeplumin_msgs::RelocalizationResult::ConstPtr& msg) {
    core_.onRelocalizationResult(localization_manager_ros::fromOdom(nav_msgs::Odometry(), "relocalization_result",
                                                                    ros::Time::now()),
                                 *msg);
  }

  /** GNSS：用于场景自动判定（OUTDOOR_OPEN / SEMI_OUTDOOR / INDOOR） */
  void onGnss(const deeplumin_msgs::Gpchc::ConstPtr& msg) {
    core_.onGnss(localization_manager_ros::fromGnss(*msg));
  }

  /**
   * 手动初值（RViz 2D Pose Estimate）：
   *   1) 将 RViz x/y/yaw 视为 2D 粗初值
   *   2) 尝试从局部地图补齐 z，形成 manager 认可的 3D initial pose
   *   3) 发布 latched /localization/manager/initial_pose 给 pose_fusion reanchor
   */
  void onInitialPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    if (!config_.initialization.allow_manual_initialpose) {
      ROS_WARN_THROTTLE(2.0, "[localization_manager] ignore /initialpose because manual initialpose is disabled");
      return;
    }

    const std::string input_frame = normalizeFrameId(msg->header.frame_id);
    const std::string map_frame = normalizeFrameId(config_.frames.map);
    if (!input_frame.empty() && input_frame != map_frame) {
      ROS_WARN_STREAM("[localization_manager] reject /initialpose: frame=" << msg->header.frame_id
                      << " expected=" << config_.frames.map
                      << ". Set RViz Fixed Frame to map or provide a tf-aware initializer.");
      return;
    }

    geometry_msgs::PoseWithCovarianceStamped initial_pose;
    localization_manager::HeightQueryResult height;
    if (config_.initialization.complete_manual_initialpose_z && height_provider_.ready()) {
      initial_pose = height_provider_.completePose(*msg, config_.frames.map, &height);
    } else {
      initial_pose = localization_manager_ros::toInitialPoseMsg(*msg, config_.frames.map);
      height.success = !config_.initialization.complete_manual_initialpose_z ||
                       config_.initialization.publish_raw_initialpose_directly;
      height.z = initial_pose.pose.pose.position.z;
      height.z_std = std::sqrt(std::max(0.0, initial_pose.pose.covariance[14]));
      height.source = "raw_initialpose";
      height.reason = height.success ? "raw_initialpose_allowed" : "height_provider_not_ready";
    }

    if (!height.success && !config_.initialization.publish_raw_initialpose_directly) {
      ROS_WARN_STREAM("[localization_manager] reject /initialpose: cannot complete z, reason="
                      << height.reason << " points=" << height.point_count);
      return;
    }

    if (!height.success && config_.initialization.publish_raw_initialpose_directly) {
      initial_pose = localization_manager_ros::toInitialPoseMsg(*msg, config_.frames.map);
    }

    core_.onInitialPose(msgTime(initial_pose), initial_pose.header.frame_id);
    manager_initial_pose_pub_.publish(initial_pose);
    ROS_INFO_STREAM("[localization_manager] accepted initialpose frame=" << msg->header.frame_id
                    << " -> " << initial_pose.header.frame_id
                    << " z=" << initial_pose.pose.pose.position.z
                    << " z_source=" << height.source
                    << " z_std=" << height.z_std
                    << " map_points=" << height.point_count
                    << " reason=" << height.reason);
  }

  /** 外部场景覆盖（如系统层下发的 INDOOR / OUTDOOR） */
  void onSceneState(const std_msgs::String::ConstPtr& msg) {
    if (msg->data.empty()) {
      return;
    }
    core_.onSceneState(msg->data);
    ROS_INFO_STREAM("[localization_manager] scene override -> " << msg->data);
  }

  /**
   * 定频决策回调（管理器核心节拍）：
   *   core.update → 场景/状态/快照/策略
   *   发布 manager_status、observer_policy
   *   若有待发布 accepted_relocalization 则取出并发布（一次性）
   */
  void onTimer(const ros::TimerEvent&) {
    const ros::Time now = ros::Time::now();
    core_.update(now);

    const auto& status = core_.status();
    const auto& policy = core_.observerPolicy();
    manager_status_pub_.publish(localization_manager_ros::toStatusMsg(status));
    observer_policy_pub_.publish(localization_manager_ros::toPolicyMsg(policy));

    if (core_.hasAcceptedRelocalization()) {
      auto accepted = localization_manager_ros::toAcceptedRelocalization(core_.takeAcceptedRelocalization());
      accepted_relocalization_pub_.publish(accepted);
      ROS_INFO_STREAM_THROTTLE(1.0, "[localization_manager] accepted relocalization map="
                                         << accepted.map_id << " method=" << accepted.method
                                         << " fitness=" << accepted.fitness_score
                                         << " inlier_ratio=" << accepted.inlier_ratio);
    }

    ROS_INFO_STREAM_THROTTLE(2.0, "[localization_manager] state="
                                   << localization_manager_ros::stateToString(status.state)
                                   << " scene=" << status.active_scene
                                   << " map=" << status.current_map_id
                                   << " reason=" << status.failure_reason);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  localization_manager::ManagerConfig config_;
  localization_manager::ManagerCore core_;
  localization_manager::MapHeightProvider height_provider_;
  deeplumin_msgs::FusionStatus last_fusion_status_msg_;  ///< 与 fused_odom 配对使用

  ros::Subscriber dr_odom_sub_;
  ros::Subscriber dr_status_sub_;
  ros::Subscriber slam_odom_sub_;
  ros::Subscriber slam_health_sub_;
  ros::Subscriber fused_odom_sub_;
  ros::Subscriber fusion_status_sub_;
  ros::Subscriber global_match_result_sub_;
  ros::Subscriber global_match_status_sub_;
  ros::Subscriber reloc_result_sub_;
  ros::Subscriber gnss_sub_;
  ros::Subscriber initialpose_sub_;
  ros::Subscriber scene_state_sub_;

  ros::Publisher manager_status_pub_;
  ros::Publisher observer_policy_pub_;
  ros::Publisher accepted_relocalization_pub_;
  ros::Publisher manager_initial_pose_pub_;
  ros::Timer timer_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "localization_manager");
  LocalizationManagerNode node;
  ros::spin();
  return 0;
}
