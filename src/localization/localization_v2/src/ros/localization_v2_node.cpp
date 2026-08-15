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

#include <deeplumin_msgs/FusionStatus.h>
#include <deeplumin_msgs/GlobalMatchResult.h>
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

namespace localization_v2 {

namespace {

constexpr double kPi = 3.14159265358979323846;

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
  LocalizationV2Node(ros::NodeHandle& nh, ros::NodeHandle& pnh)
      : nh_(nh), pnh_(pnh), config_(loadLocalizationV2Config(pnh_)) {
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
    state_machine_.configure(init, policy);

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
    if (config_.publish_current_scan_in_map) {
      current_scan_map_pub_ =
          nh_.advertise<sensor_msgs::PointCloud2>(config_.current_scan_in_map_topic, 2);
    }
    path_.header.frame_id = config_.map_frame;

    // ---- 定时器 ----
    timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(1.0, config_.output_rate_hz)),
                             &LocalizationV2Node::onTimer, this);

    ROS_INFO("[localization_v2] started slam=%s gm=%s reloc_result=%s reloc_cmd=%s fused=%s",
             config_.slam_odom_topic.c_str(), config_.global_matcher_result_topic.c_str(),
             config_.relocalization_result_topic.c_str(),
             config_.relocalization_command_topic.c_str(), config_.fused_odom_topic.c_str());
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

    if (!update.success) {
      ROS_WARN_THROTTLE(2.0, "[localization_v2] ignore rejected global matcher result: %s",
                        msg->reject_reason.c_str());
      return;
    }

    const bool accepted = fusion_.updateFromGlobalMatch(update);
    if (!accepted) {
      ROS_WARN_THROTTLE(1.0, "[localization_v2] reject global matcher result (jump guard)");
      return;
    }

    // 记录最近一次被接受的 GM 结果指标，供重定位结果比较
    have_latest_gm_ = true;
    latest_gm_inlier_ratio_ = msg->inlier_ratio;
    latest_gm_inlier_rmse_ = msg->inlier_rmse;

    // GM 自身成功：若尚未初始化则完成初始化并停止重定位
    state_machine_.onGlobalMatchSuccess();
    dispatchRelocalizationCommand();
    latest_map_id_ = msg->map_id;
    publishMapToOdom();
    ROS_INFO("[localization_v2] updated map->odom from global matcher map_id=%s fitness=%.4f inlier=%.3f rmse=%.3f",
             msg->map_id.c_str(), msg->fitness_score, msg->inlier_ratio, msg->inlier_rmse);
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
        ROS_WARN_THROTTLE(2.0, "[localization_v2] unknown initial pose frame '%s', assume %s",
                          pose_frame.c_str(), config_.map_frame.c_str());
      }
    }

    // z 补全（取人工位姿附近地图点云高度范围中值）
    double init_z = 0.0;
    if (computeInitPoseZ(map_base.position, &init_z)) {
      map_base.position.z() = init_z;
    }

    if (!fusion_.updateMapOdomFromMapBase(map_base, /*require_slam=*/false)) {
      ROS_WARN("[localization_v2] invalid manual initial pose, ignored");
      return;
    }
    fusion_.requestJumpGuardBypass();
    state_machine_.onManualPose();
    dispatchRelocalizationCommand();
    publishMapToOdom();
    ROS_INFO("[localization_v2] reset map->odom from manual initial pose frame=%s z=%.3f",
             pose_frame.c_str(), map_base.position.z());
  }

  void onRelocalizationResult(const RelocalizationSignal& signal) {
    // 仅在重定位阶段消费结果；进入 TRACKING 后忽略（避免旧结果反复重置）
    if (!state_machine_.relocalizationRunning()) {
      return;
    }
    if (!signal.success) {
      ROS_INFO_THROTTLE(2.0, "[localization_v2] relocalization not ready: %s",
                        signal.method.c_str());
      return;
    }
    if (config_.relocalization_require_accepted && !signal.accepted) {
      ROS_WARN_THROTTLE(2.0, "[localization_v2] relocalization result not accepted");
      return;
    }
    // 质量门限
    if (signal.fitness_score > config_.relocalization_max_fitness_score) {
      ROS_WARN_THROTTLE(2.0, "[localization_v2] relocalization fitness %.3f above limit %.3f",
                        signal.fitness_score, config_.relocalization_max_fitness_score);
      return;
    }
    if (signal.inlier_ratio < config_.relocalization_min_inlier_ratio) {
      ROS_WARN_THROTTLE(2.0, "[localization_v2] relocalization inlier ratio %.3f below limit %.3f",
                        signal.inlier_ratio, config_.relocalization_min_inlier_ratio);
      return;
    }
    if (signal.inlier_rmse > config_.relocalization_max_inlier_rmse) {
      ROS_WARN_THROTTLE(2.0, "[localization_v2] relocalization rmse %.3f above limit %.3f",
                        signal.inlier_rmse, config_.relocalization_max_inlier_rmse);
      return;
    }

    // 通过门限：与最近一次 GM 成功结果直接比较内点率 / RMSE，
    // 重定位更优（或尚无 GM 结果）时以重定位位姿完成初始化，否则保留 GM 初值。
    const bool reloc_better =
        !have_latest_gm_ || (signal.inlier_ratio >= latest_gm_inlier_ratio_ &&
                             signal.inlier_rmse <= latest_gm_inlier_rmse_);
    if (reloc_better) {
      if (!fusion_.updateMapOdomFromMapBase(signal.map_base, /*require_slam=*/false)) {
        ROS_WARN("[localization_v2] invalid relocalization pose, ignored");
        return;
      }
      // 注意：重定位胜出时不设置 jump guard bypass，
      // 避免下一帧 GM 的旧误匹配结果无条件覆盖重定位位姿。
      latest_map_id_ = signal.map_id;
      ROS_INFO("[localization_v2] initialized from relocalization method=%s map_id=%s "
               "(reloc inlier=%.3f rmse=%.3f vs gm inlier=%.3f rmse=%.3f)",
               signal.method.c_str(), signal.map_id.c_str(), signal.inlier_ratio,
               signal.inlier_rmse, latest_gm_inlier_ratio_, latest_gm_inlier_rmse_);
    } else {
      ROS_INFO("[localization_v2] keep global matcher initial pose "
               "(gm inlier=%.3f rmse=%.3f better than reloc inlier=%.3f rmse=%.3f)",
               latest_gm_inlier_ratio_, latest_gm_inlier_rmse_, signal.inlier_ratio,
               signal.inlier_rmse);
    }
    state_machine_.onRelocalizationResult(/*accepted=*/true);
    dispatchRelocalizationCommand();
    publishMapToOdom();
  }

  void onSourceCloud(const sensor_msgs::PointCloud2::ConstPtr& msg) { latest_cloud_ = msg; }

  // ===================================================================
  // 定时输出
  // ===================================================================

  void onTimer(const ros::TimerEvent&) {
    if (!fusion_.hasSlam()) {
      publishIdentityMapOdom();
      ROS_WARN_THROTTLE(2.0, "[localization_v2] waiting for slam odom: %s",
                        config_.slam_odom_topic.c_str());
      return;
    }
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
    msg.level = deeplumin_msgs::FusionStatus::LEVEL_NORMAL;
    msg.state = state_machine_.stateName();
    msg.initialized = state_machine_.initialized();
    msg.output_valid = output_valid;
    msg.current_map_id = latest_map_id_;
    msg.is_slam_available = fusion_.hasSlam();
    msg.is_global_matcher_available = true;
    msg.is_relocalization_available = config_.relocalization_enabled;
    msg.used_sources.push_back("slam_odometry_v2");
    if (fusion_.hasMapOdom()) {
      msg.used_sources.push_back("global_matcher");
    }
    if (state_machine_.relocalizationRunning()) {
      msg.used_sources.push_back("relocalization");
    }
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
        ROS_ERROR("[localization_v2] failed to load map for z completion: %s",
                  config_.initial_pose_map_pcd_path.c_str());
        init_z_map_.reset();
        return false;
      }
      std::vector<int> valid_indices;
      pcl::removeNaNFromPointCloud(*init_z_map_, *init_z_map_, valid_indices);
      ROS_INFO("[localization_v2] loaded map for initial pose z completion: %s (%zu pts)",
               config_.initial_pose_map_pcd_path.c_str(), init_z_map_->size());
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
  InitStateMachine state_machine_;
  std::unique_ptr<IRelocalizationClient> relocalization_;

  ros::Subscriber slam_sub_;
  ros::Subscriber gm_sub_;
  ros::Subscriber initial_pose_sub_;
  ros::Subscriber cloud_sub_;
  ros::Publisher fused_odom_pub_;
  ros::Publisher map_to_odom_pub_;
  ros::Publisher path_pub_;
  ros::Publisher status_pub_;
  ros::Publisher current_scan_map_pub_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  ros::Timer timer_;

  sensor_msgs::PointCloud2::ConstPtr latest_cloud_;
  geometry_msgs::TwistWithCovariance latest_twist_;
  nav_msgs::Path path_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr init_z_map_;
  std::string latest_map_id_;
  bool have_latest_gm_ = false;
  double latest_gm_inlier_ratio_ = 0.0;
  double latest_gm_inlier_rmse_ = std::numeric_limits<double>::infinity();
};

}  // namespace localization_v2

int main(int argc, char** argv) {
  ros::init(argc, argv, "localization_v2");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  localization_v2::LocalizationV2Node node(nh, pnh);
  ros::spin();
  return 0;
}
