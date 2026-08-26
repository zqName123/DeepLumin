/**
 * @file pose_fusion_node.cpp
 * @brief pose_fusion ROS 节点入口：订阅多源观测 → 喂入 PoseFusionCore → 定时发布融合结果与 TF
 *
 * 启动路径：
 *   pose_fusion.launch
 *     → rosparam 加载 pose_fusion.yaml 到 ~ 私有参数
 *     → 可执行文件 pose_fusion_node
 *     → main() → PoseFusionNode 构造 → ros::spin()
 *
 * 数据流（异步回调 + 定时输出）：
 *   ┌─ /localization/dr_odom              → feedDrOdom          （预测 / 递推 odom_base）
 *   ├─ /localization/slam_odom            → feedSlamOdom        （局部平滑纠偏 odom_base）
 *   ├─ /localization/global_matcher/result→ feedGlobalMatch     （全局平滑更新 map_odom）
 *   ├─ .../accepted_relocalization        → feedRelocalization  （硬重置 map_odom）
 *   ├─ .../observer_policy                → setObserverPolicy   （开关与权重）
 *   └─ /initialpose                       → reset               （手动初值硬重置）
 *                         │
 *                         ▼
 *                  PoseFusionCore（无 ROS 依赖的融合内核）
 *                         │
 *              定时器 onTimer @ output_rate_hz
 *                         │
 *         ┌───────────────┼───────────────┬──────────────┐
 *         ▼               ▼               ▼              ▼
 *   fused_odom      fused_pose      fusion_status    TF(map→odom)
 *   fused_path（可选）
 */

#include "pose_fusion/core/pose_fusion_core.hpp"
#include "pose_fusion/ros/param_loader.hpp"
#include "pose_fusion/ros/ros_adapter.hpp"
#include "pose_fusion/ros/tf_manager.hpp"

#include <deeplumin_msgs/GlobalMatchResult.h>
#include <deeplumin_msgs/ObserverPolicy.h>
#include <deeplumin_msgs/RelocalizationResult.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include <algorithm>

/**
 * @brief ROS 外壳：只做消息收发与类型转换，融合算法全部委托给 PoseFusionCore。
 *
 * 设计意图：
 *   - 保持 core 可单测、无 ROS 依赖
 *   - 所有话题名 / 帧名 / 门限来自 FusionConfig（YAML）
 */
class PoseFusionNode {
 public:
  PoseFusionNode() : nh_(), pnh_("~") {
    // 1) 从私有参数服务器读取 YAML（launch 已 load）
    config_ = pose_fusion_ros::loadFusionConfig(pnh_);
    // 2) 初始化融合内核：质量门、状态机、默认协方差、UNINITIALIZED
    core_.initialize(config_);

    // ---------- 输入订阅 ----------
    // DR：高频局部里程计，用于递推 odom_base（队列偏大，避免丢帧）
    dr_sub_ = nh_.subscribe(config_.topics.dr_odom, 200, &PoseFusionNode::onDrOdom, this);
    // SLAM：局部观测，用于平滑纠偏 odom_base
    slam_sub_ = nh_.subscribe(config_.topics.slam_odom, 50, &PoseFusionNode::onSlamOdom, this);
    // 全局匹配：提供 map 系位姿，用于更新 map_odom
    global_match_sub_ = nh_.subscribe(config_.topics.global_matcher_result, 10,
                                      &PoseFusionNode::onGlobalMatch, this);
    // localization_manager 已接受的重定位：硬重置融合状态
    relocalization_sub_ = nh_.subscribe(config_.topics.accepted_relocalization, 5,
                                        &PoseFusionNode::onRelocalization, this);
    // 观测策略：各源开关与权重（场景自适应）
    policy_sub_ = nh_.subscribe(config_.topics.observer_policy, 5, &PoseFusionNode::onPolicy, this);
    // RViz / 人工给出的 map 系初值
    initial_pose_sub_ = nh_.subscribe(config_.topics.initial_pose, 2, &PoseFusionNode::onInitialPose, this);

    // ---------- 输出发布 ----------
    fused_odom_pub_ = nh_.advertise<nav_msgs::Odometry>(config_.topics.fused_odom, 50);
    fused_pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(config_.topics.fused_pose, 50);
    path_pub_ = nh_.advertise<nav_msgs::Path>(config_.topics.fused_path, 5);
    status_pub_ = nh_.advertise<deeplumin_msgs::FusionStatus>(config_.topics.fusion_status, 10);

    // 路径消息固定在 map 系；具体点在 onTimer 中追加
    path_.header.frame_id = config_.frames.map;
    // 定频输出：即使观测抖动，下游也能拿到稳定频率的融合位姿/状态
    timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(1.0, config_.runtime.output_rate_hz)),
                            &PoseFusionNode::onTimer, this);

    ROS_INFO("[pose_fusion] started dr=%s slam=%s global_matcher=%s fused=%s map=%s odom=%s base=%s",
             config_.topics.dr_odom.c_str(), config_.topics.slam_odom.c_str(),
             config_.topics.global_matcher_result.c_str(), config_.topics.fused_odom.c_str(),
             config_.frames.map.c_str(), config_.frames.odom.c_str(), config_.frames.base_link.c_str());
  }

 private:
  /** DR 里程计回调：ROS Odometry → PoseObservation → core 递推 */
  void onDrOdom(const nav_msgs::Odometry::ConstPtr& msg) {
    core_.feedDrOdom(pose_fusion_ros::fromOdom(*msg, pose_fusion::ObservationSource::DR));
  }

  /** SLAM 里程计回调：局部系平滑更新 */
  void onSlamOdom(const nav_msgs::Odometry::ConstPtr& msg) {
    core_.feedSlamOdom(pose_fusion_ros::fromOdom(*msg, pose_fusion::ObservationSource::SLAM));
  }

  /** 全局匹配结果回调：更新 map→odom 外参 */
  void onGlobalMatch(const deeplumin_msgs::GlobalMatchResult::ConstPtr& msg) {
    core_.feedGlobalMatch(pose_fusion_ros::fromGlobalMatch(*msg));
  }

  /** 已接受重定位回调：校验策略后硬重置 */
  void onRelocalization(const deeplumin_msgs::RelocalizationResult::ConstPtr& msg) {
    core_.feedRelocalizationReset(pose_fusion_ros::fromRelocalization(*msg));
  }

  /** 观测策略回调：更新各观测源开关与权重 */
  void onPolicy(const deeplumin_msgs::ObserverPolicy::ConstPtr& msg) {
    core_.setObserverPolicy(pose_fusion_ros::fromObserverPolicy(*msg));
  }

  /**
   * 手动初值回调（如 RViz 2D Pose Estimate）。
   * 若 initialization/allow_initialpose=false 则忽略，避免误触发重置。
   */
  void onInitialPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    if (!config_.initialization.allow_initialpose) {
      ROS_WARN_THROTTLE(2.0, "[pose_fusion] ignore /initialpose because initialization/allow_initialpose=false");
      return;
    }
    core_.reset(pose_fusion_ros::fromInitialPose(*msg, config_.default_map_id));
  }

  /**
   * 定频输出回调（核心对外节拍）：
   *   1) core.tick(now)        — 惰性初始化 + 状态机刷新 + 组装 FusionOutput
   *   2) core.status(now)      — 诊断/新鲜度/最近使用与拒绝源
   *   3) 始终发布 FusionStatus；仅当 output.valid 时发布位姿/TF/路径
   */
  void onTimer(const ros::TimerEvent&) {
    const ros::Time now = ros::Time::now();
    const auto output = core_.tick(now.toSec());
    const auto status = core_.status(now.toSec());

    // 状态话题始终发，便于上层监控“未初始化 / 降级”
    status_pub_.publish(pose_fusion_ros::toStatusMsg(status, config_.frames.map));
    if (!output.valid) {
      return;
    }

    // map 系融合里程计与带协方差位姿
    fused_odom_pub_.publish(pose_fusion_ros::toOdomMsg(output, config_.frames.map, config_.frames.base_link));
    fused_pose_pub_.publish(pose_fusion_ros::toPoseMsg(output, config_.frames.map));
    // TF：默认发 map→odom；odom→base 由 runtime 开关控制
    tf_manager_.publish(output, config_, now);

    if (config_.runtime.publish_path) {
      path_.header.stamp = now;
      path_.poses.push_back(pose_fusion_ros::toPathPose(output, config_.frames.map));
      // 环形裁剪，防止路径无限增长
      if (static_cast<int>(path_.poses.size()) > config_.runtime.max_path_size) {
        const int remove_count = static_cast<int>(path_.poses.size()) - config_.runtime.max_path_size;
        path_.poses.erase(path_.poses.begin(), path_.poses.begin() + remove_count);
      }
      path_pub_.publish(path_);
    }
  }

  ros::NodeHandle nh_;   ///< 全局命名空间：订阅/发布绝对话题
  ros::NodeHandle pnh_;  ///< 私有命名空间 (~)：读取 YAML 参数
  pose_fusion::FusionConfig config_;
  pose_fusion::PoseFusionCore core_;
  pose_fusion_ros::TfManager tf_manager_;

  ros::Subscriber dr_sub_;
  ros::Subscriber slam_sub_;
  ros::Subscriber global_match_sub_;
  ros::Subscriber relocalization_sub_;
  ros::Subscriber policy_sub_;
  ros::Subscriber initial_pose_sub_;
  ros::Publisher fused_odom_pub_;
  ros::Publisher fused_pose_pub_;
  ros::Publisher path_pub_;
  ros::Publisher status_pub_;
  ros::Timer timer_;
  nav_msgs::Path path_;  ///< 累计融合轨迹（map 系）
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "pose_fusion");
  PoseFusionNode node;
  ros::spin();  // 阻塞：回调 + 定时器驱动整个模块生命周期
  return 0;
}
