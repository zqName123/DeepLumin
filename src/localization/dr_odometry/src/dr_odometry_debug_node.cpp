/**
 * @file dr_odometry_debug_node.cpp
 * @brief DR 航位推算调试节点入口。
 *
 * 启动方式：roslaunch dr_odometry dr_odometry_debug.launch
 *
 * 职责（仅 ROS 层）：
 *   1. 从参数服务器加载 frames / topics / dr 配置；
 *   2. 用私有参数 debug/* 覆盖观测开关，便于消融实验；
 *   3. 按开关订阅 IMU / CAN / GNSS，转成核心层数据类型后喂给 DrEskf；
 *   4. 按 output_rate 发布 odom / path / status，可选 TF。
 *
 * 算法本身在 dr_odometry::DrEskf，消息转换在 dr_odometry_ros::*。
 */

#include "dr_odometry/core/dr_eskf.hpp"
#include "dr_odometry/ros/ros_adapter.hpp"

#include <deeplumin_msgs/CanReceiveInfo.h>
#include <deeplumin_msgs/Gpchc.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>

/**
 * @brief 调试版 DR 节点：相对正式节点可单独开关各观测源。
 *
 * 数据流：
 *   IMU/CAN/GNSS 回调 → ros_adapter::fromRos → DrEskf::feed*
 *   主循环 publish()  → DrEskf::odometry/health → toRos* 发布
 */
class DrOdometryDebugNode {
 public:
  /**
   * @brief 构造：读参、初始化滤波、按开关建订阅/发布。
   * @param nh  全局 NodeHandle（订阅/发布用话题名来自 yaml topics/*）
   *            私有 "~" 用于读 debug/* 与 dr/*
   */
  explicit DrOdometryDebugNode(ros::NodeHandle& nh) : nh_(nh), pnh_("~") {
    // ---- 基础配置：与正式节点相同的三块参数 ----
    config_ = dr_odometry_ros::loadDrConfig(pnh_);
    topics_ = dr_odometry_ros::loadTopicConfig(pnh_);
    frames_ = dr_odometry_ros::loadFrameConfig(pnh_);

    // ---- 调试覆盖：launch 写入的 debug/* 优先于 yaml 的 dr/* ----
    // use_imu_ 只决定是否订阅 IMU，不写进 DrConfig（滤波侧默认总假设有 IMU 预测）
    pnh_.param<bool>("debug/use_imu", use_imu_, true);
    pnh_.param<bool>("debug/use_wheel", config_.use_wheel, config_.use_wheel);
    pnh_.param<bool>("debug/use_gnss", config_.use_gnss, config_.use_gnss);
    pnh_.param<bool>("debug/use_gnss_heading", config_.use_gnss_heading, config_.use_gnss_heading);
    pnh_.param<bool>("debug/use_gnss_position", config_.use_gnss_position, config_.use_gnss_position);
    pnh_.param<bool>("debug/use_gnss_velocity", config_.use_gnss_velocity, config_.use_gnss_velocity);
    pnh_.param<bool>("debug/use_imu_accel", config_.use_imu_accel, config_.use_imu_accel);
    // CAN 速度单位：与 ysw_loc 兼容时一般为 km/h
    pnh_.param<bool>("can_speed_is_kmh", can_speed_is_kmh_, true);

    eskf_.initialize(config_);

    // ---- 输入订阅：队列深度按传感器频率大致设定 ----
    if (use_imu_) {
      // IMU 通常 100–200 Hz，队列取较大以免丢帧
      imu_sub_ = nh_.subscribe(topics_.imu, 2000, &DrOdometryDebugNode::onImu, this);
    }
    if (config_.use_wheel) {
      can_sub_ = nh_.subscribe(topics_.can, 200, &DrOdometryDebugNode::onCan, this);
    }
    if (config_.use_gnss) {
      ROS_INFO("sub gnss!");
      gnss_sub_ = nh_.subscribe(topics_.gnss, 100, &DrOdometryDebugNode::onGnss, this);
    }

    // ---- 输出话题 ----
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>(topics_.odom, 50);
    path_pub_ = nh_.advertise<nav_msgs::Path>(topics_.path, 5);
    status_pub_ = nh_.advertise<deeplumin_msgs::LocalizationStatus>(topics_.status, 10);
    path_.header.frame_id = frames_.odom;

    ROS_INFO("[dr_odometry_debug] use_imu=%s use_wheel=%s use_gnss=%s heading=%s position=%s velocity=%s",
             use_imu_ ? "true" : "false", config_.use_wheel ? "true" : "false",
             config_.use_gnss ? "true" : "false", config_.use_gnss_heading ? "true" : "false",
             config_.use_gnss_position ? "true" : "false", config_.use_gnss_velocity ? "true" : "false");
  }

  /** @brief 固定频率主循环：处理回调并发布当前滤波结果。 */
  void spin() {
    ros::Rate rate(config_.output_rate);
    while (ros::ok()) {
      ros::spinOnce();  // 处理订阅队列中积压的传感器消息
      publish();        // 按输出频率取最新状态发布（滤波已在 IMU 回调中推进）
      rate.sleep();
    }
  }

 private:
  /** IMU：转内部 ImuData 后喂滤波；首帧负责 initialize，后续走 predict + 观测更新。 */
  void onImu(const sensor_msgs::Imu::ConstPtr& msg) { eskf_.feedImu(dr_odometry_ros::fromRos(*msg)); }

  /** CAN：车速（含档位方向）写入滤波缓存，真正修正在 IMU 驱动的 correctWheel。 */
  void onCan(const deeplumin_msgs::CanReceiveInfo::ConstPtr& msg) {
    eskf_.feedCan(dr_odometry_ros::fromRos(*msg, can_speed_is_kmh_));
  }

  /** GNSS：缓存观测；首帧有效位置可定 ENU 原点，有效航向可预置 yaw。 */
  void onGnss(const deeplumin_msgs::Gpchc::ConstPtr& msg) { eskf_.feedGnss(dr_odometry_ros::fromRos(*msg)); }

  /**
   * @brief 发布当前 DR 结果。
   * 若滤波尚未由 IMU 初始化（valid=false），则跳过本周期，避免发零位姿误导下游。
   */
  void publish() {
    const auto odom = eskf_.odometry();
    if (!odom.valid) {
      return;
    }

    auto odom_msg = dr_odometry_ros::toRosOdom(odom, frames_.odom, frames_.base_link);
    odom_pub_.publish(odom_msg);

    if (config_.publish_tf) {
      tf_broadcaster_.sendTransform(dr_odometry_ros::toRosTransform(odom, frames_.odom, frames_.base_link));
    }

    // Path：供 RViz 看轨迹；长度上限 10000，超限删最旧 2000 点防止内存膨胀
    geometry_msgs::PoseStamped pose;
    pose.header = odom_msg.header;
    pose.pose = odom_msg.pose.pose;
    path_.header.stamp = odom_msg.header.stamp;
    path_.poses.push_back(pose);
    if (path_.poses.size() > 10000) {
      path_.poses.erase(path_.poses.begin(), path_.poses.begin() + 2000);
    }
    path_pub_.publish(path_);

    // 健康状态：是否有 IMU/轮速/GNSS 等，供 localization_manager 等下游决策
    status_pub_.publish(dr_odometry_ros::toStatusMsg(eskf_.health(ros::Time::now().toSec()), ros::Time::now()));
  }

  ros::NodeHandle nh_;   ///< 全局句柄：订阅/发布
  ros::NodeHandle pnh_;  ///< 私有句柄 "~"：读参数

  dr_odometry::DrConfig config_;           ///< ESKF 与发布相关配置（含被 debug 覆盖后的开关）
  dr_odometry_ros::TopicConfig topics_;    ///< 输入输出话题名
  dr_odometry_ros::FrameConfig frames_;    ///< 坐标系名
  dr_odometry::DrEskf eskf_;               ///< 误差状态卡尔曼滤波核心

  bool use_imu_ = true;           ///< 调试：是否订阅 IMU
  bool can_speed_is_kmh_ = true;  ///< CAN speed 是否按 km/h 解释

  ros::Subscriber imu_sub_;
  ros::Subscriber can_sub_;
  ros::Subscriber gnss_sub_;
  ros::Publisher odom_pub_;
  ros::Publisher path_pub_;
  ros::Publisher status_pub_;
  nav_msgs::Path path_;  ///< 累积轨迹，用于可视化
  tf::TransformBroadcaster tf_broadcaster_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "dr_odometry_debug");
  ros::NodeHandle nh;
  DrOdometryDebugNode node(nh);
  node.spin();
  return 0;
}
