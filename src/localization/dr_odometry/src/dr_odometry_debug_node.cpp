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
#include "dr_odometry/ros/global_map_viz.hpp"

#include <deeplumin_msgs/CanReceiveInfo.h>
#include <deeplumin_msgs/Gpchc.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>

#include <cmath>


namespace {

const char* yesNo(bool value) { return value ? "true" : "false"; }

double yawDeg(const dr_odometry::Quatd& q) {
  const auto r = q.normalized().toRotationMatrix();
  return std::atan2(r(1, 0), r(0, 0)) * 180.0 / M_PI;
}

}  // namespace

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
    extrinsics_ = dr_odometry_ros::loadExtrinsicConfig(pnh_);

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
    pnh_.param<bool>("can_use_valid_flag", can_use_valid_flag_, false);

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
    global_map_viz_.setup(nh_, pnh_, "/localization/dr_odometry/global_map", "map", "dr_odometry_debug");

    ROS_INFO("[dr_odometry_debug] use_imu=%s use_wheel=%s use_gnss=%s heading=%s position=%s velocity=%s",
             yesNo(use_imu_), yesNo(config_.use_wheel), yesNo(config_.use_gnss),
             yesNo(config_.use_gnss_heading), yesNo(config_.use_gnss_position),
             yesNo(config_.use_gnss_velocity));
    ROS_INFO("[dr_odometry_debug][Config] rate=%.1fHz tf=%s imu_accel=%s can_kmh=%s can_valid_flag=%s frames=%s->%s topics imu=%s can=%s gnss=%s odom=%s",
             config_.output_rate, yesNo(config_.publish_tf), yesNo(config_.use_imu_accel),
             yesNo(can_speed_is_kmh_), yesNo(can_use_valid_flag_), frames_.odom.c_str(), frames_.base_link.c_str(),
             topics_.imu.c_str(), topics_.can.c_str(), topics_.gnss.c_str(), topics_.odom.c_str());
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
  void onImu(const sensor_msgs::Imu::ConstPtr& msg) {
    const auto imu = dr_odometry_ros::transformImuToBase(dr_odometry_ros::fromRos(*msg),
                                                         extrinsics_.base_to_imu);
    ROS_INFO_ONCE("[dr_odometry_debug][Input] first IMU stamp=%.9f frame=%s", imu.timestamp,
                  msg->header.frame_id.c_str());
    ROS_INFO_THROTTLE(5.0, "[dr_odometry_debug][Input] imu stamp=%.3f gyro=(%.4f %.4f %.4f) accel=(%.3f %.3f %.3f)",
                      imu.timestamp, imu.gyro.x(), imu.gyro.y(), imu.gyro.z(),
                      imu.accel.x(), imu.accel.y(), imu.accel.z());
    eskf_.feedImu(imu);
  }

  /** CAN：车速（含档位方向）写入滤波缓存，真正修正在 IMU 驱动的 correctWheel。 */
  void onCan(const deeplumin_msgs::CanReceiveInfo::ConstPtr& msg) {
    const auto can = dr_odometry_ros::fromRos(*msg, can_speed_is_kmh_, can_use_valid_flag_);
    ROS_INFO_ONCE("[dr_odometry_debug][Input] first CAN stamp=%.9f frame=%s", can.timestamp,
                  msg->header.frame_id.c_str());
    ROS_INFO_THROTTLE(5.0, "[dr_odometry_debug][Input] can stamp=%.3f raw_speed=%.3f gear=%.1f speed_mps=%.3f raw_valid=%s final_valid=%s",
                      can.timestamp, msg->speed, msg->gear, can.speed_mps, yesNo(msg->valid), yesNo(can.valid));
    eskf_.feedCan(can);
  }

  /** GNSS：缓存观测；首帧有效位置可定 ENU 原点，有效航向可预置 yaw。 */
  void onGnss(const deeplumin_msgs::Gpchc::ConstPtr& msg) {
    const auto gnss = dr_odometry_ros::transformGnssToBase(dr_odometry_ros::fromRos(*msg),
                                                           extrinsics_.base_to_gnss);
    ROS_INFO_ONCE("[dr_odometry_debug][Input] first GNSS stamp=%.9f frame=%s status=%d", gnss.timestamp,
                  msg->header.frame_id.c_str(), gnss.status);
    ROS_INFO_THROTTLE(5.0, "[dr_odometry_debug][Input] gnss stamp=%.3f status=%d heading=%.2fdeg yaw=%.2fdeg v_enu=(%.3f %.3f %.3f) valid(h/p/v)=%s/%s/%s",
                      gnss.timestamp, gnss.status, msg->heading, gnss.heading_rad * 180.0 / M_PI,
                      gnss.velocity_enu.x(), gnss.velocity_enu.y(), gnss.velocity_enu.z(),
                      yesNo(gnss.heading_valid), yesNo(gnss.position_valid), yesNo(gnss.velocity_valid));
    eskf_.feedGnss(gnss);
  }

  /**
   * @brief 发布当前 DR 结果。
   * 若滤波尚未由 IMU 初始化（valid=false），则跳过本周期，避免发零位姿误导下游。
   */
  void publish() {
    const auto odom = eskf_.odometry();
    if (!odom.valid) {
      const auto health = eskf_.health(ros::Time::now().toSec());
      ROS_WARN_THROTTLE(2.0, "[dr_odometry_debug][State] waiting odom init: imu=%s can=%s gnss=%s counts(i/c/g)=%lu/%lu/%lu invalid_imu=%lu",
                        yesNo(health.has_imu), yesNo(health.has_wheel), yesNo(health.has_gnss),
                        static_cast<unsigned long>(health.imu_count),
                        static_cast<unsigned long>(health.can_count),
                        static_cast<unsigned long>(health.gnss_count),
                        static_cast<unsigned long>(health.invalid_imu_count));
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

    const auto health = eskf_.health(odom.timestamp);
    ROS_INFO_THROTTLE(1.0, "[dr_odometry_debug][Odom] t=%.3f pos=(%.3f %.3f %.3f) vel=(%.3f %.3f %.3f) yaw=%.2fdeg pred=%lu wheel_upd=%lu gnss_h/v/p=%lu/%lu/%lu ages(w/g)=%.3f/%.3f stale(w/g)=%lu/%lu",
                      odom.timestamp, odom.position.x(), odom.position.y(), odom.position.z(),
                      odom.velocity.x(), odom.velocity.y(), odom.velocity.z(), yawDeg(odom.orientation),
                      static_cast<unsigned long>(health.predict_count),
                      static_cast<unsigned long>(health.wheel_update_count),
                      static_cast<unsigned long>(health.gnss_heading_update_count),
                      static_cast<unsigned long>(health.gnss_velocity_update_count),
                      static_cast<unsigned long>(health.gnss_position_update_count),
                      health.wheel_age, health.gnss_age,
                      static_cast<unsigned long>(health.stale_wheel_count),
                      static_cast<unsigned long>(health.stale_gnss_count));
    status_pub_.publish(dr_odometry_ros::toStatusMsg(health, ros::Time::now()));
  }

  ros::NodeHandle nh_;   ///< 全局句柄：订阅/发布
  ros::NodeHandle pnh_;  ///< 私有句柄 "~"：读参数

  dr_odometry::DrConfig config_;           ///< ESKF 与发布相关配置（含被 debug 覆盖后的开关）
  dr_odometry_ros::TopicConfig topics_;    ///< 输入输出话题名
  dr_odometry_ros::FrameConfig frames_;    ///< 坐标系名
  dr_odometry_ros::ExtrinsicConfig extrinsics_;  ///< 传感器到 base_link 的安装外参
  dr_odometry::DrEskf eskf_;
  dr_odometry_ros::GlobalMapVizPublisher global_map_viz_;               ///< 误差状态卡尔曼滤波核心

  bool use_imu_ = true;           ///< 调试：是否订阅 IMU
  bool can_speed_is_kmh_ = true;
  bool can_use_valid_flag_ = false;  ///< 是否严格使用 CAN msg.valid 标志

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
