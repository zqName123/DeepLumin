/**
 * @file dr_odometry_node.cpp
 * @brief DR 航位推算正式节点入口。
 *
 * 启动方式：roslaunch dr_odometry dr_odometry.launch
 *
 * 与 debug 节点的差异：
 *   - 无 debug/* 消融开关，始终按 yaml 的 dr/* 运行；
 *   - 始终订阅 IMU 与 CAN；GNSS 仅由 dr/use_gnss 控制是否订阅；
 *   - Path 缓存上限略小（5000），适合长期在线运行。
 *
 * 职责同样只有 ROS 适配：订阅 → fromRos → DrEskf → toRos → 发布。
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
 * @brief 正式版 DR 节点：生产配置下持续输出局部里程计。
 */
class DrOdometryNode {
 public:
  /** @brief 读参、初始化滤波、建立默认订阅/发布。 */
  explicit DrOdometryNode(ros::NodeHandle& nh) : nh_(nh), pnh_("~") {
    config_ = dr_odometry_ros::loadDrConfig(pnh_);
    topics_ = dr_odometry_ros::loadTopicConfig(pnh_);
    frames_ = dr_odometry_ros::loadFrameConfig(pnh_);
    extrinsics_ = dr_odometry_ros::loadExtrinsicConfig(pnh_);
    pnh_.param<bool>("can_speed_is_kmh", can_speed_is_kmh_, true);
    pnh_.param<bool>("can_use_valid_flag", can_use_valid_flag_, false);
    eskf_.initialize(config_);

    // 正式节点：IMU / CAN 必订；GNSS 按配置
    imu_sub_ = nh_.subscribe(topics_.imu, 2000, &DrOdometryNode::onImu, this);
    can_sub_ = nh_.subscribe(topics_.can, 200, &DrOdometryNode::onCan, this);
    if (config_.use_gnss) {
      gnss_sub_ = nh_.subscribe(topics_.gnss, 100, &DrOdometryNode::onGnss, this);
    }

    odom_pub_ = nh_.advertise<nav_msgs::Odometry>(topics_.odom, 50);
    status_pub_ = nh_.advertise<deeplumin_msgs::LocalizationStatus>(topics_.status, 10);
    path_pub_ = nh_.advertise<nav_msgs::Path>(topics_.path, 5);
    path_.header.frame_id = frames_.odom;
    global_map_viz_.setup(nh_, pnh_, "/localization/dr_odometry/global_map", "map", "dr_odometry");

    ROS_INFO("[dr_odometry] imu=%s can=%s gnss=%s use_gnss=%s use_wheel=%s output=%s",
             topics_.imu.c_str(), topics_.can.c_str(), topics_.gnss.c_str(),
             yesNo(config_.use_gnss), yesNo(config_.use_wheel), topics_.odom.c_str());
    ROS_INFO("[dr_odometry][Config] rate=%.1fHz tf=%s imu_accel=%s wheel=%s gnss=%s heading=%s velocity=%s position=%s can_kmh=%s can_valid_flag=%s frames=%s->%s",
             config_.output_rate, yesNo(config_.publish_tf), yesNo(config_.use_imu_accel),
             yesNo(config_.use_wheel), yesNo(config_.use_gnss), yesNo(config_.use_gnss_heading),
             yesNo(config_.use_gnss_velocity), yesNo(config_.use_gnss_position), yesNo(can_speed_is_kmh_),
             yesNo(can_use_valid_flag_), frames_.odom.c_str(), frames_.base_link.c_str());
  }

  /** @brief 固定频率：spinOnce 处理传感器回调，再发布最新状态。 */
  void spin() {
    ros::Rate rate(config_.output_rate);
    while (ros::ok()) {
      ros::spinOnce();
      publish();
      rate.sleep();
    }
  }

 private:
  void onImu(const sensor_msgs::Imu::ConstPtr& msg) {
    const auto imu = dr_odometry_ros::transformImuToBase(dr_odometry_ros::fromRos(*msg),
                                                         extrinsics_.base_to_imu);
    ROS_INFO_ONCE("[dr_odometry][Input] first IMU stamp=%.9f frame=%s", imu.timestamp,
                  msg->header.frame_id.c_str());
    ROS_INFO_THROTTLE(5.0, "[dr_odometry][Input] imu stamp=%.3f gyro=(%.4f %.4f %.4f) accel=(%.3f %.3f %.3f)",
                      imu.timestamp, imu.gyro.x(), imu.gyro.y(), imu.gyro.z(),
                      imu.accel.x(), imu.accel.y(), imu.accel.z());
    eskf_.feedImu(imu);
  }

  void onCan(const deeplumin_msgs::CanReceiveInfo::ConstPtr& msg) {
    const auto can = dr_odometry_ros::fromRos(*msg, can_speed_is_kmh_, can_use_valid_flag_);
    ROS_INFO_ONCE("[dr_odometry][Input] first CAN stamp=%.9f frame=%s", can.timestamp,
                  msg->header.frame_id.c_str());
    ROS_INFO_THROTTLE(5.0, "[dr_odometry][Input] can stamp=%.3f raw_speed=%.3f gear=%.1f speed_mps=%.3f raw_valid=%s final_valid=%s",
                      can.timestamp, msg->speed, msg->gear, can.speed_mps, yesNo(msg->valid), yesNo(can.valid));
    eskf_.feedCan(can);
  }

  void onGnss(const deeplumin_msgs::Gpchc::ConstPtr& msg) {
    const auto gnss = dr_odometry_ros::transformGnssToBase(dr_odometry_ros::fromRos(*msg),
                                                           extrinsics_.base_to_gnss);
    ROS_INFO_ONCE("[dr_odometry][Input] first GNSS stamp=%.9f frame=%s status=%d", gnss.timestamp,
                  msg->header.frame_id.c_str(), gnss.status);
    ROS_INFO_THROTTLE(5.0, "[dr_odometry][Input] gnss stamp=%.3f status=%d heading=%.2fdeg yaw=%.2fdeg v_enu=(%.3f %.3f %.3f) valid(h/p/v)=%s/%s/%s",
                      gnss.timestamp, gnss.status, msg->heading, gnss.heading_rad * 180.0 / M_PI,
                      gnss.velocity_enu.x(), gnss.velocity_enu.y(), gnss.velocity_enu.z(),
                      yesNo(gnss.heading_valid), yesNo(gnss.position_valid), yesNo(gnss.velocity_valid));
    eskf_.feedGnss(gnss);
  }

  /**
   * @brief 发布 odom / TF / path / status。
   * 未初始化前 (valid=false) 不发布，避免下游误用零位姿。
   */
  void publish() {
    const auto odom = eskf_.odometry();
    if (!odom.valid) {
      const auto health = eskf_.health(ros::Time::now().toSec());
      ROS_WARN_THROTTLE(2.0, "[dr_odometry][State] waiting odom init: imu=%s can=%s gnss=%s counts(i/c/g)=%lu/%lu/%lu invalid_imu=%lu",
                        yesNo(health.has_imu), yesNo(health.has_wheel), yesNo(health.has_gnss),
                        static_cast<unsigned long>(health.imu_count),
                        static_cast<unsigned long>(health.can_count),
                        static_cast<unsigned long>(health.gnss_count),
                        static_cast<unsigned long>(health.invalid_imu_count));
      return;
    }
    const auto odom_msg = dr_odometry_ros::toRosOdom(odom, frames_.odom, frames_.base_link);
    odom_pub_.publish(odom_msg);
    if (config_.publish_tf) {
      tf_broadcaster_.sendTransform(dr_odometry_ros::toRosTransform(odom, frames_.odom, frames_.base_link));
    }

    geometry_msgs::PoseStamped pose;
    pose.header = odom_msg.header;
    pose.pose = odom_msg.pose.pose;
    path_.header.stamp = odom_msg.header.stamp;
    path_.poses.push_back(pose);
    // 在线跑长时间：上限 5000，超限删最旧 1000
    if (path_.poses.size() > 5000) {
      path_.poses.erase(path_.poses.begin(), path_.poses.begin() + 1000);
    }
    path_pub_.publish(path_);

    const auto health = eskf_.health(odom.timestamp);
    ROS_INFO_THROTTLE(1.0, "[dr_odometry][Odom] t=%.3f pos=(%.3f %.3f %.3f) vel=(%.3f %.3f %.3f) yaw=%.2fdeg pred=%lu wheel_upd=%lu gnss_h/v/p=%lu/%lu/%lu ages(w/g)=%.3f/%.3f stale(w/g)=%lu/%lu",
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

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  dr_odometry::DrConfig config_;
  dr_odometry_ros::TopicConfig topics_;
  dr_odometry_ros::FrameConfig frames_;
  dr_odometry_ros::ExtrinsicConfig extrinsics_;
  dr_odometry::DrEskf eskf_;
  dr_odometry_ros::GlobalMapVizPublisher global_map_viz_;
  bool can_speed_is_kmh_ = true;
  bool can_use_valid_flag_ = false;
  ros::Subscriber imu_sub_;
  ros::Subscriber can_sub_;
  ros::Subscriber gnss_sub_;
  ros::Publisher odom_pub_;
  ros::Publisher status_pub_;
  ros::Publisher path_pub_;
  nav_msgs::Path path_;
  tf::TransformBroadcaster tf_broadcaster_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "dr_odometry");
  ros::NodeHandle nh;
  DrOdometryNode node(nh);
  node.spin();
  return 0;
}
