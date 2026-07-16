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

#include <deeplumin_msgs/CanReceiveInfo.h>
#include <deeplumin_msgs/Gpchc.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>

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

    ROS_INFO("[dr_odometry] imu=%s can=%s gnss=%s use_gnss=%s use_wheel=%s output=%s",
             topics_.imu.c_str(), topics_.can.c_str(), topics_.gnss.c_str(),
             config_.use_gnss ? "true" : "false", config_.use_wheel ? "true" : "false",
             topics_.odom.c_str());
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
    eskf_.feedImu(dr_odometry_ros::transformImuToBase(dr_odometry_ros::fromRos(*msg),
                                                      extrinsics_.base_to_imu));
  }

  void onCan(const deeplumin_msgs::CanReceiveInfo::ConstPtr& msg) {
    eskf_.feedCan(dr_odometry_ros::fromRos(*msg, can_speed_is_kmh_));
  }

  void onGnss(const deeplumin_msgs::Gpchc::ConstPtr& msg) {
    eskf_.feedGnss(dr_odometry_ros::transformGnssToBase(dr_odometry_ros::fromRos(*msg),
                                                        extrinsics_.base_to_gnss));
  }

  /**
   * @brief 发布 odom / TF / path / status。
   * 未初始化前 (valid=false) 不发布，避免下游误用零位姿。
   */
  void publish() {
    const auto odom = eskf_.odometry();
    if (!odom.valid) {
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

    const auto health = eskf_.health(ros::Time::now().toSec());
    status_pub_.publish(dr_odometry_ros::toStatusMsg(health, ros::Time::now()));
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  dr_odometry::DrConfig config_;
  dr_odometry_ros::TopicConfig topics_;
  dr_odometry_ros::FrameConfig frames_;
  dr_odometry_ros::ExtrinsicConfig extrinsics_;
  dr_odometry::DrEskf eskf_;
  bool can_speed_is_kmh_ = true;
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
