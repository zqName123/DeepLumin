/**
 * @file dr_odometry_v2_node.cpp
 * @brief dr_odometry_v2 节点（薄编排层）
 *
 * 职责：订阅传感器/定位 → convert → EventQueue 入队 → 定时 drain（时间序处理）→ 发布。
 * 业务数学全部在 core：eskf（预测/观测/更新）、pose_gate（跳变判断）、event_queue（时间同步）。
 *
 * 时间同步（设计文档 §5）：IMU 时基；drain 按 stamp 升序处理（自然形成 ~10ms 重排窗）；
 * 观测传播到其精确时刻再更新；输出前向外推到发布时刻。
 */

#include <mutex>
#include <string>

#include <deeplumin_msgs/CanReceiveInfo.h>
#include <deeplumin_msgs/FusionStatus.h>
#include <deeplumin_msgs/Gpchc.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>

#include <syslog/logger.h>

#include "dr_odometry_v2/core/eskf.hpp"
#include "dr_odometry_v2/core/event_queue.hpp"
#include "dr_odometry_v2/ros/config.hpp"
#include "dr_odometry_v2/ros/converters.hpp"
#include "dr_odometry_v2/ros/map_viz.hpp"

namespace dr_odometry_v2 {

class DrOdometryV2Node {
 public:
  DrOdometryV2Node(ros::NodeHandle& nh, ros::NodeHandle& pnh)
      : nh_(nh), config_(dr_odometry_v2_ros::loadConfig(pnh)), queue_(config_.core.time_sync.queue_size) {
    eskf_.initialize(config_.core);

    imu_sub_ = nh_.subscribe(config_.topics.imu, 2000, &DrOdometryV2Node::onImu, this);
    can_sub_ = nh_.subscribe(config_.topics.can, 200, &DrOdometryV2Node::onCan, this);
    if (config_.core.gnss.enabled) {
      gnss_sub_ = nh_.subscribe(config_.topics.gnss, 100, &DrOdometryV2Node::onGnss, this);
    }
    if (config_.core.localization.enabled) {
      pose_sub_ = nh_.subscribe(config_.topics.localization_pose, 50,
                                &DrOdometryV2Node::onLocalizationPose, this);
      status_sub_ = nh_.subscribe(config_.topics.localization_status, 10,
                                  &DrOdometryV2Node::onLocalizationStatus, this);
    }

    odom_pub_ = nh_.advertise<nav_msgs::Odometry>(config_.topics.odom, 50);
    status_pub_ = nh_.advertise<deeplumin_msgs::LocalizationStatus>(config_.topics.status, 10);
    path_pub_ = nh_.advertise<nav_msgs::Path>(config_.topics.path, 5);
    path_.header.frame_id = config_.frames.odom;

    timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(1.0, config_.core.output.rate)),
                             &DrOdometryV2Node::onTimer, this);

    // ---- 全局地图可视化（仅 RViz 观察 DR 轨迹，算法不消费）----
    map_viz_.setup(nh_, config_.map_viz);

    if (ros::Time::isSimTime()) {
      SYS_LOG(syslog::LogSeverity::WARNING, true)
          << "use_sim_time=true: rosbag play 必须带 --clock（否则定时器不推进、无 odom/path 输出）";
    }

    SYS_LOG(syslog::LogSeverity::INFO, true)
        << "dr_odometry_v2 started imu=" << config_.topics.imu << " can=" << config_.topics.can
        << " gnss=" << config_.topics.gnss << " loc=" << config_.topics.localization_pose
        << " out=" << config_.topics.odom
        << " switches: wheel=" << config_.core.wheel.enabled
        << " gnss_att=" << config_.core.gnss.use_attitude
        << " gnss_acc=" << config_.core.gnss.use_accel
        << " loc_obs=" << config_.core.localization.enabled;
  }

 private:
  // ============================ 订阅：转换 + 入队 ============================

  void onImu(const sensor_msgs::Imu::ConstPtr& msg) {
    ImuSample s = dr_odometry_v2_ros::fromRos(*msg, config_.extrinsics.base_to_imu);
    if (s.stamp <= 0.0) {
      s.stamp = ros::Time::now().toSec();  // 零戳兜底
    }
    std::lock_guard<std::mutex> lock(mutex_);
    queue_.push(Event::makeImu(s));
  }

  void onCan(const deeplumin_msgs::CanReceiveInfo::ConstPtr& msg) {
    WheelSample s = dr_odometry_v2_ros::fromRos(*msg, config_.core.wheel);
    if (s.stamp <= 0.0) {
      s.stamp = ros::Time::now().toSec();
    }
    std::lock_guard<std::mutex> lock(mutex_);
    queue_.push(Event::makeWheel(s));
  }

  void onGnss(const deeplumin_msgs::Gpchc::ConstPtr& msg) {
    GnssSample s = dr_odometry_v2_ros::fromRos(*msg);
    if (s.stamp <= 0.0) {
      s.stamp = ros::Time::now().toSec();
    }
    std::lock_guard<std::mutex> lock(mutex_);
    queue_.push(Event::makeGnss(s));
  }

  void onLocalizationStatus(const deeplumin_msgs::FusionStatus::ConstPtr& msg) {
    const auto info = dr_odometry_v2_ros::fromRos(*msg);
    std::lock_guard<std::mutex> lock(mutex_);
    gate_tracking_ = info.tracking;
    gate_slam_age_ = info.slam_age;
    have_status_ = true;
  }

  void onLocalizationPose(const nav_msgs::Odometry::ConstPtr& msg) {
    // 观测门控（设计 §4.1.1）：status 正常 + SLAM 新鲜（防 DR 模式回喂闭环）
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (config_.core.localization.require_tracking && !(have_status_ && gate_tracking_)) {
        return;
      }
      if (config_.core.localization.require_slam_fresh &&
          gate_slam_age_ > config_.core.localization.slam_age_limit) {
        return;
      }
    }
    PoseSample s = dr_odometry_v2_ros::fromRos(*msg);
    if (s.stamp <= 0.0) {
      s.stamp = ros::Time::now().toSec();
    }
    std::lock_guard<std::mutex> lock(mutex_);
    queue_.push(Event::makePose(s));
  }

  // ============================ 时间序处理 + 发布 ============================

  void onTimer(const ros::TimerEvent&) {
    // drain：按 stamp 升序处理全部待处理事件（时间对齐由 eskf.propagateTo 保证）
    {
      std::lock_guard<std::mutex> lock(mutex_);
      Event event;
      while (queue_.popNext(&event)) {
        eskf_.processEvent(event);
      }
    }

    const double now = ros::Time::now().toSec();
    const OdomResult odom = eskf_.odometry(now);
    if (!odom.valid) {
      return;
    }

    // 去重：外推未生效时（如 bag 回放：消息头是传感器时钟、now 是 bag/墙钟，差值≫
    // max_extrapolate），输出戳=滤波数据时间轴，其更新频率=传感器频率（如 IMU 10Hz），
    // 低于定时器 100Hz——同戳重复发布会触发 TF_REPEATED_DATA，故同戳跳过。
    if (odom.timestamp <= last_output_stamp_ + 1e-9) {
      return;
    }
    if (!clock_mismatch_logged_ && std::abs(now - odom.timestamp) > config_.core.output.max_extrapolate) {
      clock_mismatch_logged_ = true;
      SYS_LOG(syslog::LogSeverity::WARNING, true)
          << "output stamp follows sensor clock (now-stamp=" << (now - odom.timestamp)
          << "s > max_extrapolate=" << config_.core.output.max_extrapolate
          << "), extrapolation disabled; TF/odom rate = sensor rate";
    }
    last_output_stamp_ = odom.timestamp;

    odom_pub_.publish(
        dr_odometry_v2_ros::toRos(odom, config_.frames.odom, config_.frames.base_link));
    if (config_.core.output.publish_tf) {
      tf::Transform transform;
      transform.setOrigin(tf::Vector3(odom.pose.position.x(), odom.pose.position.y(),
                                      odom.pose.position.z()));
      const Quatd q = odom.pose.orientation.normalized();
      transform.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
      tf_broadcaster_.sendTransform(
          tf::StampedTransform(transform, ros::Time(odom.timestamp), config_.frames.odom,
                               config_.frames.base_link));
    }

    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time(odom.timestamp);
    pose.header.frame_id = config_.frames.odom;
    pose.pose.position.x = odom.pose.position.x();
    pose.pose.position.y = odom.pose.position.y();
    pose.pose.position.z = odom.pose.position.z();
    const Quatd q = odom.pose.orientation.normalized();
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();

    // 轨迹：按步长稀疏追加（保持长时间历史），低频发布（累计量无需 100Hz）
    const double step = config_.core.output.path_min_step;
    bool append_pose = path_.poses.empty();
    if (!append_pose) {
      const auto& last = path_.poses.back().pose.position;
      const double moved = std::hypot(pose.pose.position.x - last.x, pose.pose.position.y - last.y);
      const bool timed_out = (odom.timestamp - last_path_stamp_) >= 1.0;  // 原地旋转也留点
      append_pose = (step <= 0.0) || moved >= step || timed_out;
    }
    if (append_pose) {
      path_.poses.push_back(pose);
      last_path_stamp_ = odom.timestamp;
      if (static_cast<int>(path_.poses.size()) > config_.core.output.path_max_size) {
        path_.poses.erase(path_.poses.begin(),
                          path_.poses.begin() + config_.core.output.path_max_size / 5);
      }
    }
    const double path_period = 1.0 / std::max(0.1, config_.core.output.path_publish_rate);
    if (now - last_path_pub_time_ >= path_period) {
      last_path_pub_time_ = now;
      path_.header.stamp = pose.header.stamp;
      path_pub_.publish(path_);
    }

    status_pub_.publish(dr_odometry_v2_ros::toStatusMsg(eskf_.health(now), now));

    // 轻量运行日志（1s 节流）
    if (now - last_log_time_ > 1.0) {
      last_log_time_ = now;
      const auto h = eskf_.health(now);
      SYS_LOG(syslog::LogSeverity::INFO, false)
          << "odom t=" << odom.timestamp << " pos=(" << odom.pose.position.x() << ", "
          << odom.pose.position.y() << ", " << odom.pose.position.z() << ") upd(w/g/p)="
          << h.wheel_update_count << "/" << h.gnss_attitude_update_count << "/"
          << h.pose_update_count << " gate(rej/pend/step)=" << h.pose_reject_count << "/"
          << h.pose_pending_count << "/" << h.pose_step_count
          << " drop(reordered)=" << queue_.reorderedDropped();
    }
  }

  ros::NodeHandle nh_;
  dr_odometry_v2_ros::LoadedConfig config_;
  Eskf eskf_;
  EventQueue queue_;
  std::mutex mutex_;

  ros::Subscriber imu_sub_;
  ros::Subscriber can_sub_;
  ros::Subscriber gnss_sub_;
  ros::Subscriber pose_sub_;
  ros::Subscriber status_sub_;
  ros::Publisher odom_pub_;
  ros::Publisher status_pub_;
  ros::Publisher path_pub_;
  ros::Timer timer_;
  tf::TransformBroadcaster tf_broadcaster_;
  nav_msgs::Path path_;
  dr_odometry_v2_ros::MapVizPublisher map_viz_;

  bool have_status_ = false;
  bool gate_tracking_ = false;
  double gate_slam_age_ = 1e9;
  double last_log_time_ = 0.0;
  double last_path_stamp_ = -1.0;
  double last_path_pub_time_ = 0.0;
  double last_output_stamp_ = -1.0;
  bool clock_mismatch_logged_ = false;
};

}  // namespace dr_odometry_v2

int main(int argc, char** argv) {
  ros::init(argc, argv, "dr_odometry_v2");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::string log_dir;
  pnh.param<std::string>("log_dir", log_dir,
                         "/home/hhy/2004ros/DeepLumin_zq/src/localization/loclog");
  SYS_LOG_INIT_FULL(log_dir, syslog::LogSeverity::INFO, 50, "dr_odometry_v2");

  dr_odometry_v2::DrOdometryV2Node node(nh, pnh);
  ros::spin();
  return 0;
}
