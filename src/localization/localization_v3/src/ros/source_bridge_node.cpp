/**
 * @file source_bridge_node.cpp
 * @brief 局部里程计源桥（SLAM 主 / DR 备）——薄 ROS 适配层
 *
 * 职责（业务逻辑全在 core/local_odom_source，本文件只做话题/点云/时间）：
 *   1. 订阅 SLAM / DR 里程计 → LocalOdomSource 仲裁 → 发布统一源
 *        /localization/odom_source   : 统一输出（单一纯源位姿，100Hz）
 *        /localization/source_anchor : 标定观测锚（SLAM 样本事件式，扫描戳 + 差分速度）
 *   2. 源点云统一：SLAM 在时透传其注册云（必要时 ⊕ glue_slam 进虚拟系）；
 *      DR 时用原始点云按统一源位姿注册 → /localization/source_cloud
 *
 * 观测是硬契约：
 *   观测 = map_odom ⊕ anchor = map_base（GM 校正后的**全局位姿**，零偏学习信号最高），
 *   只在 **SLAM 驱动输出** 时按样本事件发布（DR 驱动期不发）——
 *   anchor 不含 DR 增量、map_odom 是外部信息 ⇒ 观测结构性无自回灌；
 *   DR 接管期输出含 DR 自身增量，绝不能回灌成观测（正反馈，实测大幅抖动）。
 *   【红线】dr 的 localization_pose 必须接本话题，不得直连 fused_odom
 *   （DR 模式下 fused_odom = map_odom ⊕ DR 派生输出 = 自回灌形态）。
 *
 * 对现有系统零改动：fusion / GM / 状态机只换话题输入（launch 一处接线）。
 */

#include <algorithm>
#include <deque>
#include <string>

#include <nav_msgs/Odometry.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <syslog/logger.h>

#include "localization_v3/core/local_odom_source.hpp"
#include "localization_v3/core/types.hpp"

namespace localization_v3 {

namespace {

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

/**
 * @brief 时间基自适应：消息戳与 ROS 时钟同源时用消息戳（精度高），
 * 否则（如 bag 回放：消息头=传感器时钟、now=bag 时钟）退化为接收时间——
 * 保证桥内部新鲜度判断与输出戳在同一时间基上，不因时钟域错位误判"双源皆死"。
 */
double stampOrNow(const ros::Time& stamp) {
  if (!ros::Time::isValid()) {
    return stamp.toSec() > 0.0 ? stamp.toSec() : 0.0;  // 时钟未到：避免 now() 抛异常
  }
  const double now = ros::Time::now().toSec();
  const double s = stamp.toSec();
  return (s > 0.0 && std::abs(s - now) < 5.0) ? s : now;
}

}  // namespace

class SourceBridgeNode {
 public:
  SourceBridgeNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) : nh_(nh) {
    // ---- 参数（config/source_bridge.yaml）----
    LocalOdomSource::Config cfg;
    pnh.param<double>("odometry/slam_timeout_sec", cfg.slam_timeout_sec, cfg.slam_timeout_sec);
    pnh.param<double>("odometry/dr_timeout_sec", cfg.dr_timeout_sec, cfg.dr_timeout_sec);
    pnh.param<int>("odometry/switch_back_confirm_frames", cfg.switch_back_confirm_frames,
                   cfg.switch_back_confirm_frames);
    pnh.param<int>("odometry/flap_guard_count", cfg.flap_guard_count, cfg.flap_guard_count);
    pnh.param<double>("odometry/flap_window_sec", cfg.flap_window_sec, cfg.flap_window_sec);
    pnh.param<bool>("odometry/slam_require_cloud", cfg.slam_require_cloud, cfg.slam_require_cloud);
    pnh.param<double>("odometry/slam_cloud_timeout_sec", cfg.slam_cloud_timeout_sec,
                      cfg.slam_cloud_timeout_sec);

    pnh.param<std::string>("topics/slam_odom", slam_odom_topic_, "/slam_odometry/odom");
    pnh.param<std::string>("topics/dr_odom", dr_odom_topic_, "/dr_odometry/odom");
    pnh.param<std::string>("topics/slam_cloud", slam_cloud_topic_,
                           "/slam_odometry/cloud_effect");
    pnh.param<std::string>("topics/raw_cloud", raw_cloud_topic_, "/ouster/points");
    pnh.param<std::string>("topics/unified_odom", unified_odom_topic_,
                           "/localization/odom_source");
    pnh.param<std::string>("topics/unified_cloud", unified_cloud_topic_,
                           "/localization/source_cloud");
    pnh.param<std::string>("topics/source_anchor", source_anchor_topic_,
                           "/localization/source_anchor");
    pnh.param<std::string>("topics/map_to_odom", map_to_odom_topic_,
                           "/localization/map_to_odom");
    pnh.param<std::string>("frames/odom", odom_frame_, "odom");
    pnh.param<std::string>("frames/map", map_frame_, "map");
    pnh.param<std::string>("frames/base_link", base_frame_, "base_link");

    pnh.param<bool>("cloud/enabled", cloud_enabled_, true);
    // 云处理关闭时无法跟踪云活性 → 源可用判定退化为仅 odom 新鲜
    if (!cloud_enabled_) {
      cfg.slam_require_cloud = false;
    }
    source_.setConfig(cfg);
    std::vector<double> t(3, 0.0), r(9, 0.0);
    r[0] = r[4] = r[8] = 1.0;
    pnh.param<std::vector<double>>("cloud/base_to_lidar/translation", t, t);
    pnh.param<std::vector<double>>("cloud/base_to_lidar/rotation", r, r);
    base_to_lidar_.setIdentity();
    if (t.size() == 3 && r.size() == 9) {
      base_to_lidar_.block<3, 3>(0, 0) << r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7], r[8];
      base_to_lidar_.block<3, 1>(0, 3) << t[0], t[1], t[2];
    }

    pnh.param<double>("output/rate", rate_hz_, 100.0);

    // ---- 订阅 / 发布 ----
    slam_sub_ = nh_.subscribe(slam_odom_topic_, 100, &SourceBridgeNode::onSlamOdom, this);
    dr_sub_ = nh_.subscribe(dr_odom_topic_, 100, &SourceBridgeNode::onDrOdom, this);
    if (cloud_enabled_) {
      slam_cloud_sub_ =
          nh_.subscribe(slam_cloud_topic_, 5, &SourceBridgeNode::onSlamCloud, this);
      raw_cloud_sub_ = nh_.subscribe(raw_cloud_topic_, 5, &SourceBridgeNode::onRawCloud, this);
      unified_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(unified_cloud_topic_, 5);
    }
    unified_odom_pub_ = nh_.advertise<nav_msgs::Odometry>(unified_odom_topic_, 100);
    source_anchor_pub_ = nh_.advertise<nav_msgs::Odometry>(source_anchor_topic_, 100);
    map_to_odom_sub_ =
        nh_.subscribe(map_to_odom_topic_, 10, &SourceBridgeNode::onMapToOdom, this);

    timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(1.0, rate_hz_)),
                             &SourceBridgeNode::onTimer, this);

    SYS_LOG(syslog::LogSeverity::INFO, true)
        << "source_bridge started slam=" << slam_odom_topic_ << " dr=" << dr_odom_topic_
        << " -> unified=" << unified_odom_topic_ << " anchor=" << source_anchor_topic_
        << " cloud=" << unified_cloud_topic_ << " (slam_timeout=" << cfg.slam_timeout_sec
        << "s back_confirm=" << cfg.switch_back_confirm_frames << ")";
  }

 private:
  // ============================ 样本输入 ============================

  void onSlamOdom(const nav_msgs::Odometry::ConstPtr& msg) {
    const double stamp = stampOrNow(msg->header.stamp);
    source_.onSlam(poseFromRos(msg->pose.pose), stamp);
    slam_twist_ = msg->twist;  // body 系速度，glue 不变 → 直接透传
    publishAnchor(stamp);
  }

  void onDrOdom(const nav_msgs::Odometry::ConstPtr& msg) {
    source_.onDr(poseFromRos(msg->pose.pose), stampOrNow(msg->header.stamp));
    dr_twist_ = msg->twist;
  }

  /// map→odom 修正（fusion/GM 写入，缓变量）：观测合成 map_base 用
  void onMapToOdom(const nav_msgs::Odometry::ConstPtr& msg) {
    map_odom_ = poseFromRos(msg->pose.pose);
    have_map_odom_ = true;
  }

  /**
   * @brief 标定观测：SLAM 样本事件式发布（每个新样本一次），只在 SLAM 驱动输出时发。
   *
   * 观测值 z = map_odom ⊕ anchor = map_base（GM 校正后的全局位姿）：
   * 早期方案 c 的量测语义——DR 世界系锚定 map 系，零偏学习信号质量最高；
   * anchor 不含 DR 增量、map_odom 为外部信息 ⇒ 结构性无自回灌。
   * DR 驱动期不发（kNone/kDr 一律不发）。
   *
   * 【红线】map_odom 未建立时**不发观测**：纯锚在虚拟系、map_base 在 map 系，
   * 二者是不同坐标帧（车辆不在原点时数值差一个 map→odom 偏移，实测 ~12m）。
   * 早期"退化纯锚"分支曾让首帧观测抢跑 map_to_odom 建立 → DR 地图基准被永久
   * 钉在错误帧 → 之后每条观测恒差 ~12m 超 hard 档永久拒收（2026-10-13 实测 run6）。
   * 帧不一致的观测宁可不发：无观测期 DR 纯递推 + 零偏冻结，由现有机制覆盖。
   *
   * 时间戳 = **扫描戳**（sample_stamp）：消 v·latency 错位（发布戳带一个 LIO 处理
   * 延迟，观测会被 ESKF 拉到错误时刻比对）。迟到由 DR 侧兜底：事件队列
   * pose_late_tolerance 放宽位姿迟到容忍 + observePose 一阶外推 z += v·Δt。
   *
   * twist = **锚位姿差分的真实速度**（body 系，⓪'）：连续锚样本位置差分 / 时间差，
   * 旋到 body 系；只差分 anchor（不含 map_odom），GM 修正台阶不会注入假速度。
   * 首个样本无差分基准 → 保持全零（DR 侧 has_velocity 仍 true，仅一帧影响可忽略）；
   * 差分间隔异常（换源后首帧等）同样保持上次速度（ZOH）。
   */
  void publishAnchor(double sample_stamp) {
    if (source_.active() != LocalOdomSource::Source::kSlam) {
      have_last_anchor_ = false;  // 换源后差分基准作废
      return;  // 仅 SLAM 驱动输出期发观测（kNone/kDr 一律不发）
    }
    if (!have_map_odom_) {
      return;  // 红线：map 系未建立 → 不发（纯锚与 map_base 不同帧，禁止退化）
    }
    Pose3d anchor;
    if (!source_.purePose(&anchor)) {
      return;
    }
    // ⓪' 差分真实速度：v_body = R(anchor)ᵀ · Δanchor/Δt（body 系，与 twist 契约一致）
    if (have_last_anchor_) {
      const double dt = sample_stamp - last_anchor_stamp_;
      if (dt > kFdMinDt && dt < kFdMaxDt) {
        const Vec3d v_body =
            anchor.orientation.conjugate() * ((anchor.position - last_anchor_.position) / dt);
        if (isFinite(v_body)) {
          anchor_twist_.twist.linear.x = v_body.x();
          anchor_twist_.twist.linear.y = v_body.y();
          anchor_twist_.twist.linear.z = v_body.z();
        }
      }
      // dt 异常（过近重复戳/过大间隙）→ 保持上次速度（ZOH），不发全零假"停车"
    }
    last_anchor_ = anchor;
    last_anchor_stamp_ = sample_stamp;
    have_last_anchor_ = true;
    const Pose3d z = compose(map_odom_, anchor);
    publishOdom(source_anchor_pub_, z, ros::Time(sample_stamp), anchor_twist_, map_frame_);
  }

  // ============================ 源点云统一 ============================

  void onSlamCloud(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    if (!cloud_enabled_) {
      return;
    }
    // 先喂云活性（无论当前活跃源是谁）：SLAM 云断流属于源不可用的一部分
    source_.onSlamCloud(stampOrNow(msg->header.stamp));
    if (source_.active() != LocalOdomSource::Source::kSlam) {
      return;
    }
    // 契约：桥输出的云必须与统一源同处【虚拟系】。
    // glue=I（从未发生 DR 接管/回切）：零成本原样透传；
    // glue≠I（发生过 DR→SLAM 回切）：SLAM 云在 SLAM-odom 系，须乘 glue_slam 进虚拟系——
    // 否则 map_odom（=map→云端系）与统一源（虚拟系）混乘，path/TF 偏移而点云看似正常。
    const Pose3d& glue = source_.slamGlue();
    if (isIdentity(glue)) {
      unified_cloud_pub_.publish(*msg);
      return;
    }
    pcl::PointCloud<pcl::PointXYZI> in, out;
    pcl::fromROSMsg(*msg, in);
    if (in.empty()) {
      return;
    }
    pcl::transformPointCloud(in, out, toMatrix4f(glue));
    sensor_msgs::PointCloud2 out_msg;
    pcl::toROSMsg(out, out_msg);
    out_msg.header = msg->header;  // 保持源时间戳
    out_msg.header.frame_id = odom_frame_;
    unified_cloud_pub_.publish(out_msg);
  }

  void onRawCloud(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    if (!cloud_enabled_) {
      return;
    }
    // DR 模式：原始点云按统一源位姿注册到虚拟局部系（GM 的源点云）
    if (source_.active() != LocalOdomSource::Source::kDr) {
      return;
    }
    const Pose3d* base = poseAt(stampOrNow(msg->header.stamp));
    if (base == nullptr) {
      return;
    }
    // T_unified_lidar = T_unified_base ⊕ T_base_lidar
    const Eigen::Matrix4f T = toMatrix4f(compose(*base, Pose3dFromMatrix(base_to_lidar_)));
    pcl::PointCloud<pcl::PointXYZI> in, out;
    pcl::fromROSMsg(*msg, in);
    if (in.empty()) {
      return;
    }
    pcl::transformPointCloud(in, out, T);
    pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
    voxel_filter.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>(out));
    voxel_filter.setLeafSize(0.2, 0.2, 0.2);
    voxel_filter.filter(out);

    sensor_msgs::PointCloud2 out_msg;
    pcl::toROSMsg(out, out_msg);
    out_msg.header = msg->header;  // 保持源时间戳
    out_msg.header.frame_id = odom_frame_;
    unified_cloud_pub_.publish(out_msg);
  }

  // ============================ 统一源输出 ============================

  void onTimer(const ros::TimerEvent&) {
    const double now = ros::Time::now().toSec();
    Pose3d motion;
    if (!source_.sample(&motion, now)) {
      return;  // 双源皆失：停发（fusion 层看门狗判失效）
    }

    history_.push_back({now, motion});
    while (history_.size() > 256) {  // 100Hz × 2.5s
      history_.pop_front();
    }

    const ros::Time stamp = ros::Time::now();
    publishOdom(unified_odom_pub_, motion, stamp, activeTwist(), odom_frame_);

    // 状态变化日志
    const auto snap = source_.snapshot(now);
    if (snap.active != last_logged_source_) {
      last_logged_source_ = snap.active;
      SYS_LOG(syslog::LogSeverity::WARNING, true)
          << "odometry source switch -> "
          << (snap.active == LocalOdomSource::Source::kSlam
                  ? "SLAM"
                  : (snap.active == LocalOdomSource::Source::kDr ? "DR" : "NONE"))
          << " (switch#" << snap.switch_count << ")";
    }
    if (snap.flap_guard && !flap_warned_) {
      flap_warned_ = true;
      SYS_LOG(syslog::LogSeverity::WARNING, true)
          << "odometry source flapping (" << snap.switch_count
          << " switches in window) -> switch-back confirm doubled";
    }
    if (now - last_log_time_ > 5.0) {
      last_log_time_ = now;
      SYS_LOG(syslog::LogSeverity::INFO, false)
          << "source=" << (snap.active == LocalOdomSource::Source::kSlam ? "SLAM" : "DR")
          << " slam_age=" << snap.slam_age << " dr_age=" << snap.dr_age
          << " switches=" << snap.switch_count;
    }
  }

  // ============================ 辅助 ============================

  struct TimedPose {
    double stamp;
    Pose3d pose;
  };

  /// 取扫描时刻的统一源位姿（100Hz 历史取最近样本；容差收紧到 0.2s）
  const Pose3d* poseAt(double stamp) const {
    const TimedPose* best = nullptr;
    double best_dt = 1e9;
    for (const auto& p : history_) {
      const double dt = std::abs(p.stamp - stamp);
      if (dt < best_dt) {
        best_dt = dt;
        best = &p;
      }
    }
    return (best != nullptr && best_dt < 0.2) ? &best->pose : nullptr;
  }

  /// 当前活跃源的 body 系速度（SLAM 里程计通常不填 twist → 全零，语义与接入前一致）
  const geometry_msgs::TwistWithCovariance& activeTwist() const {
    return source_.active() == LocalOdomSource::Source::kDr ? dr_twist_ : slam_twist_;
  }

  void publishOdom(ros::Publisher& pub, const Pose3d& pose, const ros::Time& stamp,
                   const geometry_msgs::TwistWithCovariance& twist,
                   const std::string& frame) const {
    nav_msgs::Odometry msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame;
    msg.child_frame_id = base_frame_;
    fillPose(pose, &msg.pose.pose);
    msg.twist = twist;  // body 系，glue 不变 → 原样透传
    pub.publish(msg);
  }

  static Eigen::Matrix4d toMatrix4d(const Pose3d& pose) {
    Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
    m.block<3, 3>(0, 0) = pose.orientation.normalized().toRotationMatrix();
    m.block<3, 1>(0, 3) = pose.position;
    return m;
  }
  static Eigen::Matrix4f toMatrix4f(const Pose3d& pose) {
    return toMatrix4d(pose).cast<float>();
  }
  static Pose3d Pose3dFromMatrix(const Eigen::Matrix4d& m) {
    Pose3d p;
    p.position = m.block<3, 1>(0, 3);
    p.orientation = Quatd(m.block<3, 3>(0, 0)).normalized();
    return p;
  }

  ros::NodeHandle nh_;
  LocalOdomSource source_;

  std::string slam_odom_topic_, dr_odom_topic_, slam_cloud_topic_, raw_cloud_topic_;
  std::string unified_odom_topic_, unified_cloud_topic_, source_anchor_topic_;
  std::string map_to_odom_topic_;
  std::string odom_frame_, map_frame_, base_frame_;
  bool cloud_enabled_ = true;
  Eigen::Matrix4d base_to_lidar_ = Eigen::Matrix4d::Identity();
  double rate_hz_ = 100.0;

  ros::Subscriber slam_sub_;
  ros::Subscriber dr_sub_;
  ros::Subscriber slam_cloud_sub_;
  ros::Subscriber raw_cloud_sub_;
  ros::Subscriber map_to_odom_sub_;
  ros::Publisher unified_odom_pub_;
  ros::Publisher source_anchor_pub_;
  ros::Publisher unified_cloud_pub_;
  ros::Timer timer_;

  geometry_msgs::TwistWithCovariance slam_twist_;
  geometry_msgs::TwistWithCovariance dr_twist_;
  // ⓪' 观测 twist 差分基准（anchor 本体系速度）
  static constexpr double kFdMinDt = 1e-3;  ///< 差分间隔下限（s）：防重复戳除小数
  static constexpr double kFdMaxDt = 0.5;   ///< 差分间隔上限（s）：间隙过大差分无意义 → ZOH 保持
  Pose3d last_anchor_;
  double last_anchor_stamp_ = 0.0;
  bool have_last_anchor_ = false;
  geometry_msgs::TwistWithCovariance anchor_twist_;  ///< 观测 twist（差分速度，ZOH 保持）
  Pose3d map_odom_;  ///< map→odom 修正（fusion 发布，缓变量）
  bool have_map_odom_ = false;
  std::deque<TimedPose> history_;
  LocalOdomSource::Source last_logged_source_ = LocalOdomSource::Source::kNone;
  bool flap_warned_ = false;
  double last_log_time_ = 0.0;
};

}  // namespace localization_v3

int main(int argc, char** argv) {
  ros::init(argc, argv, "source_bridge");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::string log_dir;
  pnh.param<std::string>("log_dir", log_dir,
                         "/home/hhy/2004ros/DeepLumin_zq/src/localization/loclog");
  SYS_LOG_INIT_FULL(log_dir, syslog::LogSeverity::INFO, 50, "source_bridge");

  // sim time 模式：必须等 /clock 有效后再建定时器（否则时钟到位后定时器不触发）
  if (ros::Time::isSimTime() && !ros::Time::isValid()) {
    SYS_LOG(syslog::LogSeverity::WARNING, true)
        << "use_sim_time=true: waiting for /clock (rosbag play 需带 --clock) ...";
    // 墙钟轮询：ros::Time::now()/waitForValid 在时钟无效时可能抛异常，不可用
    while (ros::ok() && !ros::Time::isValid()) {
      ros::WallDuration(0.05).sleep();
    }
    SYS_LOG(syslog::LogSeverity::INFO, true) << "/clock ready";
  }

  localization_v3::SourceBridgeNode node(nh, pnh);
  ros::spin();
  return 0;
}
