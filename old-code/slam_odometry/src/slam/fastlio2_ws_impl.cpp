#include "localization/slam/fastlio2_ws_impl.hpp"

#include "localization/factory/factory_registry.hpp"

#include <laser_mapping.h>

#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include <cstring>
#include <memory>

namespace localization {

class FastLio2WsImpl::Impl {
 public:
  ~Impl() {
    if (mapping_) {
      mapping_->Finish();
    }
  }

  bool initialize(const SlamConfig& cfg) {
    config_ = cfg;
    if (cfg.faster_lio_config_yaml.empty()) {
      ROS_ERROR("[fastlio2] slam/faster_lio/config_yaml is empty, cannot init faster-lio");
      return false;
    }
    mapping_ = std::make_unique<faster_lio::LaserMapping>();
    if (!mapping_->InitWithoutROS(cfg.faster_lio_config_yaml)) {
      ROS_ERROR("[fastlio2] faster-lio InitWithoutROS failed: %s", cfg.faster_lio_config_yaml.c_str());
      mapping_.reset();
      return false;
    }
    // 用 slam_odometry 的配置覆盖核心参数（单一开关源）
    mapping_->SetVelocityGnssParams(
        cfg.use_vel, cfg.use_gnss_attitude, cfg.can_vel_diff_thresh_kmh,
        cfg.can_timeout_sec, cfg.gnss_attitude_timeout_sec,
        cfg.attitude_diff_thresh_deg, cfg.attitude_correction_gain);
    inited_ = true;
    return true;
  }

  void feedImu(const ImuData& imu) {
    if (!mapping_) {
      return;
    }
    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu());
    msg->header.stamp = ros::Time(imu.timestamp);
    msg->angular_velocity.x = imu.gyro.x();
    msg->angular_velocity.y = imu.gyro.y();
    msg->angular_velocity.z = imu.gyro.z();
    msg->linear_acceleration.x = imu.accel.x();
    msg->linear_acceleration.y = imu.accel.y();
    msg->linear_acceleration.z = imu.accel.z();
    mapping_->IMUCallBack(msg);
  }

  void feedLidar(const TimestampedPointCloud& pc) {
    if (!mapping_ || !pc.cloud || pc.cloud->empty()) {
      return;
    }
    // faster-lio 的 Ouster 预处理（Oust64Handler）要求点云带 't'（逐点时间，uint32 纳秒）字段：
    //   curvature(ms) = t(ns) / 1e6
    // 接口传入的 PointXYZINormal 云把逐点时间存在 curvature(ms) 里，pcl::toROSMsg
    // 不会生成 't' 字段，导致时间信息丢失、去畸变/建图异常（Map grid 卡在 1）。
    // 这里手工重建 Ouster 字段布局：x,y,z,intensity(float32) + t(uint32 纳秒)，
    // t = curvature * 1e6，正好还原 faster-lio 期望的输入。
    sensor_msgs::PointCloud2::Ptr msg(new sensor_msgs::PointCloud2());
    msg->header.stamp = ros::Time(pc.timestamp);
    msg->header.frame_id = "livox_frame";
    msg->height = 1;
    msg->width = pc.cloud->size();
    msg->is_dense = true;
    msg->point_step = 20;  // 4 * FLOAT32 + 1 * UINT32
    msg->row_step = msg->point_step * msg->width;

    sensor_msgs::PointField field;
    field.count = 1;
    field.datatype = sensor_msgs::PointField::FLOAT32;
    for (const char* name : {"x", "y", "z", "intensity"}) {
      field.name = name;
      field.offset = static_cast<uint32_t>(msg->fields.size() * 4);
      msg->fields.push_back(field);
    }
    field.name = "t";
    field.datatype = sensor_msgs::PointField::UINT32;
    field.offset = static_cast<uint32_t>(msg->fields.size() * 4);
    msg->fields.push_back(field);

    msg->data.resize(msg->row_step);
    const auto& pts = pc.cloud->points;
    for (size_t i = 0; i < pts.size(); ++i) {
      float* base = reinterpret_cast<float*>(msg->data.data() + i * 20);
      base[0] = pts[i].x;
      base[1] = pts[i].y;
      base[2] = pts[i].z;
      base[3] = pts[i].intensity;
      const uint32_t t_ns = static_cast<uint32_t>(pts[i].curvature * 1e6f);
      std::memcpy(msg->data.data() + i * 20 + 16, &t_ns, sizeof(t_ns));
    }

    last_lidar_ts_ = pc.timestamp;
    mapping_->StandardPCLCallBack(msg);
  }

  bool processOnce() {
    return mapping_ ? mapping_->Run() : false;
  }

  OdomResult getOdometry() const {
    OdomResult out;
    if (!mapping_) {
      return out;
    }
    nav_msgs::Odometry odom;
    if (!mapping_->GetOdometry(odom)) {
      return out;
    }
    out.timestamp = odom.header.stamp.toSec();
    out.position << odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z;
    out.orientation = Quatd(odom.pose.pose.orientation.w, odom.pose.pose.orientation.x,
                            odom.pose.pose.orientation.y, odom.pose.pose.orientation.z);
    out.valid = true;
    out.frame_id = "odom";
    return out;
  }

  TimestampedPointCloud getCurrentScan() const {
    TimestampedPointCloud out;
    if (!mapping_) {
      return out;
    }
    localization::PointCloudPtr cloud;
    // faster-lio 的 scan_down_body_ 在 lidar 系；GetScanInImu 已用 lidar->IMU 外参
    // 转到 IMU/body 系，节点按 identity base_to_imu/base_to_lidar 变换输出即坐标系自洽
    if (mapping_->GetScanInImu(cloud)) {
      out.cloud = cloud;
      out.timestamp = last_lidar_ts_;
      out.frame_id = "imu_link";
    }
    return out;
  }

  TimestampedPointCloud getEffectScan() const {
    TimestampedPointCloud out;
    if (!mapping_) {
      return out;
    }
    // faster-lio 的 corr_pts_ 在 lidar 系；GetEffectInImu 用 lidar->IMU 外参转到 IMU/body 系，
    // 与 getCurrentScan 的坐标系一致，节点按 identity base_to_imu/base_to_lidar 变换输出。
    localization::PointCloudPtr cloud;
    if (mapping_->GetEffectInImu(cloud)) {
      out.cloud = cloud;
      out.timestamp = last_lidar_ts_;
      out.frame_id = "imu_link";
    }
    return out;
  }

  TimestampedPointCloud getLocalMap() const {
    TimestampedPointCloud out;
    if (!mapping_) {
      return out;
    }
    localization::PointCloudPtr cloud;
    if (mapping_->GetLocalMap(cloud)) {
      out.cloud = cloud;
      out.timestamp = last_lidar_ts_;
      out.frame_id = "odom";
    }
    return out;
  }

  void reset() {
    if (!config_.faster_lio_config_yaml.empty()) {
      mapping_.reset();
      inited_ = false;
      initialize(config_);
    }
  }

  void feedImuRaw(const sensor_msgs::Imu& msg) {
    if (!mapping_) {
      return;
    }
    if (!imu_logged_) {
      ROS_INFO("[fastlio2] raw imu stamp=%.3f acc=(%.4f %.4f %.4f) gyr=(%.6f %.6f %.6f)",
               msg.header.stamp.toSec(), msg.linear_acceleration.x, msg.linear_acceleration.y,
               msg.linear_acceleration.z, msg.angular_velocity.x, msg.angular_velocity.y,
               msg.angular_velocity.z);
      imu_logged_ = true;
    }
    sensor_msgs::Imu::Ptr p(new sensor_msgs::Imu(msg));
    mapping_->IMUCallBack(p);
  }

  void feedLidarRaw(const sensor_msgs::PointCloud2& msg) {
    if (!mapping_) {
      return;
    }
    if (!raw_logged_) {
      std::string names;
      for (const auto& f : msg.fields) {
        names += f.name;
        names += ",";
      }
      ROS_INFO("[fastlio2] raw lidar fields=[%s] width=%u point_step=%u stamp=%.3f",
               names.c_str(), msg.width, msg.point_step, msg.header.stamp.toSec());
      raw_logged_ = true;
    }
    last_lidar_ts_ = msg.header.stamp.toSec();
    sensor_msgs::PointCloud2::Ptr p(new sensor_msgs::PointCloud2(msg));
    mapping_->StandardPCLCallBack(p);
  }

  void feedCanVelocity(const localization::CanVelocityData &can) {
    if (!mapping_) {
      return;
    }
    faster_lio::LaserMapping::CanVelocityData c;
    c.timestamp = can.timestamp;
    c.speed_kmh = can.speed_kmh;
    c.valid = can.valid;
    mapping_->FeedCanVelocity(c);
  }

  void feedGnssAttitude(const localization::GnssAttitudeData &gnss) {
    if (!mapping_) {
      return;
    }
    faster_lio::LaserMapping::GnssAttitudeData g;
    g.timestamp = gnss.timestamp;
    g.heading_deg = gnss.heading_deg;
    g.pitch_deg = gnss.pitch_deg;
    g.roll_deg = gnss.roll_deg;
    g.valid = gnss.valid;
    mapping_->FeedGnssAttitude(g);
  }

 private:
  SlamConfig config_;
  std::unique_ptr<faster_lio::LaserMapping> mapping_;
  double last_lidar_ts_ = 0.0;
  bool inited_ = false;
  bool raw_logged_ = false;
  bool imu_logged_ = false;
};

bool FastLio2WsImpl::initialize(const SlamConfig& cfg) {
  impl_ = std::make_unique<Impl>();
  return impl_->initialize(cfg);
}

void FastLio2WsImpl::feedImu(const ImuData& imu) { impl_->feedImu(imu); }
void FastLio2WsImpl::feedLidar(const TimestampedPointCloud& pc) { impl_->feedLidar(pc); }
void FastLio2WsImpl::feedImuRaw(const sensor_msgs::Imu& msg) { impl_->feedImuRaw(msg); }
void FastLio2WsImpl::feedLidarRaw(const sensor_msgs::PointCloud2& msg) { impl_->feedLidarRaw(msg); }
void FastLio2WsImpl::feedCanVelocity(const CanVelocityData& can) {
  if (impl_) {
    impl_->feedCanVelocity(can);
  }
}
void FastLio2WsImpl::feedGnssAttitude(const GnssAttitudeData& gnss) {
  if (impl_) {
    impl_->feedGnssAttitude(gnss);
  }
}
void FastLio2WsImpl::feedDynamicMask(const TimestampedPointCloud&) {}
bool FastLio2WsImpl::processOnce() { return impl_ && impl_->processOnce(); }
OdomResult FastLio2WsImpl::getOdometry() const { return impl_ ? impl_->getOdometry() : OdomResult(); }
TimestampedPointCloud FastLio2WsImpl::getLocalMap() const { return impl_ ? impl_->getLocalMap() : TimestampedPointCloud(); }
TimestampedPointCloud FastLio2WsImpl::getCurrentScan() const { return impl_ ? impl_->getCurrentScan() : TimestampedPointCloud(); }
TimestampedPointCloud FastLio2WsImpl::getEffectScan() const { return impl_ ? impl_->getEffectScan() : TimestampedPointCloud(); }
bool FastLio2WsImpl::isDegenerate() const { return false; }
double FastLio2WsImpl::getDegeneracyFactor() const { return 0.0; }
void FastLio2WsImpl::reset() {
  if (impl_) {
    impl_->reset();
  }
}

void registerFastLio2Plugin() {
  SlamFactory::instance().registerCreator(
      "fastlio2", []() { return std::make_shared<FastLio2WsImpl>(); });
}

}  // namespace localization
