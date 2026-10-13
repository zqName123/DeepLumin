#include "lio_odometry/core/sensor_health.h"

#include <cmath>
#include <sstream>

namespace slam_odometry {

// ============================================================================
// SensorHealthMonitor
// ============================================================================

void SensorHealthMonitor::Configure(bool use_wheel, bool use_gnss, const Options& opt) {
    opt_ = opt;
    // LiDAR/IMU 是硬依赖，无条件视为已配置；轮速/GNSS 是软依赖，受配置开关控制
    imu_.configured = true;
    lidar_.configured = true;
    wheel_.configured = use_wheel;
    gnss_.configured = use_gnss;
}

void SensorHealthMonitor::Touch(SensorStatus& st, double stamp) {
    if (!std::isfinite(stamp) || stamp < 0.0) {
        return;  // 非法时间戳不登记：宁可显示"无数据"，也不能让错误时基污染判定
    }
    st.ever_received = true;
    st.last_stamp = stamp;
    ++st.received;
}

void SensorHealthMonitor::OnImu(double stamp) { Touch(imu_, stamp); }
void SensorHealthMonitor::OnLidar(double stamp) { Touch(lidar_, stamp); }
void SensorHealthMonitor::OnWheel(double stamp) { Touch(wheel_, stamp); }

void SensorHealthMonitor::OnGnss(double stamp, int status, int nsv) {
    Touch(gnss_, stamp);
    // status/nsv 仅透传供诊断；本次不参与 usable() 判定，避免改变算法输出
    gnss_.quality_status = status;
    gnss_.quality_nsv = nsv;
}

void SensorHealthMonitor::Refresh(SensorStatus& st, double t_ref, double max_age, double max_clock_skew) {
    if (!st.ever_received) {
        st.age = 1e9;
        st.fresh = false;
        return;
    }
    st.age = t_ref - st.last_stamp;
    // age < 0（数据比当前帧新）是实时流水线的正常表现：last_stamp 天然领先
    // lidar_end_time_ 一个流水线延迟。只有"太老"（age > max_age）或"超前离谱"
    // （age < -max_clock_skew，说明时基不同源）才判为不新鲜。
    st.fresh = (st.age <= max_age) && (st.age >= -max_clock_skew);
    if (!st.fresh) {
        ++st.stale_hits;
    }
}

SensorHealthMonitor::Snapshot SensorHealthMonitor::Evaluate(double t_ref) {
    Refresh(imu_, t_ref, opt_.imu_max_age_sec, opt_.max_clock_skew_sec);
    Refresh(lidar_, t_ref, opt_.lidar_max_age_sec, opt_.max_clock_skew_sec);
    Refresh(wheel_, t_ref, opt_.wheel_max_age_sec, opt_.max_clock_skew_sec);
    Refresh(gnss_, t_ref, opt_.gnss_max_age_sec, opt_.max_clock_skew_sec);

    Snapshot s;
    s.imu = imu_;
    s.lidar = lidar_;
    s.wheel = wheel_;
    s.gnss = gnss_;
    return s;
}

std::string SensorHealthMonitor::DebugString(const Snapshot& s) const {
    auto one = [](const char* name, const SensorStatus& st) {
        std::ostringstream os;
        os << name << "[cfg=" << (st.configured ? 1 : 0)
           << " got=" << (st.ever_received ? 1 : 0)
           << " age=" << st.age
           << " use=" << (st.usable() ? 1 : 0) << "]";
        return os.str();
    };
    std::ostringstream os;
    os << one("imu", s.imu) << " " << one("lidar", s.lidar) << " "
       << one("wheel", s.wheel) << " " << one("gnss", s.gnss);
    return os.str();
}

// ============================================================================
// StationaryDetector
// ============================================================================

void StationaryDetector::Configure(const Options& opt) { opt_ = opt; }

void StationaryDetector::Reset() {
    hold_ = 0;
    confirmed_ = false;
    resume_hold_ = 0;
}

bool StationaryDetector::Update(double speed) {
    // 滞回：低于 min 累积，高于 resume 清零，介于两者之间保持不变
    if (speed < opt_.min_speed_mps) {
        ++hold_;
    } else if (speed > opt_.resume_speed_mps) {
        hold_ = 0;
    }

    confirmed_ = (hold_ >= opt_.confirm_frames);
    if (confirmed_) {
        // 静止期间持续刷新尾帧计数；解除后的第一帧由它继承，再逐帧递减
        resume_hold_ = opt_.resume_hold_frames;
    }
    return confirmed_;
}

void StationaryDetector::TickResumeHold() {
    if (resume_hold_ > 0) {
        --resume_hold_;
    }
}

}  // namespace slam_odometry
