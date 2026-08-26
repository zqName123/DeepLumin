/**
 * @file dr_eskf.cpp
 * @brief DrEskf 实现：IMU 预测、轮速/GNSS 更新、ENU 坐标转换。
 */

#include "dr_odometry/core/dr_eskf.hpp"
#include "dr_odometry/core/math_utils.hpp"

#include <algorithm>
#include <cmath>

namespace dr_odometry {
namespace {
constexpr double kDegToRad = M_PI / 180.0;
// WGS84 长半轴与第一偏心率平方（用于 LLA↔ECEF）
constexpr double kEarthA = 6378137.0;
constexpr double kEarthE2 = 6.69437999014e-3;

bool finiteVec(const Vec3d& v) {
  return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
}

template <typename Derived>
bool finiteEigen(const Eigen::MatrixBase<Derived>& m) {
  for (Eigen::Index r = 0; r < m.rows(); ++r) {
    for (Eigen::Index c = 0; c < m.cols(); ++c) {
      if (!std::isfinite(m(r, c))) {
        return false;
      }
    }
  }
  return true;
}
}  // namespace

bool DrEskf::initialize(const DrConfig& config) {
  config_ = config;
  state_ = DrState();
  state_.position = config.initial_position;
  state_.orientation = math::quatFromYaw(config.initial_yaw_deg * kDegToRad);
  state_.gyro_bias = config.initial_gyro_bias;
  state_.accel_bias = config.initial_accel_bias;
  // 初始协方差块对角标度：位置/速度较松，姿态适中，零偏较紧
  state_.covariance.setIdentity();
  state_.covariance.block<3, 3>(0, 0) *= 1.0;   // p
  state_.covariance.block<3, 3>(3, 3) *= 1.0;   // v
  state_.covariance.block<3, 3>(6, 6) *= 0.1;  // θ
  state_.covariance.block<3, 3>(9, 9) *= 0.01; // bg
  state_.covariance.block<3, 3>(12, 12) *= 0.05; // ba
  has_last_imu_ = false;
  has_can_ = false;
  has_gnss_ = false;
  imu_count_ = 0;
  can_count_ = 0;
  gnss_count_ = 0;
  predict_count_ = 0;
  wheel_update_count_ = 0;
  gnss_heading_update_count_ = 0;
  gnss_velocity_update_count_ = 0;
  gnss_position_update_count_ = 0;
  invalid_imu_count_ = 0;
  nonpositive_imu_dt_count_ = 0;
  stale_wheel_count_ = 0;
  stale_gnss_count_ = 0;
  origin_set_ = false;
  return true;
}

void DrEskf::reset(const DrState& state) {
  state_ = state;
  state_.orientation.normalize();
  has_last_imu_ = false;  // 下一帧 IMU 只对齐时间，不跨未知 dt 预测
}

void DrEskf::initializeFromImu(const ImuData& imu) {
  state_.timestamp = imu.timestamp;
  state_.initialized = true;
  last_imu_ = imu;
  has_last_imu_ = true;
}

void DrEskf::feedImu(const ImuData& imu) {
  ++imu_count_;
  if (!std::isfinite(imu.timestamp) || !finiteVec(imu.gyro) || !finiteVec(imu.accel)) {
    ++invalid_imu_count_;
    return;
  }
  // 首帧：只初始化，不做预测（尚无上一帧算 dt）
  if (!state_.initialized) {
    initializeFromImu(imu);
    return;
  }
  if (!has_last_imu_) {
    last_imu_ = imu;
    has_last_imu_ = true;
    return;
  }

  double dt = imu.timestamp - last_imu_.timestamp;
  if (dt <= 0.0) {
    // 乱序或重复时间戳：刷新缓存，跳过本次
    ++nonpositive_imu_dt_count_;
    last_imu_ = imu;
    return;
  }
  // 断流保护：限制单步最大积分时间
  dt = std::min(dt, config_.max_dt);

  // 中值 IMU，降低梯形积分误差
  ImuData mid;
  mid.timestamp = imu.timestamp;
  mid.gyro = 0.5 * (last_imu_.gyro + imu.gyro);
  mid.accel = 0.5 * (last_imu_.accel + imu.accel);
  predict(mid, dt);  // 利用imu进行递推
  ++predict_count_;
  last_imu_ = imu;

  // 在 IMU 时刻上做异步观测更新（轮速/GNSS 需通过新鲜度检查）
  if (config_.use_wheel) {
    correctWheel(imu.timestamp);
  }
  if (config_.use_gnss && has_gnss_) {
    if (gnssFresh(imu.timestamp, last_gnss_)) {
      correctGnss(last_gnss_);
    } else {
      ++stale_gnss_count_;
    }
  }
}

void DrEskf::feedCan(const CanData& can) {
  ++can_count_;
  if (!std::isfinite(can.timestamp) || !std::isfinite(can.speed_mps)) {
    return;
  }
  last_can_ = can;
  has_can_ = can.valid;
}

void DrEskf::feedGnss(const GnssData& gnss) {
  ++gnss_count_;
  if (!std::isfinite(gnss.timestamp)) {
    return;
  }
  last_gnss_ = gnss;
  has_gnss_ = true;

  // 首次有效位置：固定局部 ENU 原点（后续位置观测相对该原点）
  if (!origin_set_ && gnss.position_valid) {
    origin_lat_rad_ = gnss.latitude * kDegToRad;
    origin_lon_rad_ = gnss.longitude * kDegToRad;
    origin_alt_ = gnss.altitude;
    origin_ecef_ = llaToEcef(gnss.latitude, gnss.longitude, gnss.altitude);
    origin_set_ = true;
  }

  // 滤波尚未 IMU 初始化时，可用有效航向预置 yaw，减少启动航向漂移
  if (!state_.initialized && config_.use_gnss_heading && gnss.heading_valid) {
    state_.orientation = math::quatFromYaw(gnss.heading_rad);
  }
}

void DrEskf::predict(const ImuData& imu, double dt) {
  const Vec3d omega = imu.gyro - state_.gyro_bias;  // 减去零偏
  const Vec3d accel_body = imu.accel - state_.accel_bias;
  const Mat3d Rwb = state_.orientation.toRotationMatrix(); // body -> world
  const Vec3d gravity(0.0, 0.0, -config_.gravity);

  // 世界系加速度；关闭 use_imu_accel 时加速度项为零，靠轮速/GNSS 约束速度
  Vec3d accel_world = Vec3d::Zero();
  if (config_.use_imu_accel) {
    accel_world = Rwb * accel_body + gravity;
  }

  // 标称状态积分
  state_.position += state_.velocity * dt + 0.5 * accel_world * dt * dt;
  state_.velocity += accel_world * dt;
  state_.orientation = (state_.orientation * math::deltaQuat(omega, dt)).normalized();
  state_.timestamp = imu.timestamp;

  // 误差状态连续模型离散化：F ≈ I + F_c·dt
  // δp' = δv
  // δv' = -R[a]_× δθ - R δba   （仅 use_imu_accel）
  // δθ' = -δbg
  Eigen::Matrix<double, 15, 15> F = Eigen::Matrix<double, 15, 15>::Identity();
  F.block<3, 3>(0, 3) = Mat3d::Identity() * dt;
  if (config_.use_imu_accel) {
    F.block<3, 3>(3, 6) = -Rwb * math::skew(accel_body) * dt;
    F.block<3, 3>(3, 12) = -Rwb * dt;
  }
  F.block<3, 3>(6, 9) = -Mat3d::Identity() * dt;

  // 过程噪声 Q（对角近似）
  Eigen::Matrix<double, 15, 15> Q = Eigen::Matrix<double, 15, 15>::Zero();
  Q.block<3, 3>(3, 3).diagonal().setConstant(config_.accel_noise * config_.accel_noise * dt * dt);
  Q.block<3, 3>(6, 6).diagonal().setConstant(config_.gyro_noise * config_.gyro_noise * dt * dt);
  Q.block<3, 3>(9, 9).diagonal().setConstant(config_.gyro_bias_noise * config_.gyro_bias_noise * dt);
  Q.block<3, 3>(12, 12).diagonal().setConstant(config_.accel_bias_noise * config_.accel_bias_noise * dt);
  state_.covariance = F * state_.covariance * F.transpose() + Q;
}

bool DrEskf::wheelFresh(double timestamp) const {
  // 允许轻微未来戳（-0.05），并限制相对 IMU 时间差不超过 wheel_timeout
  return has_can_ && (timestamp - last_can_.timestamp) >= -0.05 &&
         std::abs(timestamp - last_can_.timestamp) <= config_.wheel_timeout;
}

bool DrEskf::gnssFresh(double timestamp, const GnssData& gnss) const {
  return has_gnss_ && (timestamp - gnss.timestamp) >= -0.05 &&
         std::abs(timestamp - gnss.timestamp) <= config_.gnss_timeout;
}

void DrEskf::correctWheel(double timestamp) {
  if (!wheelFresh(timestamp)) {
    ++stale_wheel_count_;
    return;
  }

  // 稳定模式：用当前 yaw 把轮速投到世界系，作为世界系速度观测。
  // 轮速只修正速度；方向由 GNSS heading/IMU 陀螺维护，避免轮速残差反向注入姿态。
  const double yaw = math::yawFromQuat(state_.orientation);
  const double cy = std::cos(yaw);
  const double sy = std::sin(yaw);
  const Vec3d measured_velocity(last_can_.speed_mps * cy,
                                last_can_.speed_mps * sy,
                                0.0);
  const Vec3d residual = measured_velocity - state_.velocity;
  if (!finiteVec(measured_velocity) || !finiteVec(residual)) {
    return;
  }

  // 粗门限只防止异常观测/已发散状态继续向滤波器注入 NaN 或极大修正。
  const double horizontal_error = residual.head<2>().norm();
  const double horizontal_gate = std::max(8.0, 3.0 * std::abs(last_can_.speed_mps) + 3.0);
  const double vertical_gate = std::max(5.0, 2.0 * std::abs(last_can_.speed_mps) + 3.0);
  if (horizontal_error > horizontal_gate || std::abs(residual.z()) > vertical_gate) {
    return;
  }

  Eigen::Matrix<double, 3, 15> H = Eigen::Matrix<double, 3, 15>::Zero();
  H.block<3, 3>(0, 3) = Mat3d::Identity();

  Mat3d R_yaw = Mat3d::Identity();
  R_yaw(0, 0) = cy;
  R_yaw(0, 1) = -sy;
  R_yaw(1, 0) = sy;
  R_yaw(1, 1) = cy;

  Eigen::Matrix3d R_body = Eigen::Matrix3d::Zero();
  R_body(0, 0) = config_.wheel_velocity_noise * config_.wheel_velocity_noise;
  R_body(1, 1) = config_.nonholonomic_noise * config_.nonholonomic_noise;
  R_body(2, 2) = config_.nonholonomic_noise * config_.nonholonomic_noise;
  const Eigen::Matrix3d R = R_yaw * R_body * R_yaw.transpose();

  if (update(residual, H, R)) {
    ++wheel_update_count_;
  }
}

void DrEskf::correctGnss(const GnssData& gnss) {
  // 航向：低速时车体航向与 GNSS heading 可能不一致，需车速门限
  const double heading_speed = has_can_ ? std::abs(last_can_.speed_mps) : std::abs(gnss.speed_mps);
  if (config_.use_gnss_heading && gnss.heading_valid &&
      heading_speed >= config_.min_wheel_speed_for_heading) {
    Eigen::Matrix<double, 1, 15> H = Eigen::Matrix<double, 1, 15>::Zero();
    H(0, 8) = 1.0;  // 观测直接约束误差姿态的 yaw 分量 δθ_z
    Eigen::Matrix<double, 1, 1> R;
    const double std_rad = config_.gnss_heading_noise_deg * kDegToRad;
    R(0, 0) = std_rad * std_rad;
    Eigen::Matrix<double, 1, 1> residual;
    residual(0) = math::normalizeAngle(gnss.heading_rad - math::yawFromQuat(state_.orientation));
    if (update(residual, H, R)) {
      ++gnss_heading_update_count_;
    }
  }

  // ENU 速度：直接观测世界系速度
  if (config_.use_gnss_velocity && gnss.velocity_valid) {
    Eigen::Matrix<double, 3, 15> H = Eigen::Matrix<double, 3, 15>::Zero();
    H.block<3, 3>(0, 3) = Mat3d::Identity();
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity() * config_.gnss_velocity_noise * config_.gnss_velocity_noise;
    const Vec3d residual = gnss.velocity_enu - state_.velocity;
    if (update(residual, H, R)) {
      ++gnss_velocity_update_count_;
    }
  }

  // ENU 位置：需已建立局部原点
  if (config_.use_gnss_position && gnss.position_valid && origin_set_) {
    const Vec3d pos = llaToEnu(gnss.latitude, gnss.longitude, gnss.altitude, origin_lat_rad_,
                               origin_lon_rad_, origin_ecef_);
    Eigen::Matrix<double, 3, 15> H = Eigen::Matrix<double, 3, 15>::Zero();
    H.block<3, 3>(0, 0) = Mat3d::Identity();
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity() * config_.gnss_position_noise * config_.gnss_position_noise;
    if (update(pos - state_.position, H, R)) {
      ++gnss_position_update_count_;
    }
  }
}

bool DrEskf::update(const Eigen::VectorXd& residual, const Eigen::MatrixXd& H,
                    const Eigen::MatrixXd& R) {
  if (residual.size() == 0 || H.rows() != residual.size() || H.cols() != 15 ||
      R.rows() != residual.size() || R.cols() != residual.size()) {
    return false;
  }
  if (!finiteEigen(residual) || !finiteEigen(H) || !finiteEigen(R) ||
      !finiteEigen(state_.covariance)) {
    return false;
  }

  const Eigen::MatrixXd S = H * state_.covariance * H.transpose() + R;
  if (!finiteEigen(S)) {
    return false;
  }

  Eigen::LDLT<Eigen::MatrixXd> ldlt(S);
  if (ldlt.info() != Eigen::Success) {
    return false;
  }

  const Eigen::MatrixXd PHt = state_.covariance * H.transpose();
  const Eigen::MatrixXd K = ldlt.solve(PHt.transpose()).transpose();
  const Eigen::Matrix<double, 15, 1> dx = K * residual;
  if (!finiteEigen(K) || !finiteEigen(dx)) {
    return false;
  }

  const Eigen::Matrix<double, 15, 15> I = Eigen::Matrix<double, 15, 15>::Identity();
  Eigen::Matrix<double, 15, 15> new_covariance =
      (I - K * H) * state_.covariance * (I - K * H).transpose() + K * R * K.transpose();
  new_covariance = 0.5 * (new_covariance + new_covariance.transpose());
  if (!finiteEigen(new_covariance)) {
    return false;
  }

  injectError(dx);
  state_.covariance = new_covariance;
  return true;
}

void DrEskf::injectError(const Eigen::Matrix<double, 15, 1>& dx) {
  state_.position += dx.segment<3>(0);
  state_.velocity += dx.segment<3>(3);
  // 小角度误差右乘到名义姿态：q ← q ⊗ δq(δθ)
  state_.orientation = (state_.orientation * math::deltaQuat(dx.segment<3>(6), 1.0)).normalized();
  state_.gyro_bias += dx.segment<3>(9);
  state_.accel_bias += dx.segment<3>(12);
}

DrHealth DrEskf::health(double now) const {
  DrHealth health;
  health.has_imu = has_last_imu_;
  health.has_wheel = has_can_;
  health.has_gnss = has_gnss_;
  health.gnss_heading_used = config_.use_gnss && config_.use_gnss_heading && has_gnss_ && last_gnss_.heading_valid;
  health.gnss_velocity_used = config_.use_gnss && config_.use_gnss_velocity && has_gnss_ && last_gnss_.velocity_valid;
  health.gnss_position_used = config_.use_gnss && config_.use_gnss_position && has_gnss_ && last_gnss_.position_valid;
  health.wheel_used = config_.use_wheel && has_can_;
  health.last_update_time = state_.timestamp;
  health.wheel_age = has_can_ ? std::abs(now - last_can_.timestamp) : 1e9;
  health.gnss_age = has_gnss_ ? std::abs(now - last_gnss_.timestamp) : 1e9;
  health.imu_count = imu_count_;
  health.can_count = can_count_;
  health.gnss_count = gnss_count_;
  health.predict_count = predict_count_;
  health.wheel_update_count = wheel_update_count_;
  health.gnss_heading_update_count = gnss_heading_update_count_;
  health.gnss_velocity_update_count = gnss_velocity_update_count_;
  health.gnss_position_update_count = gnss_position_update_count_;
  health.invalid_imu_count = invalid_imu_count_;
  health.nonpositive_imu_dt_count = nonpositive_imu_dt_count_;
  health.stale_wheel_count = stale_wheel_count_;
  health.stale_gnss_count = stale_gnss_count_;
  return health;
}

OdomResult DrEskf::odometry() const {
  OdomResult odom;
  odom.timestamp = state_.timestamp;
  odom.position = state_.position;
  odom.velocity = state_.velocity;
  odom.orientation = state_.orientation;
  odom.valid = state_.initialized;
  // 6x6：位置协方差 ← δp；姿态协方差 ← δθ（索引 6:9）
  odom.covariance.setIdentity();
  odom.covariance.block<3, 3>(0, 0) = state_.covariance.block<3, 3>(0, 0);
  odom.covariance.block<3, 3>(3, 3) = state_.covariance.block<3, 3>(6, 6);
  return odom;
}

Vec3d llaToEcef(double lat_deg, double lon_deg, double alt) {
  const double lat = lat_deg * kDegToRad;
  const double lon = lon_deg * kDegToRad;
  const double sin_lat = std::sin(lat);
  const double cos_lat = std::cos(lat);
  const double sin_lon = std::sin(lon);
  const double cos_lon = std::cos(lon);
  const double N = kEarthA / std::sqrt(1.0 - kEarthE2 * sin_lat * sin_lat);
  return Vec3d((N + alt) * cos_lat * cos_lon,
               (N + alt) * cos_lat * sin_lon,
               (N * (1.0 - kEarthE2) + alt) * sin_lat);
}

Vec3d llaToEnu(double lat_deg, double lon_deg, double alt, double origin_lat_rad,
               double origin_lon_rad, const Vec3d& origin_ecef) {
  const Vec3d d = llaToEcef(lat_deg, lon_deg, alt) - origin_ecef;
  const double sin_lat = std::sin(origin_lat_rad);
  const double cos_lat = std::cos(origin_lat_rad);
  const double sin_lon = std::sin(origin_lon_rad);
  const double cos_lon = std::cos(origin_lon_rad);
  // 标准 ECEF→ENU 旋转
  Vec3d enu;
  enu.x() = -sin_lon * d.x() + cos_lon * d.y();
  enu.y() = -sin_lat * cos_lon * d.x() - sin_lat * sin_lon * d.y() + cos_lat * d.z();
  enu.z() = cos_lat * cos_lon * d.x() + cos_lat * sin_lon * d.y() + sin_lat * d.z();
  return enu;
}

}  // namespace dr_odometry
