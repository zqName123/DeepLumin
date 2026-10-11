#include "dr_odometry_v2/core/eskf.hpp"

#include <algorithm>
#include <cmath>

namespace dr_odometry_v2 {
namespace {

using math::deg2rad;
using math::finiteEigen;
using math::finiteVec;
using math::skew;

/// 长断流阈值：超过该间隙只对齐时间不积分（间隙内运动不可知，防 ZOH 假运动）
constexpr double kMaxIntegrateGapSec = 1.0;

/// 航向约定换算：GPCHC 罗盘（北基准顺时针）→ 世界系 yaw（东基准逆时针）
double yawFromHeading(double heading, const GnssConfig& cfg) {
  const double yaw = (cfg.heading_convention == "compass") ? (math::kPi / 2.0 - heading) : heading;
  return yaw + math::deg2rad(cfg.heading_offset_deg);
}

/// 标称状态 IMU 积分（可复用于前向外推）
void integrateNominal(DrState& s, const ImuSample& imu, double dt, const ImuConfig& cfg) {  const Vec3d omega = imu.gyro - s.gyro_bias;
  const Vec3d accel_body = imu.accel - s.accel_bias;
  const Mat3d Rwb = s.orientation.toRotationMatrix();
  Vec3d accel_world = Vec3d::Zero();
  if (cfg.use_accel) {
    accel_world = Rwb * accel_body + Vec3d(0.0, 0.0, -cfg.gravity);
  }
  s.position += s.velocity * dt + 0.5 * accel_world * dt * dt;
  s.velocity += accel_world * dt;
  s.orientation = (s.orientation * math::deltaQuat(omega, dt)).normalized();
  s.timestamp = imu.stamp;
}

}  // namespace

bool Eskf::initialize(const DrConfig& config) {
  config_ = config;
  state_ = DrState();
  state_.orientation = math::quatFromYaw(deg2rad(config.initial_yaw_deg));
  state_.covariance.setIdentity();
  state_.covariance.block<3, 3>(0, 0) *= 1.0;
  state_.covariance.block<3, 3>(3, 3) *= 1.0;
  state_.covariance.block<3, 3>(6, 6) *= 0.1;
  state_.covariance.block<3, 3>(9, 9) *= 0.01;
  state_.covariance.block<3, 3>(12, 12) *= 0.05;

  PoseGate::Config gate;
  gate.soft_trans = config.localization.jump_soft_trans;
  gate.soft_yaw_deg = config.localization.jump_soft_yaw_deg;
  gate.hard_trans = config.localization.jump_hard_trans;
  gate.hard_yaw_deg = config.localization.jump_hard_yaw_deg;
  gate.confirm_frames = config.localization.jump_confirm_frames;
  gate.consistency_trans = config.localization.jump_confirm_consistency_trans;
  gate.consistency_yaw_deg = config.localization.jump_confirm_consistency_yaw_deg;
  pose_gate_.setConfig(gate);

  has_last_imu_ = false;
  has_wheel_ = has_gnss_ = has_pose_ = false;
  origin_set_ = false;
  gnss_baseline_set_ = false;
  map_datum_set_ = false;
  counters_ = DrHealth();
  return true;
}

void Eskf::reset(const DrState& state) {
  state_ = state;
  state_.orientation.normalize();
  has_last_imu_ = false;  // 下一帧 IMU 只对齐时间
  pose_gate_.reset();
  gnss_baseline_set_ = false;
  map_datum_set_ = false;
}

// ============================== 事件驱动 ==============================

void Eskf::processEvent(const Event& event) {
  switch (event.type) {
    case EventType::kImu: {
      const ImuSample& imu = event.imu;
      ++counters_.imu_count;
      if (!std::isfinite(imu.stamp) || !finiteVec(imu.gyro) || !finiteVec(imu.accel)) {
        ++counters_.invalid_sample_count;
        return;
      }
      if (!state_.initialized) {
        // 首帧：初始化时间轴，不预测
        state_.initialized = true;
        state_.timestamp = imu.stamp;
        last_imu_ = imu;
        has_last_imu_ = true;
        return;
      }
      if (!has_last_imu_) {
        last_imu_ = imu;
        has_last_imu_ = true;
        return;
      }
      // dt 以滤波时间轴为准（state.timestamp 可能已被观测 propagateTo 推进过），
      // 避免重复积分
      double dt = imu.stamp - state_.timestamp;
      if (dt <= 0.0) {
        ++counters_.nonpositive_dt_count;
        last_imu_ = imu;
        return;
      }
      ImuSample mid;
      mid.gyro = 0.5 * (last_imu_.gyro + imu.gyro);
      mid.accel = 0.5 * (last_imu_.accel + imu.accel);
      if (dt > kMaxIntegrateGapSec) {
        // 长断流：间隙内运动不可知，仅对齐时间轴（丢弃该段积分），防 ZOH 假运动
        state_.timestamp = imu.stamp;
      } else {
        // 分步全积分（每步 ≤ max_dt）：max_dt 只限制积分步长，不丢积分时间
        double remain = dt;
        while (remain > 1e-9) {
          const double step = std::min(remain, config_.imu.max_dt);
          mid.stamp = state_.timestamp + step;
          predict(mid, step);
          remain -= step;
        }
      }
      last_imu_ = imu;
      return;
    }

    case EventType::kWheel: {
      ++counters_.wheel_count;
      last_wheel_ = event.wheel;
      has_wheel_ = true;
      if (!state_.initialized) {
        return;
      }
      propagateTo(event.stamp);
      observeWheel(event.wheel);
      return;
    }

    case EventType::kGnss: {
      ++counters_.gnss_count;
      last_gnss_ = event.gnss;
      has_gnss_ = true;
      // 滤波未初始化时的朝向预置：仅 absolute 模式（世界系锚 ENU）。
      // relative 模式严禁预置——世界系=初始车体姿态（identity），预置会把 GNSS
      // 绝对姿态漏进来污染初始方向（实测表现为 odom x 轴偏离车头 100°+）。
      if (!state_.initialized && config_.gnss.enabled && config_.gnss.use_attitude &&
          config_.gnss.attitude_mode == "absolute" && event.gnss.attitude_valid) {
        state_.orientation = math::quatFromRpy(
            event.gnss.roll, event.gnss.pitch,
            yawFromHeading(event.gnss.heading, config_.gnss));
      }
      if (!state_.initialized) {
        return;
      }
      propagateTo(event.stamp);
      if (!config_.gnss.enabled) {
        return;
      }
      if (config_.gnss.use_attitude) {
        observeGnssAttitude(event.gnss);
      }
      if (config_.gnss.use_accel) {
        observeGnssAccel(event.gnss);
      }
      if (config_.gnss.use_velocity) {
        observeGnssVelocity(event.gnss);
      }
      if (config_.gnss.use_position) {
        observeGnssPosition(event.gnss);
      }
      return;
    }

    case EventType::kPose: {
      ++counters_.pose_count;
      last_pose_ = event.pose;
      has_pose_ = true;
      if (!state_.initialized) {
        return;
      }
      propagateTo(event.stamp);
      observePose(event.pose);
      return;
    }
  }
}

void Eskf::propagateTo(double stamp) {
  if (!has_last_imu_ || !state_.initialized) {
    return;
  }
  while (state_.timestamp < stamp) {
    double dt = stamp - state_.timestamp;
    if (dt > config_.imu.max_dt) {
      dt = config_.imu.max_dt;
    }
    // 观测时刻与 IMU 时刻之间：IMU 输入 ZOH
    ImuSample hold = last_imu_;
    hold.stamp = state_.timestamp + dt;
    predict(hold, dt);
  }
}

void Eskf::predict(const ImuSample& imu_mid, double dt) {
  const Vec3d accel_body = imu_mid.accel - state_.accel_bias;
  const Mat3d Rwb = state_.orientation.toRotationMatrix();
  integrateNominal(state_, imu_mid, dt, config_.imu);

  // 误差状态传播 F、Q（块序 [p|v|θ|bg|ba]）
  Mat15d F = Mat15d::Identity();
  F.block<3, 3>(0, 3) = Mat3d::Identity() * dt;
  if (config_.imu.use_accel) {
    F.block<3, 3>(3, 6) = -Rwb * skew(accel_body) * dt;
    F.block<3, 3>(3, 12) = -Rwb * dt;
  }
  F.block<3, 3>(6, 9) = -Mat3d::Identity() * dt;

  Mat15d Q = Mat15d::Zero();
  Q.block<3, 3>(3, 3).diagonal().setConstant(config_.imu.accel_noise * config_.imu.accel_noise * dt * dt);
  Q.block<3, 3>(6, 6).diagonal().setConstant(config_.imu.gyro_noise * config_.imu.gyro_noise * dt * dt);
  Q.block<3, 3>(9, 9).diagonal().setConstant(config_.imu.gyro_bias_noise * config_.imu.gyro_bias_noise * dt);
  Q.block<3, 3>(12, 12).diagonal().setConstant(config_.imu.accel_bias_noise * config_.imu.accel_bias_noise * dt);
  state_.covariance = F * state_.covariance * F.transpose() + Q;
  ++counters_.predict_count;
}

// ============================== 观测模型 ==============================

void Eskf::observeWheel(const WheelSample& s) {
  if (!config_.wheel.enabled || !s.valid || !std::isfinite(s.speed_mps)) {
    return;
  }
  if (!wheelFresh(s.stamp)) {
    ++counters_.stale_wheel_count;
    return;
  }
  const Mat3d R = state_.orientation.toRotationMatrix();
  const Vec3d v_body = R.transpose() * state_.velocity;
  const Vec3d z(s.speed_mps, 0.0, 0.0);  // (speed, 0, 0) 形式约束
  const Vec3d r = z - v_body;
  if (!finiteVec(r)) {
    return;
  }
  // 粗门限防发散注入
  const double gate = std::max(8.0, 3.0 * std::abs(s.speed_mps) + 3.0);
  if (r.head<2>().norm() > gate || std::abs(r.z()) > gate) {
    return;
  }

  Eigen::Matrix<double, 3, 15> H = Eigen::Matrix<double, 3, 15>::Zero();
  H.block<3, 3>(0, 3) = R.transpose();
  H.block<3, 3>(0, 6) = skew(R.transpose() * state_.velocity);

  Mat3d Rm = Mat3d::Zero();
  Rm(0, 0) = config_.wheel.velocity_noise * config_.wheel.velocity_noise;
  Rm(1, 1) = config_.wheel.nonholonomic_noise * config_.wheel.nonholonomic_noise;
  Rm(2, 2) = config_.wheel.nonholonomic_noise * config_.wheel.nonholonomic_noise;

  if (update(r, H, Rm)) {
    ++counters_.wheel_update_count;
  }
}

void Eskf::observeGnssAttitude(const GnssSample& s) {
  if (!s.attitude_valid) {
    return;
  }
  if (!gnssFresh(s.stamp)) {
    ++counters_.stale_gnss_count;
    return;
  }
  const Quatd q_gnss =
      math::quatFromRpy(s.roll, s.pitch, yawFromHeading(s.heading, config_.gnss));

  Quatd q_meas;
  if (config_.gnss.attitude_mode == "absolute") {
    q_meas = q_gnss;
  } else {
    // relative：只用姿态变化量（与 LIO 一致）；世界系=初始姿态系，绝对基准消去
    if (!gnss_baseline_set_) {
      gnss_base_q_ = q_gnss;
      datum_q_ = state_.orientation;
      gnss_baseline_set_ = true;
    }
    // 静止重锚：吸收安装角/初始对准误差，起步无台阶（lio 同款 rezero）
    if (config_.gnss.rezero_when_static &&
        state_.velocity.norm() < config_.gnss.static_speed_eps) {
      gnss_base_q_ = q_gnss;
      datum_q_ = state_.orientation;
    }
    const Quatd dq = gnss_base_q_.conjugate() * q_gnss;  // 姿态变化量
    q_meas = (datum_q_ * dq).normalized();
  }

  const Vec3d r = math::logQuat(state_.orientation, q_meas);  // Log(R̂^T R_obs) ≈ δθ
  if (!finiteVec(r)) {
    return;
  }

  Eigen::Matrix<double, 3, 15> H = Eigen::Matrix<double, 3, 15>::Zero();
  H.block<3, 3>(0, 6) = Mat3d::Identity();

  const double s_roll = deg2rad(config_.gnss.attitude_noise_roll_deg);
  const double s_pitch = deg2rad(config_.gnss.attitude_noise_pitch_deg);
  double s_yaw = deg2rad(config_.gnss.attitude_noise_yaw_deg);
  if (s.speed_mps < config_.gnss.min_speed_for_heading) {
    s_yaw *= 10.0;  // 低速时航向不可信，拉大噪声
  }
  Mat3d Rm = Mat3d::Zero();
  Rm(0, 0) = s_roll * s_roll;
  Rm(1, 1) = s_pitch * s_pitch;
  Rm(2, 2) = s_yaw * s_yaw;

  if (update(r, H, Rm)) {
    ++counters_.gnss_attitude_update_count;
  }
}

void Eskf::observeGnssAccel(const GnssSample& s) {
  if (!s.accel_valid || !has_last_imu_) {
    return;
  }
  if (!gnssFresh(s.stamp)) {
    ++counters_.stale_gnss_count;
    return;
  }
  const Mat3d R = state_.orientation.toRotationMatrix();
  Eigen::Matrix<double, 3, 15> H = Eigen::Matrix<double, 3, 15>::Zero();
  Vec3d r;
  if (config_.gnss.accel_frame == "body") {
    // 机体系比力观测：h = a_imu - b_a
    r = s.accel - (last_imu_.accel - state_.accel_bias);
    H.block<3, 3>(0, 12) = -Mat3d::Identity();
  } else {
    // 世界系加速度观测：h = R(a_imu - b_a) + g
    const Vec3d u = last_imu_.accel - state_.accel_bias;
    const Vec3d h = R * u + Vec3d(0.0, 0.0, -config_.imu.gravity);
    r = s.accel - h;
    H.block<3, 3>(0, 6) = -R * skew(u);
    H.block<3, 3>(0, 12) = -R;
  }
  if (!finiteVec(r)) {
    return;
  }
  const double sigma = config_.gnss.accel_noise;
  const Mat3d Rm = Mat3d::Identity() * sigma * sigma;
  if (update(r, H, Rm)) {
    ++counters_.gnss_accel_update_count;
  }
}

void Eskf::observeGnssVelocity(const GnssSample& s) {
  if (!s.velocity_valid) {
    return;
  }
  const Vec3d r = s.velocity_enu - state_.velocity;
  if (!finiteVec(r)) {
    return;
  }
  Eigen::Matrix<double, 3, 15> H = Eigen::Matrix<double, 3, 15>::Zero();
  H.block<3, 3>(0, 3) = Mat3d::Identity();
  const double sigma = config_.gnss.velocity_noise;
  if (update(r, H, Mat3d::Identity() * sigma * sigma)) {
    ++counters_.gnss_velocity_update_count;
  }
}

void Eskf::observeGnssPosition(const GnssSample& s) {
  if (!s.position_valid) {
    return;
  }
  if (!origin_set_) {
    origin_lat_deg_ = s.lat_deg;
    origin_lon_deg_ = s.lon_deg;
    origin_ecef_ = math::llaToEcef(s.lat_deg, s.lon_deg, s.alt);
    origin_set_ = true;
  }
  const Vec3d pos = math::llaToEnu(s.lat_deg, s.lon_deg, s.alt, origin_lat_deg_, origin_lon_deg_,
                                   origin_ecef_);
  const Vec3d r = pos - state_.position;
  if (!finiteVec(r)) {
    return;
  }
  Eigen::Matrix<double, 3, 15> H = Eigen::Matrix<double, 3, 15>::Zero();
  H.block<3, 3>(0, 0) = Mat3d::Identity();
  const double sigma = config_.gnss.position_noise;
  if (update(r, H, Mat3d::Identity() * sigma * sigma)) {
    ++counters_.gnss_position_update_count;
  }
}

void Eskf::observePose(const PoseSample& s) {
  if (!config_.localization.enabled) {
    return;
  }
  if (!poseFresh(s.stamp)) {
    ++counters_.stale_pose_count;
    return;
  }
  // 首帧定位观测：直接锚定 map 基准（"起点=map 起点"假设的自动推广）
  if (!map_datum_set_) {
    applyPoseStep(s);
    map_datum_set_ = true;
    ++counters_.pose_step_count;
    return;
  }
  // 跳变判断：防伪跳变污染
  Pose3d current;
  current.position = state_.position;
  current.orientation = state_.orientation;
  const GateVerdict verdict = pose_gate_.evaluate(current, s.map_base, state_.timestamp);
  switch (verdict) {
    case GateVerdict::kReject:
      ++counters_.pose_reject_count;
      return;
    case GateVerdict::kConfirmPending:
      ++counters_.pose_pending_count;
      return;
    case GateVerdict::kAcceptStep:
      applyPoseStep(s);  // 大跳确认：台阶修正（只修 p/θ，零偏不动）
      ++counters_.pose_step_count;
      return;
    case GateVerdict::kAccept:
      break;
  }

  // 小跳：正常观测更新
  const int rows = s.has_velocity ? 9 : 6;
  Eigen::VectorXd r(rows);
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(rows, 15);
  Eigen::MatrixXd Rm = Eigen::MatrixXd::Identity(rows, rows);

  r.segment<3>(0) = s.map_base.position - state_.position;
  H.block<3, 3>(0, 0) = Mat3d::Identity();
  const double sp = config_.localization.pose_noise_trans;
  Rm.block<3, 3>(0, 0) = Mat3d::Identity() * sp * sp;

  r.segment<3>(3) = math::logQuat(state_.orientation, s.map_base.orientation);
  H.block<3, 3>(3, 6) = Mat3d::Identity();
  const double sa = deg2rad(config_.localization.pose_noise_att_deg);
  Rm.block<3, 3>(3, 3) = Mat3d::Identity() * sa * sa;

  if (s.has_velocity) {
    const Mat3d R = state_.orientation.toRotationMatrix();
    r.segment<3>(6) = s.velocity_body - R.transpose() * state_.velocity;
    H.block<3, 3>(6, 3) = R.transpose();
    H.block<3, 3>(6, 6) = skew(R.transpose() * state_.velocity);
    const double sv = config_.localization.vel_noise;
    Rm.block<3, 3>(6, 6) = Mat3d::Identity() * sv * sv;
  }

  if (update(r, H, Rm)) {
    ++counters_.pose_update_count;
  }
}

void Eskf::applyPoseStep(const PoseSample& s) {
  // 台阶修正：直接置 p/θ（可选 v），零偏与协方差的零偏块保留
  state_.position = s.map_base.position;
  state_.orientation = s.map_base.orientation.normalized();
  if (s.has_velocity) {
    const Mat3d R = state_.orientation.toRotationMatrix();
    state_.velocity = R * s.velocity_body;
  }
  // 姿态基准同步：relative GNSS 只贡献变化量，基准跟到新姿态（跨台阶连续）
  if (gnss_baseline_set_) {
    datum_q_ = state_.orientation;
  }
  // p/θ 协方差收敛到小值，切断与零偏的交叉项；bg/ba 块保留
  Mat15d P = state_.covariance;
  const Mat3d P_bg = P.block<3, 3>(9, 9);
  const Mat3d P_ba = P.block<3, 3>(12, 12);
  P.setZero();
  P.block<3, 3>(0, 0) = Mat3d::Identity() * 1e-4;
  P.block<3, 3>(3, 3) = Mat3d::Identity() * 1e-2;
  P.block<3, 3>(6, 6) = Mat3d::Identity() * 1e-4;
  P.block<3, 3>(9, 9) = P_bg;
  P.block<3, 3>(12, 12) = P_ba;
  state_.covariance = P;
  pose_gate_.reset();
}

// ============================== 卡尔曼更新 ==============================

bool Eskf::update(const Eigen::VectorXd& residual, const Eigen::MatrixXd& H,
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
  const Vec15d dx = K * residual;
  if (!finiteEigen(K) || !finiteEigen(dx)) {
    return false;
  }
  Mat15d new_cov = (Mat15d::Identity() - K * H) * state_.covariance *
                       (Mat15d::Identity() - K * H).transpose() +
                   K * R * K.transpose();
  new_cov = 0.5 * (new_cov + new_cov.transpose());
  if (!finiteEigen(new_cov)) {
    return false;
  }
  injectError(dx);
  state_.covariance = new_cov;
  return true;
}

void Eskf::injectError(const Vec15d& dx) {
  state_.position += dx.segment<3>(0);
  state_.velocity += dx.segment<3>(3);
  state_.orientation = (state_.orientation * math::deltaQuat(dx.segment<3>(6), 1.0)).normalized();
  // 零偏增量限幅：大观测阶跃瞬态下 θ↔bg 耦合会把姿态突变误归因到零偏（假零偏暴走
  // 导致过冲），限幅让零偏平滑学习、免疫瞬态
  const Vec3d dbg = dx.segment<3>(9).cwiseMax(-config_.imu.gyro_bias_step_clamp)
                        .cwiseMin(config_.imu.gyro_bias_step_clamp);
  const Vec3d dba = dx.segment<3>(12).cwiseMax(-config_.imu.accel_bias_step_clamp)
                         .cwiseMin(config_.imu.accel_bias_step_clamp);
  state_.gyro_bias += dbg;
  state_.accel_bias += dba;
}

// ============================== 输出 / 健康 ==============================

OdomResult Eskf::odometry(double t_out) const {
  OdomResult out;
  if (!state_.initialized) {
    return out;
  }
  DrState s = state_;
  // 前向外推到输出时刻（IMU 输入 ZOH，限时）
  if (has_last_imu_ && t_out > s.timestamp && (t_out - s.timestamp) <= config_.output.max_extrapolate) {
    while (s.timestamp < t_out) {
      double dt = t_out - s.timestamp;
      if (dt > config_.imu.max_dt) {
        dt = config_.imu.max_dt;
      }
      ImuSample hold = last_imu_;
      hold.stamp = s.timestamp + dt;
      integrateNominal(s, hold, dt, config_.imu);
    }
  }
  out.timestamp = s.timestamp;
  out.pose.position = s.position;
  out.pose.orientation = s.orientation;
  out.velocity = s.velocity;
  out.valid = true;
  return out;
}

DrHealth Eskf::health(double now) const {
  DrHealth h = counters_;
  h.last_update_time = state_.timestamp;
  (void)now;
  return h;
}

// ============================== 新鲜度 ==============================

bool Eskf::wheelFresh(double stamp) const {
  return std::abs(state_.timestamp - stamp) <= config_.wheel.timeout;
}

bool Eskf::gnssFresh(double stamp) const {
  return std::abs(state_.timestamp - stamp) <= config_.gnss.timeout;
}

bool Eskf::poseFresh(double stamp) const {
  return std::abs(state_.timestamp - stamp) <= config_.localization.max_latency;
}

}  // namespace dr_odometry_v2
