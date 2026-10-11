#pragma once

/**
 * @file eskf.hpp
 * @brief 15 维误差状态卡尔曼滤波（p/v/θ/bg/ba）
 *
 * 事件驱动（时间序）：
 *   processEvent(Imu)    → predict（中值积分）
 *   processEvent(Wheel/Gnss/Pose) → propagateTo(stamp)（IMU 输入 ZOH）→ 观测更新
 *
 * 观测模型（全部开关配置，见 types.hpp 的 DrConfig）：
 *   轮速      v_body = (speed, 0, 0)             → [v, θ]
 *   GNSS 姿态 yaw/pitch/roll                     → [θ]
 *   GNSS 加速度 acc（world/body 约定）           → [θ, ba]
 *   GNSS 速度/位置（可选）                        → [v] / [p]
 *   定位结果  map->base + twist（跳变判断保护）   → [p, θ, v]
 *
 * 右扰动：q ← q ⊗ δq(δθ)；update 用 Joseph 形式。
 */

#include "dr_odometry_v2/core/event_queue.hpp"
#include "dr_odometry_v2/core/pose_gate.hpp"
#include "dr_odometry_v2/core/types.hpp"

namespace dr_odometry_v2 {

class Eskf {
 public:
  bool initialize(const DrConfig& config);
  const DrConfig& config() const { return config_; }

  /// 时间序事件处理（stamp 必须升序，由 EventQueue 保证）
  void processEvent(const Event& event);

  /// 输出：传播到 t_out（IMU 输入 ZOH）后导出
  OdomResult odometry(double t_out) const;

  DrHealth health(double now) const;
  const DrState& state() const { return state_; }

  /// 外部强制重置（冷启动/测试）；下一帧 IMU 只对齐时间
  void reset(const DrState& state);

  // ---- 观测模型（公开便于单测直接验证）----
  void observeWheel(const WheelSample& sample);
  void observeGnssAttitude(const GnssSample& sample);
  void observeGnssAccel(const GnssSample& sample);
  void observeGnssVelocity(const GnssSample& sample);
  void observeGnssPosition(const GnssSample& sample);
  void observePose(const PoseSample& sample);

 private:
  void predict(const ImuSample& imu_mid, double dt);
  void propagateTo(double stamp);
  bool update(const Eigen::VectorXd& residual, const Eigen::MatrixXd& H,
              const Eigen::MatrixXd& R);
  void injectError(const Vec15d& dx);
  void applyPoseStep(const PoseSample& sample);
  bool wheelFresh(double stamp) const;
  bool gnssFresh(double stamp) const;
  bool poseFresh(double stamp) const;

  DrConfig config_;
  DrState state_;
  ImuSample last_imu_;
  WheelSample last_wheel_;
  GnssSample last_gnss_;
  PoseSample last_pose_;
  bool has_last_imu_ = false;
  bool has_wheel_ = false;
  bool has_gnss_ = false;
  bool has_pose_ = false;

  PoseGate pose_gate_;

  // 姿态基准（relative 模式）：q_obs = datum_q_ ⊗ (gnss_base_q_⁻¹ ⊗ q_gnss)
  bool gnss_baseline_set_ = false;
  Quatd gnss_base_q_ = Quatd::Identity();  ///< 基准时刻的 GNSS 姿态
  Quatd datum_q_ = Quatd::Identity();      ///< 基准时刻的 DR 姿态（重定位/pose step 后同步）
  bool map_datum_set_ = false;             ///< 首帧定位观测已锚定 map 基准

  // LLA→ENU 原点（首帧有效 GNSS 位置）
  bool origin_set_ = false;
  double origin_lat_deg_ = 0.0;
  double origin_lon_deg_ = 0.0;
  Vec3d origin_ecef_ = Vec3d::Zero();

  DrHealth counters_;
};

}  // namespace dr_odometry_v2
