#pragma once

/**
 * @file dr_eskf.hpp
 * @brief 15 维误差状态卡尔曼滤波（ESKF）航位推算核心。
 *
 * 线程模型：由节点在单线程回调中按序调用 feedImu / feedCan / feedGnss，
 * 非线程安全；预测与观测更新以 IMU 时间为时钟主轴。
 *
 * 典型调用时序：
 *   initialize(config)
 *   循环：
 *     feedCan / feedGnss   → 只缓存最新观测（GNSS 可定 ENU 原点 / 预置航向）
 *     feedImu              → predict；若开关与新鲜度允许则 correctWheel / correctGnss
 *   odometry() / health()  → 节点定时取出发布
 */

#include "dr_odometry/core/types.hpp"

namespace dr_odometry {

class DrEskf {
 public:
  /** @brief 按配置重置名义状态、协方差块与缓存标志。 */
  bool initialize(const DrConfig& config);

  /**
   * @brief IMU 驱动主循环：校验 → 首帧初始化 → 中值预测 →（可选）轮速/GNSS 更新。
   * 非有限数据直接丢弃；dt≤0 或乱序只更新 last_imu_。
   */
  void feedImu(const ImuData& imu);

  /** @brief 缓存最新有效 CAN 车速（actual 修正在 feedImu 内 correctWheel）。 */
  void feedCan(const CanData& can);

  /**
   * @brief 缓存 GNSS；首帧有效位置锁定 LLA→ENU 原点；
   * 若滤波尚未初始化且航向有效，可用 GNSS heading 预置 yaw。
   */
  void feedGnss(const GnssData& gnss);

  /** @brief 外部强制重置名义状态（如重定位注入），清空 last_imu 以便重新对齐时间。 */
  void reset(const DrState& state);

  const DrState& state() const { return state_; }
  /** @brief 汇总传感器是否到达及相对 now 的年龄。 */
  DrHealth health(double now) const;
  /** @brief 导出可发布的 OdomResult；initialized 前 valid=false。 */
  OdomResult odometry() const;

 private:
  void initializeFromImu(const ImuData& imu);
  /** @brief 标称状态 IMU 积分 + 误差状态协方差传播 F、Q。 */
  void predict(const ImuData& imu, double dt);
  /** @brief 车体速度观测：纵向=车速，侧向/垂向≈0（非完整约束）。 */
  void correctWheel(double timestamp);
  /** @brief 按配置更新航向 / ENU 速度 / ENU 位置。 */
  void correctGnss(const GnssData& gnss);
  /** @brief 标准线性卡尔曼更新 + Joseph 形式协方差，再 injectError。 */
  bool update(const Eigen::VectorXd& residual, const Eigen::MatrixXd& H,
              const Eigen::MatrixXd& R);
  /** @brief 将误差状态注入名义状态（姿态用小角度右乘）。 */
  void injectError(const Eigen::Matrix<double, 15, 1>& dx);
  bool wheelFresh(double timestamp) const;
  bool gnssFresh(double timestamp, const GnssData& gnss) const;

  DrConfig config_;
  DrState state_;
  ImuData last_imu_;
  CanData last_can_;
  GnssData last_gnss_;
  bool has_last_imu_ = false;
  bool has_can_ = false;
  bool has_gnss_ = false;

  std::uint64_t imu_count_ = 0;
  std::uint64_t can_count_ = 0;
  std::uint64_t gnss_count_ = 0;
  std::uint64_t predict_count_ = 0;
  std::uint64_t wheel_update_count_ = 0;
  std::uint64_t gnss_heading_update_count_ = 0;
  std::uint64_t gnss_velocity_update_count_ = 0;
  std::uint64_t gnss_position_update_count_ = 0;
  std::uint64_t invalid_imu_count_ = 0;
  std::uint64_t nonpositive_imu_dt_count_ = 0;
  std::uint64_t stale_wheel_count_ = 0;
  std::uint64_t stale_gnss_count_ = 0;

  // LLA→ENU 局部原点（首次 position_valid 的 GNSS）
  bool origin_set_ = false;
  double origin_lat_rad_ = 0.0;
  double origin_lon_rad_ = 0.0;
  double origin_alt_ = 0.0;
  Vec3d origin_ecef_ = Vec3d::Zero();
};

/** @brief WGS84 近似：经纬高(度,度,米) → ECEF。 */
Vec3d llaToEcef(double lat_deg, double lon_deg, double alt);

/** @brief ECEF 差分后投影到以原点为中心的 ENU。 */
Vec3d llaToEnu(double lat_deg, double lon_deg, double alt, double origin_lat_rad,
               double origin_lon_rad, const Vec3d& origin_ecef);

}  // namespace dr_odometry
