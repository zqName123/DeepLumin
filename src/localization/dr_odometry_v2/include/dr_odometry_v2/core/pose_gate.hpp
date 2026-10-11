#pragma once

/**
 * @file pose_gate.hpp
 * @brief 定位观测跳变判断（防伪跳变污染滤波器）
 *
 * 定位端 map->base 的跳变分两种：
 *   真实修正（GM 校正漂移，后续观测稳定在新水平）
 *   伪跳变  （几何相似误匹配，一次性/来回跳）
 *
 * 三档判定（创新量 = 当前滤波位姿 vs 观测位姿）：
 *   小跳  Δ≤soft           → 直接接收（正常更新）
 *   大跳  soft~hard        → 连续 confirm_frames 帧互相一致才接受（台阶修正）
 *   超限  >hard            → 拒收（伪跳变特征）
 */

#include <deque>

#include "dr_odometry_v2/core/math_utils.hpp"
#include "dr_odometry_v2/core/types.hpp"

namespace dr_odometry_v2 {

enum class GateVerdict {
  kAccept = 0,      ///< 小跳：正常观测更新
  kAcceptStep,      ///< 大跳确认通过：台阶修正（只修 p/θ）
  kConfirmPending,  ///< 大跳：等待后续一致帧
  kReject,          ///< 超限 / 确认缓冲内不一致
};

class PoseGate {
 public:
  struct Config {
    double soft_trans = 0.5;
    double soft_yaw_deg = 2.0;
    double hard_trans = 5.0;
    double hard_yaw_deg = 15.0;
    int confirm_frames = 3;
    double consistency_trans = 0.5;
    double consistency_yaw_deg = 2.0;
  };

  void setConfig(const Config& config) {
    config_ = config;
    reset();
  }
  const Config& config() const { return config_; }
  void reset() { candidates_.clear(); }
  std::size_t pendingCount() const { return candidates_.size(); }

  /**
   * @brief 判定一次定位观测
   * @param current   当前滤波位姿（世界系）
   * @param candidate 观测位姿
   */
  GateVerdict evaluate(const Pose3d& current, const Pose3d& candidate, double now_sec);

 private:
  struct Candidate {
    Pose3d pose;
    double stamp = 0.0;
  };
  Config config_;
  std::deque<Candidate> candidates_;
};

}  // namespace dr_odometry_v2
