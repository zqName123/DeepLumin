#pragma once

/**
 * @file failure_detector.hpp
 * @brief 快速失效判定：拒因分类 / 出图快路径 / 无修正看门狗 / 超时
 *
 * 与状态机的分工（每个判定只在一个模块做一次）：
 *   - state_machine：状态转移与失败/退化/恢复计数（阈值触发转移）
 *   - failure_detector：活性信号（出图连击、无修正、超时、SLAM 断流）
 *     节点把信号翻译为状态机事件（forceTrackingLost / forceLocalizationFailure）
 */

#include <string>

#include "localization_v3/core/quality_gate.hpp"

namespace localization_v3 {

/// GM 拒绝原因分类
enum class RejectClass {
  kNone = 0,
  kMapMiss,      ///< 出图 / 地图不可用（快路径判失效）
  kQuality,      ///< 匹配质量不达标 / 跳变不合理
  kTransient,    ///< 瞬态（数据未到等），轻计数
};

struct FailureDetectorConfig {
  int map_miss_count = 3;                  ///< 出图连击阈值（≈0.3s @10Hz）
  double no_correction_timeout_sec = 3.0;  ///< 无有效修正看门狗
  double lost_timeout_sec = 3.0;           ///< TRACKING_LOST 未恢复超时
  double slam_timeout_sec = 0.5;           ///< SLAM 断流超时
};

class FailureDetector {
 public:
  /// GM 拒因字符串 → 分类（与 global_matcher_v2 的 reject_reason 对应）
  static RejectClass classifyRejectReason(const std::string& reason);

  void configure(const FailureDetectorConfig& config) { config_ = config; }
  const FailureDetectorConfig& config() const { return config_; }
  void reset();

  // ---- 事件喂入 ----
  void onGmResult(bool success, QualityLevel quality, const std::string& reject_reason,
                  double now_sec);
  void onCorrectionAccepted(double now_sec);  ///< 任意位姿写入成功（含人工位姿）
  void onTrackingLost(double now_sec);
  void onTrackingRecovered(double now_sec);
  void onSlamUpdate(double now_sec);

  // ---- 活性信号 ----
  bool mapMissExceeded() const { return map_miss_streak_ >= config_.map_miss_count; }
  bool noCorrectionTimeout(double now_sec) const;
  bool lostTimeout(double now_sec) const;
  bool slamTimeout(double now_sec) const;

  // ---- 查询（状态发布 / 日志）----
  int mapMissStreak() const { return map_miss_streak_; }
  RejectClass lastRejectClass() const { return last_reject_class_; }
  const std::string& lastRejectReason() const { return last_reject_reason_; }
  double lastCorrectionStamp() const { return last_correction_stamp_; }

  /// 当前最可信的失效原因描述（无失效返回空串）
  std::string failureReason(double now_sec) const;

 private:
  FailureDetectorConfig config_;

  int map_miss_streak_ = 0;
  RejectClass last_reject_class_ = RejectClass::kNone;
  std::string last_reject_reason_;

  double last_correction_stamp_ = -1.0;
  double last_slam_stamp_ = -1.0;
  double tracking_lost_since_ = -1.0;
};

}  // namespace localization_v3
